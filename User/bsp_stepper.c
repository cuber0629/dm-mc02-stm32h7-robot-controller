#include "bsp_stepper.h"
#include "main.h"       // 必须包含 main.h 以获取 SystemCoreClock 和 HAL 定义
#include "gpio.h"
#include "delay.h"
#include <stdio.h>

/*
 * 步进电机控制模块说明
 * --------------------
 * 1. 通过 DWT 周期计数器实现微秒级计时，适合 STM32H7 这类高主频 MCU。
 * 2. Stepper_MoveRevs() 只负责写入本次运动的目标参数，不直接发脉冲。
 * 3. Stepper_Update() 采用非阻塞方式调度 STEP 脉冲，需要在主循环中高频调用。
 * 4. speed_delay 表示相邻两个 STEP 脉冲之间的间隔，单位为 us。
 *
 * 典型调用流程：
 * Stepper_Init() -> Stepper_MoveRevs() -> while(1) 中持续调用 Stepper_Update()
 */

/* 步进电机当前的全局运行状态，保存“本次任务”的调度信息 */
StepperMotor_t stepmotor = {
    .speed_delay = 400,      // 默认脉冲间隔 400 us
    .direction = 1,          // 默认方向：1 为正转
    .is_running = 0,         // 上电后默认停止
    .target_steps = 0,       // 尚未分配运动目标
    .current_steps = 0,      // 已输出步数清零
    .steps_per_rev = 400,    // 默认细分配置下每圈 400 步
    .last_toggle_tick = 0    // 上一次输出 STEP 脉冲的时间戳
};

/**
 * @brief 初始化 DWT 周期计数器
 * @note  STM32H7 上必须先解锁 LAR，DWT->CYCCNT 才能正常计数
 */
void DWT_Init(void) {
    // 先打开调试/跟踪单元时钟，否则 DWT 计数器不会工作
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    
    // H7 系列需要写入解锁码，允许访问 DWT 相关寄存器
    DWT->LAR = 0xC5ACCE55; 
    
    // 清零后开启周期计数，后续可直接读取 DWT->CYCCNT 计时
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 微秒级忙等待延时
 * @param nus 需要延时的时间，单位 us
 * @note  这里是阻塞延时，只适合用于生成 STEP 高电平等极短等待场景
 */
void bujindelay_us(uint32_t nus) {
    uint32_t start = DWT->CYCCNT;
    // 根据当前系统主频换算出目标等待的 CPU 周期数
    uint32_t ticks = nus * (SystemCoreClock / 1000000);
    
    // 利用无符号减法自动兼容 CYCCNT 32 位回绕
    while ((DWT->CYCCNT - start) < ticks);
}

/**
 * @brief 初始化步进电机
 * @param steps_per_revolution 电机一圈对应的步数，传 0 时使用默认值 400
 * @note  请在 GPIO 初始化完成后调用，以确保 DIR/PUL 引脚可正常输出
 */
void Stepper_Init(uint32_t steps_per_revolution) {
    // 先初始化 DWT，后面无论是短延时还是非阻塞调度都依赖它
    DWT_Init(); 
    
    // 允许上层传 0 来使用默认步数，避免未配置时出现非法参数
    if (steps_per_revolution == 0) steps_per_revolution = 400;
    stepmotor.steps_per_rev = steps_per_revolution;
    stepmotor.is_running = 0;
    stepmotor.current_steps = 0;
    stepmotor.target_steps = 0;

    // 设置空闲电平：方向给一个确定初值，脉冲脚保持低电平
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PUL_PORT, PUL_PIN, GPIO_PIN_RESET);
    
    // 如需调试输出，可在串口初始化完成后打开下面这行
    // printf("H7 Stepper Init: %lu steps/rev, Clock: %lu MHz\r\n", steps_per_revolution, SystemCoreClock/1000000);
}

/**
 * @brief 配置并启动一次步进运动
 * @param revolutions 目标圈数，大于 0 才会启动
 * @param speed_delay 相邻脉冲间隔，单位 us
 * @param direction   方向控制，1 为正转，0 为反转
 * @note  本函数只写入目标参数，真正的脉冲输出在 Stepper_Update() 中完成
 */
void Stepper_MoveRevs(float revolutions, uint32_t speed_delay, uint8_t direction) {
    // 圈数非法时直接忽略，避免启动一次空运动
    if (revolutions <= 0) return;

    // 将圈数转换为总步数，作为本次运动的终止条件
    // 这里直接截断为 uint32_t，适合“按步数落地”的控制场景
    stepmotor.target_steps = (uint32_t)(revolutions * stepmotor.steps_per_rev);
    stepmotor.speed_delay = speed_delay;
    stepmotor.direction = direction;
    stepmotor.current_steps = 0;
    
    // 先锁定方向，再开始后续脉冲输出
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    // 重置调度参考时间，避免启动后因上次残留时间基准而立即补发脉冲
    stepmotor.last_toggle_tick = DWT->CYCCNT;
    
    // 置位运行标志，允许 Stepper_Update() 开始发脉冲
    stepmotor.is_running = 1;
}

/**
 * @brief 非阻塞步进脉冲调度函数
 * @note  需要在 main 的 while(1) 中尽可能高频调用，函数每次最多输出一个脉冲
 */
void Stepper_Update(void) {
    // 未处于运行态时直接返回，保持主循环轻量
    if (!stepmotor.is_running) return;

    // 已达到目标步数时停止输出
    if (stepmotor.current_steps >= stepmotor.target_steps) {
        stepmotor.is_running = 0;
        // 运动结束后如需提示，可在这里打印；运动过程中不建议频繁打印
        // printf("Move Done\r\n");
        return;
    }

    uint32_t current_cycle = DWT->CYCCNT;
    // 将设定的 us 间隔换算为 CPU 周期数
    uint32_t cycles_needed = stepmotor.speed_delay * (SystemCoreClock / 1000000);

    // 只有达到设定间隔后才输出下一个 STEP 脉冲
    if ((current_cycle - stepmotor.last_toggle_tick) >= cycles_needed) {
        
        // 更新时间基准，为下一次脉冲调度做准备
        stepmotor.last_toggle_tick = current_cycle;

        // 输出一个完整的 STEP 脉冲：拉高、保持、再拉低
        HAL_GPIO_WritePin(PUL_PORT, PUL_PIN, GPIO_PIN_SET);
        
        // H7 翻转 GPIO 很快，增加约 3us 高电平宽度以满足驱动器脉宽要求
        bujindelay_us(3); 
        
        HAL_GPIO_WritePin(PUL_PORT, PUL_PIN, GPIO_PIN_RESET);

        // 每输出一个有效脉冲，电机理论上前进一步
        stepmotor.current_steps++;
    }
}
