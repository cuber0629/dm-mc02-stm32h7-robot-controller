#ifndef __BSP_STEPPER_H
#define __BSP_STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h" // 包含 STM32H7xx_hal.h 和 Pin 定义
#include "gpio.h"

/* STEP/DIR 引脚映射，需与实际硬件连接保持一致 */
#define PUL_PORT GPIOE
#define PUL_PIN  GPIO_PIN_9
#define DIR_PORT GPIOA
#define DIR_PIN  GPIO_PIN_0

/* 步进电机当前控制状态 */
typedef struct {
    uint32_t speed_delay;        // 相邻脉冲的时间间隔，单位 us
    uint8_t  direction;          // 方向位：1 正转，0 反转
    volatile uint8_t is_running; // 运行标志，可能在主循环外被读取
    uint32_t target_steps;       // 本次运动目标步数
    uint32_t current_steps;      // 当前已经输出的步数
    uint32_t steps_per_rev;      // 电机一圈对应的总步数
    uint32_t last_toggle_tick;   // 上次输出脉冲时的 DWT 时间戳
} StepperMotor_t;

extern StepperMotor_t stepmotor;

/* 初始化步进模块并配置默认步数 */
void Stepper_Init(uint32_t steps_per_revolution);

/* 启动一次按圈数计的运动，实际脉冲由 Stepper_Update() 输出 */
void Stepper_MoveRevs(float revolutions, uint32_t speed_delay, uint8_t direction);

/* 非阻塞调度函数，需要在主循环中持续调用 */
void Stepper_Update(void);

/* 微秒级阻塞延时，主要供步进脉冲高电平保持使用 */
void bujindelay_us(uint32_t nus);

#ifdef __cplusplus
}
#endif

#endif