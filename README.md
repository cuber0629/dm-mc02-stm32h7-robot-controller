# 达妙 DM-MC02 STM32H7 机械臂控制器

基于达妙 DM-MC02（STM32H7）的机械臂下位机控制程序。控制器通过两路 FDCAN 总线驱动多关节达妙伺服电机，同时使用 STEP/DIR 脉冲控制步进电机，实现机器人身体升降。工程使用 STM32 HAL，适合在 STM32CubeMX + Keil MDK-ARM 环境中继续开发。

## 功能

- 双 FDCAN 总线：左侧关节和腰部使用 FDCAN2，右侧关节使用 FDCAN1。
- 达妙电机协议封装：支持 MIT、位置、速度和电流/力矩相关控制模式，以及使能、失能、清错、参数读写和反馈解析。
- 最多 17 个逻辑电机：左侧 9 个（含腰部）和右侧 8 个，电机 ID 在两条总线上分别从 `0x01` 开始。
- 角度控制带软件限位，并根据左右侧机械结构进行方向修正。
- 步进电机非阻塞脉冲调度：DWT 微秒计时，支持按圈数、脉冲间隔和方向运动。
- USART1 中断接收文本命令，并返回 `ACK`/`ERROR` 结果，便于上位机或串口工具调试。
- TIM3 周期轮询达妙电机反馈。

## 硬件与默认参数

| 项目 | 配置 |
| --- | --- |
| MCU | STM32H723VGT6，Cortex-M7 |
| 主频 | 480 MHz（外部 24 MHz HSE） |
| FDCAN1 | PD0 RX / PD1 TX，默认经典 CAN 1 Mbps |
| FDCAN2 | PB5 RX / PB6 TX，默认经典 CAN 1 Mbps |
| USART1 | PA9 TX / PA10 RX，921600 baud |
| 步进 STEP | PE9 |
| 步进 DIR | PA0 |
| 步进默认细分 | 3200 steps/rev |
| 电机反馈周期 | TIM3，约 1 ms |

FDCAN2 使用独立 Message RAM 偏移 `512`，两路 CAN 需要分别连接收发器并共地。电机电源由 PC13/PC14 控制，具体有效电平请按实际 DM-MC02 接线确认。

## 串口命令

每条命令以回车或换行结束，串口参数为 **921600, 8-N-1**。

| 命令 | 示例 | 说明 |
| --- | --- | --- |
| 设置每圈步数 | `SET_STEPS 3200` | 范围 200~51200 |
| 步进运动 | `MOVE 2.5 400 1` | 运动 2.5 圈，脉冲间隔 400 us，方向 1；方向 0 为反向 |
| 停止步进 | `Stepper_STOP` | 立即停止脉冲输出 |
| 全部关节回零 | `1` | 将全部逻辑电机目标角度设为 0° |
| 快捷动作 | `2` | 左右两侧 1 号电机设为 -30° |
| 单电机角度 | `4 90.5` | 电机编号 1~17，角度单位为度 |
| 多电机角度 | `1 10#2 -5.5#10 30` | 一条命令设置多个电机 |

串口命令只负责下发目标值；实际角度范围还会受到 `User/dm_motor_ctrl.c` 中每个电机的软件限位约束。首次上电会依次失能、写入位置模式、保存参数并重新使能电机，请确认机械结构处于安全状态。

## 目录结构

```text
Core/
  Inc/                  STM32CubeMX 生成的头文件
  Src/                  启动流程、时钟、GPIO、FDCAN、定时器和串口初始化
Drivers/
  CMSIS/                ARM CMSIS 与 STM32H7 器件头文件
  STM32H7xx_HAL_Driver/ STM32 HAL 驱动
User/
  bsp_fdcan.*           FDCAN 过滤器、收发和回调封装
  dm_motor_drv.*        达妙 CAN 帧打包、解包和模式命令
  dm_motor_ctrl.*       多电机配置、角度限位和反馈管理
  bsp_stepper.*         STEP/DIR 步进脉冲调度
  delay.*               延时辅助函数
CtrBoard.ioc            STM32CubeMX 配置
MDK-ARM/CtrBoard.uvprojx
                         Keil MDK-ARM 工程
```

`MDK-ARM/CtrBoard/` 下的 `.o`、`.d`、`.axf`、`.hex`、`.map` 等文件是本地构建产物，不是源码。它们已通过 `.gitignore` 排除；需要发布固件时建议在 GitHub Release 中单独上传经过验证的 `.hex` 文件。

## 编译与下载

1. 安装 Keil MDK-ARM 5.x，并准备 STM32H7 Device Family Pack。
2. 打开 `MDK-ARM/CtrBoard.uvprojx`。
3. 检查芯片、下载器和 FDCAN 收发器接线，执行 **Rebuild**。
4. 下载到 DM-MC02，连接串口工具发送上述命令进行单关节和步进机构测试。

也可以用 STM32CubeMX 打开 `CtrBoard.ioc` 查看或重新生成外设初始化代码。重新生成前请备份 `Core/` 和 `User/` 中的用户代码，并确认 CubeMX 的用户代码保护选项已开启。

## 安全提示

这是面向实际机械机构的实验性控制程序。首次调试应卸载负载、降低位置模式速度、逐个验证电机 ID、方向和软限位，并在 CAN 总线增加合适的终端电阻。软件限位不能替代机械限位、急停和电源保护。

## 许可

当前仓库未附带正式开源许可证。若计划公开复用，请根据个人或团队意愿补充 `LICENSE` 文件，并确认 STM32 HAL、CMSIS 及达妙协议资料的授权范围。
