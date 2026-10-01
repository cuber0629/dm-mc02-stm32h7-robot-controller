/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
// 建议在 gpio.h 或此处定义宏，方便后面 Stepper 驱动调用
#define DIR_PORT GPIOA
#define DIR_PIN  GPIO_PIN_0
#define PUL_PORT GPIOE
#define PUL_PIN  GPIO_PIN_9
/* USER CODE END 0 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE(); // 如果你不使用 PD02，可以不开启D口时钟

  /* 1. 初始化默认输出电平 */
  
  // --- 关键修改：先拉高 PC15，开启接口的 5V 电源 ---
  // 同时保持原有的 PC13/14 配置（假设它们是CAN的静默脚或LED）
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); 
  // 注意：PC15 设置为 SET (高电平) 以开启 5V 电源
  // PC13/14 根据你之前的代码逻辑，这里先给个初值，具体看原理图是高有效还是低有效
  
  // PA00 (DIR) 和 PA02 (PUL) 默认拉低
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
	
  /* 2. 配置 PA00 为方向引脚 (DIR) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 3. 配置 PA02 为脉冲引脚 (PUL) */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 脉冲脚速度要高
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* 4. 配置 PC15 (5V电源开关) 以及 PC13/PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_15 | GPIO_PIN_14 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 5. 原有的 PA15 输入配置 (按键) */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}