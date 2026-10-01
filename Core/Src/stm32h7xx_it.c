/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2; // 新增 CAN2 句柄
extern TIM_HandleTypeDef   htim3;
extern TIM_HandleTypeDef   htim4;
extern UART_HandleTypeDef  huart1;  // 新增 串口1 句柄

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void)
{
  while (1) {}
}

void HardFault_Handler(void)
{
  while (1) {}
}

void MemManage_Handler(void)
{
  while (1) {}
}

void BusFault_Handler(void)
{
  while (1) {}
}

void UsageFault_Handler(void)
{
  while (1) {}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief 串口1 中断处理函数 (必须有这个，串口指令才会生效)
  */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief FDCAN1 中断 0
  */
void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

/**
  * @brief FDCAN1 中断 1
  */
void FDCAN1_IT1_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
}

/**
  * @brief FDCAN2 中断 0 (为你以后开启 CAN2 做准备)
  */
void FDCAN2_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan2);
}

/**
  * @brief FDCAN2 中断 1
  */
void FDCAN2_IT1_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan2);
}

/**
  * @brief TIM3 全局中断
  */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

/**
  * @brief TIM4 全局中断
  */
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
}

/**
  * @brief FDCAN 校准单元中断
  */
void FDCAN_CAL_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);
  HAL_FDCAN_IRQHandler(&hfdcan2);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
