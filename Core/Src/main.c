/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 整合串口解析与电机控制 - 包含步进电机高级指令
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_fdcan.h"
#include "dm_motor_ctrl.h"
#include <stdio.h>    // 用于 sscanf 和 sprintf
#include <string.h>   // 用于 strlen, strcmp, strncmp
#include <stdlib.h>   // 用于 atoi, atof
#include "bsp_stepper.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t aRxBuffer;            // 接收中断缓冲（每次收1字节）
char rx_buf[256];             // 存放完整指令的数组
uint8_t rx_cnt = 0;           // 计数器

/* 声明步进电机结构体，以便修改 steps_per_rev */
extern StepperMotor_t stepmotor; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
/**
 * @brief 指令解析函数
 * --- 新增指令 ---
 * 1. SET_STEPS <数值>     : 修改每圈步数 (200-51200)
 * 2. MOVE <圈数> <延时> <方向> : 步进电机运动控制
 * 3. Stepper_STOP         : 立即停止步进电机
 * 
 * --- 原有指令 ---
 * 4. "1"                  : 所有 DM 电机归零
 * 5. "2"                  : 1号 DM 电机运动
 * 6. "ID 角度"            : 单个 DM 电机控制
 * 7. "ID1 A1#ID2 A2..."   : 多个 DM 电机控制
 */
void Command_Parse(char *buf) {
    
    // ================== 1. 步进电机参数设置 (SET_STEPS) ==================
    if (strncmp(buf, "SET_STEPS", 9) == 0) {
        uint32_t steps = 0;
        if (sscanf(buf, "SET_STEPS %lu", &steps) == 1) {
            if (steps >= 200 && steps <= 51200) {
                stepmotor.steps_per_rev = steps;
                char msg[64];
                sprintf(msg, "ACK: Steps/rev set to %lu\r\n", steps);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
            } else {
                char msg[] = "ERROR: Steps must be 200~51200\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
            }
        }
        return;
    }

    // ================== 2. 步进电机运动 (MOVE) ==================
    // 格式: MOVE <圈数> <延时us> <方向>
    else if (strncmp(buf, "MOVE", 4) == 0) {
        float revs = 0.0f;
        uint32_t delay = 0;
        int dir = 0;

        // 解析参数
        if (sscanf(buf, "MOVE %f %lu %d", &revs, &delay, &dir) == 3) {
            Stepper_MoveRevs(revs, delay, (uint8_t)dir);
            char msg[100];
            sprintf(msg, "ACK: Moving %.2f revs, Delay %lu us, Dir %d\r\n", revs, delay, dir);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
        } else {
            char msg[] = "ERROR: MOVE requires 3 arguments (Revs Delay Dir)\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
        }
        return;
    }

    // ================== 3. 步进电机停止 (Stepper_STOP) ==================
    else if (strcmp(buf, "Stepper_STOP") == 0) {
        stepmotor.is_running = 0; // 直接清除运行标志
        // 也可以选择拉低脉冲引脚
        HAL_GPIO_WritePin(PUL_PORT, PUL_PIN, GPIO_PIN_RESET); 
        
        char msg[] = "ACK: Stepper Stopped\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
        return;
    }

    // ================== 4. 原有 DM 电机指令逻辑 ==================
    
    // 处理快捷指令 "1"
    if (strcmp(buf, "1") == 0) {
        for(int i = 0; i < num; i++) {
            dm_motor_set_pos_by_deg(i, 0.0f);
        }
        char msg[] = "ACK: All motors move to 0 deg\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
        return;
    }

    // 处理快捷指令 "2"
    if (strcmp(buf, "2") == 0) {
        dm_motor_set_pos_by_deg(LMotor1, -30.0f);
        dm_motor_set_pos_by_deg(RMotor1, -30.0f);
        char msg[] = "ACK: Motor 1 move to 30 deg\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
        return;
    }

    // 处理多电机指令 "ID1 角度1#ID2 角度2#..."
    if (strchr(buf, '#') != NULL) {
        char temp_buf[256];
        strcpy(temp_buf, buf);
        char *token = strtok(temp_buf, "#");
        int processed_count = 0;
        int error_count = 0;
        char error_msg[128] = {0};

        while (token != NULL) {
            int id = 0;
            float angle = 0.0f;
            
            if (sscanf(token, "%d %f", &id, &angle) == 2) {
                if (id >= 1 && id <= num) {
                    dm_motor_set_pos_by_deg(id - 1, angle);
                    processed_count++;
                } else {
                    error_count++;
                    if (strlen(error_msg) < 100) {
                        char error_part[64];
                        sprintf(error_part, "Motor %d out of range; ", id);
                        strcat(error_msg, error_part);
                    }
                }
            } else {
                error_count++;
                if (strlen(error_msg) < 100) {
                    char error_part[64];
                    sprintf(error_part, "Fmt err: %s; ", token);
                    strcat(error_msg, error_part);
                }
            }
            token = strtok(NULL, "#");
        }

        char feedback[128];
        if (error_count == 0) {
            sprintf(feedback, "ACK: %d motors set OK\r\n", processed_count);
        } else {
            sprintf(feedback, "ACK: %d OK, %d ERR: %s\r\n", processed_count, error_count, error_msg);
        }
        HAL_UART_Transmit(&huart1, (uint8_t*)feedback, strlen(feedback), 10);
        return;
    }

    // 处理常规单电机指令 "电机号 角度" (例如 "4 90.5")
    int id = 0;
    float angle = 0.0f;
    if (sscanf(buf, "%d %f", &id, &angle) == 2) {
        if (id >= 1 && id <= num) {
            dm_motor_set_pos_by_deg(id - 1, angle);
            char feedback[64];
            sprintf(feedback, "ACK: Motor %d set to %.2f deg\r\n", id, angle);
            HAL_UART_Transmit(&huart1, (uint8_t*)feedback, strlen(feedback), 10);
        } else {
            char error_feedback[64];
            sprintf(error_feedback, "Error: Motor %d out of range\r\n", id);
            HAL_UART_Transmit(&huart1, (uint8_t*)error_feedback, strlen(error_feedback), 10);
        }
    }
}

/**
 * @brief 串口接收回调函数
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (aRxBuffer == '\n' || aRxBuffer == '\r') {
            if (rx_cnt > 0) {
                rx_buf[rx_cnt] = '\0'; 
                Command_Parse(rx_buf);  
                rx_cnt = 0;             
            }
        } else {
            if (rx_cnt < 256) {
                rx_buf[rx_cnt++] = aRxBuffer; 
            }
        }
        HAL_UART_Receive_IT(&huart1, &aRxBuffer, 1); 
    }
}

/**
 * @brief 定时器回调函数：轮询读取电机反馈
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        read_all_motor_data(&motor[LMotor1]);
    }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  
  /* USER CODE BEGIN Init */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  DBGMCU->CR &= ~0x00000020;  
  DBGMCU->CR |= 0x00000001;
  /* USER CODE END Init */

  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  // --- 强制开启硬件使能 ---
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); 
	
  LeftPower(1); 
  RightPower(1); 
  HAL_Delay(1000);
  
  dm_motor_init();
  bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
  bsp_fdcan_set_baud(&hfdcan2, CAN_CLASS, CAN_BR_1M);
  bsp_can_init(); 
  HAL_Delay(100);
    
  // 测试串口是否通畅
  HAL_UART_Transmit(&huart1, (uint8_t*)"System Start OK!\r\n", 18, 100);
    
  // 初始化右边电机
  for(int i=0; i<9; i++) {
      dm_motor_disable(&hfdcan1, &motor[i]); HAL_Delay(20);
      write_motor_data(motor[i].id, 10, pos_mode, 0, 0, 0); HAL_Delay(20);
      save_motor_data(motor[i].id, 10); HAL_Delay(20);
      dm_motor_enable(&hfdcan1, &motor[i]); HAL_Delay(20);
  }

  // 初始化左边电机
  for(int i=0; i<9; i++) {
      dm_motor_disable(&hfdcan2, &motor[i]); HAL_Delay(20);
      write_motor_data(motor[i].id, 10, pos_mode, 0, 0, 0); HAL_Delay(20);
      save_motor_data(motor[i].id, 10); HAL_Delay(20);
      dm_motor_enable(&hfdcan2, &motor[i]); HAL_Delay(20);
  }
    
  // 开启串口中断
  HAL_UART_Receive_IT(&huart1, &aRxBuffer, 1);
  
  // 启动定时器轮询
  HAL_TIM_Base_Start_IT(&htim3);
    
  Stepper_Init(3200); // 默认步进设置
  /* USER CODE END 2 */
            
  while (1)
  {
    // 步进电机脉冲生成核心函数
    Stepper_Update();

    /* --- 按键逻辑 --- */
    static uint8_t press_count = 0;      
    static uint8_t last_button_state = 1; 
    uint8_t current_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

    if (last_button_state == 1 && current_state == 0) 
    {
        HAL_Delay(20); // 消抖
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 0) 
        {
            press_count++; 
            if (press_count == 1) 
            {
                // 按一下：所有电机归零位
                for(int i=0; i<8; i++) dm_motor_set_pos_by_deg(i, 0.0f);
            } 
            else if (press_count == 2) 
            {
                // 按两下：1号电机到30度
                dm_motor_set_pos_by_deg(LMotor1, -30.0f);
                dm_motor_set_pos_by_deg(RMotor1, -30.0f);
                press_count = 0; 
            }
        }
    }
    last_button_state = current_state;
  }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
