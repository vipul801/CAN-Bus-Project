/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_TEMP 60
#define STEP_PIN GPIO_PIN_12  // Step pin (e.g., PA0)
#define DIR_PIN  GPIO_PIN_13  // Direction pin (e.g., PA1)
#define STEP_PORT GPIOD       // GPIO Port for Step pin
#define DIR_PORT  GPIOD       // GPIO Port for Direction pin
#define MOTOR_PIN_1 GPIO_PIN_1
#define MOTOR_PIN_2 GPIO_PIN_2
#define MOTOR_PIN_3 GPIO_PIN_3
#define MOTOR_PIN_4 GPIO_PIN_4
#define MOTOR_GPIO_PORT GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];// Actual Payload
uint8_t TxData[8];
uint8_t Temperature;
uint8_t Humidity;
uint16_t Rain_water;
uint8_t temp=0;
uint8_t adcValue;
uint8_t iot_Send_Flag = 0;
uint32_t PWMdutycycle;
char ch[50];
char uartSend[9]={'F','A','B','C','D','L','\r','\n'};
// Define the step sequence for the 28BYJ-48 stepper motor
const uint8_t step_sequence[4] = {0x01, 0x02, 0x04, 0x08};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void Error_Handler(void);
void CAN1_Tx();
void CAN_filterConfig(void);
void Adjust_Fan_Speed(uint8_t temp);
void Stepper_Move(uint16_t steps, uint8_t direction);
void iot_Send_Data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_BUSOFF)!= HAL_OK)
  {
	  Error_Handler();
  }
  CAN_filterConfig();

 if (HAL_CAN_Start(&hcan1) != HAL_OK)
   	  {
    	Error_Handler();
   	  }
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  	{
  	  // Notification Error
  	  Error_Handler();
  	}
  if(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)!=HAL_OK)
  {
  	Error_Handler();
  }
  Adjust_Fan_Speed(Temperature);
  //temp=0;
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  				if(temp==1)
	  				{
	  			  	  	  // Rotate motor forward 180 degrees
	  			          Stepper_Move(70, 1); // 32 steps forward
	  			          //HAL_Delay(1000); // Wait 1 second

	  			          // Rotate motor backward 180 degrees
	  			          Stepper_Move(70, 0); // 32 steps backward
	  			          //HAL_Delay(1000); // Wait 1 second
	  				}

	  				if(iot_Send_Flag == 1)
	  				{
	  					iot_Send_Flag=0;
	  					iot_Send_Data();
	  				}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 24;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7199;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



  	void CAN_filterConfig (void)
  	{
  		CAN_FilterTypeDef sFilterConfig;
  		  sFilterConfig.FilterBank = 0;
  		  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  		  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  		  sFilterConfig.FilterIdHigh = 0x0000;
  		  sFilterConfig.FilterIdLow = 0x0000;
  		  sFilterConfig.FilterMaskIdHigh = 0x0000;
  		  sFilterConfig.FilterMaskIdLow = 0x0000;
  		  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  		  sFilterConfig.FilterActivation = ENABLE;
  		  sFilterConfig.SlaveStartFilterBank = 14;
  		  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  		  {
  		    Error_Handler();
  		  }
  	}

  	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
  	{
  	 /* Get RX message from Fifo0 as message is Pending in Fifo to be Read*/
  	 if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  	   Error_Handler();
  	 if ((RxHeader.StdId == 0x65D) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 6))
  	 {
  		 Temperature=RxData[0];
  		 Humidity = RxData[1];
  		 Rain_water=RxData[2]<<8 ;
  		Rain_water=RxData[3]+Rain_water;

  	 }
  	 Adjust_Fan_Speed(Temperature);
  	 if(Rain_water<2000)
  		 temp=1;
  	 else
  		 temp=0;
  	}


  	void Stepper_Move(uint16_t steps, uint8_t direction)
  	   {
  	       for (uint16_t i = 0; ((i < steps)&&(temp==1)); i++)
  	       {
  	           // Set the GPIO pins based on the step sequence
  	           for (uint8_t j = 0; ((j < 4)&&(temp==1)); j++)
  	           {
  	               if (direction) // Forward
  	               {
  	                   HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_1, (step_sequence[j] & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  	                   HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_2, (step_sequence[j] & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  	                   HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_3, (step_sequence[j] & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  	                   HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_4, (step_sequence[j] & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  	               }
  	               else // Backward
  	               {
  	                   uint8_t k = 3 - j; // Reverse the sequence for backward movement
  	                   HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_1, (step_sequence[k] & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  	                   HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_2, (step_sequence[k] & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  	                   HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_3, (step_sequence[k] & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  	                   HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_PIN_4, (step_sequence[k] & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  	               }

  	               HAL_Delay(5); // Delay for step pulse width
  	           }
  	       }
  	   }




  	void Adjust_Fan_Speed(uint8_t temp)
  	{
  		PWMdutycycle=99;//(uint32_t)(((Temperature)/MAX_TEMP)*100);
  		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,PWMdutycycle);
  	}

  	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  	{
  		iot_Send_Flag = 1;
  	 HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
  	}

  	void iot_Send_Data(void){
  		uartSend[1]= (char)(Temperature);
  		uartSend[2]= (char)(Humidity);
  		uartSend[3]= (char)((Rain_water));
  		sprintf(uartSend,"%d %d %d \r\n",Temperature,Humidity,Rain_water);
  		 if(HAL_UART_Transmit(&huart4, (uint8_t *)uartSend, strlen(uartSend), 50)!= HAL_OK)
  		 {
  			 Error_Handler();
  		 }
  	}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
