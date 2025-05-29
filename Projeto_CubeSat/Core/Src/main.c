/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "bmp280.h"
#include <string.h>
#include <stdarg.h>
#include "LoggerService.h"
#include "Barometer.h"
#include "Accelerometer.h"
#include "Gyroscope.h"
#include "UartService.h"
#include "SDCard.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BaroTask */
osThreadId_t BaroTaskHandle;
const osThreadAttr_t BaroTask_attributes = {
  .name = "BaroTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for LoggerTask */
osThreadId_t LoggerTaskHandle;
const osThreadAttr_t LoggerTask_attributes = {
  .name = "LoggerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for BuzzerTask */
osThreadId_t BuzzerTaskHandle;
const osThreadAttr_t BuzzerTask_attributes = {
  .name = "BuzzerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CommandTask */
osThreadId_t CommandTaskHandle;
const osThreadAttr_t CommandTask_attributes = {
  .name = "CommandTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal6,
};
/* Definitions for CriticalCommand */
osThreadId_t CriticalCommandHandle;
const osThreadAttr_t CriticalCommand_attributes = {
  .name = "CriticalCommand",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for GyroTask */
osThreadId_t GyroTaskHandle;
const osThreadAttr_t GyroTask_attributes = {
  .name = "GyroTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for AccTask */
osThreadId_t AccTaskHandle;
const osThreadAttr_t AccTask_attributes = {
  .name = "AccTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp280;
float pressure, temperature, humidity;
uint16_t size;
volatile bool buzzer_enabled = true;
osMessageQueueId_t loggerQueueHandle;
osMessageQueueId_t commQueueHandle;
osMessageQueueId_t uartQueueHandle;
osMessageQueueId_t commandQueueHandle;
osMessageQueueId_t criticalCommandQueueHandle;
osMutexId_t sdMutexHandle;
uint8_t rx_byte;
uint8_t sat_location = 1; // 1 = terra, 2 = em lançamento, 3 = em órbita
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void StartBaroTask(void *argument);
void StartLoggerTask(void *argument);
void StartCommTask(void *argument);
void StartBuzzerTask(void *argument);
void StartCommandTask(void *argument);
void StartCriticalCommandTask(void *argument);
void StartGyroTask(void *argument);
void StartAccTask(void *argument);

/* USER CODE BEGIN PFP */
bool ReadBarometer();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len){
	(void)file;
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
		HAL_UART_Transmit(&huart3, (uint8_t*) &ptr[DataIdx], 1, 100);

	return len;
}
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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Barometer_Init(&hi2c2);
  HAL_TIM_Base_Start_IT(&htim3);
  if (!SDCard_Init()) {
      printf("Failed to initialize SD card\r\n");
  } else {
      printf("SD card initialized\r\n");
  }
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  sdMutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  loggerQueueHandle  		 = osMessageQueueNew(10, sizeof(BarometerData), NULL);
  commQueueHandle    		 = osMessageQueueNew(10, sizeof(CommMessage), NULL);
  uartQueueHandle 	 		 = osMessageQueueNew(10, sizeof(UartMessage), NULL);
  commandQueueHandle 		 = osMessageQueueNew(10, sizeof(char), NULL);
  criticalCommandQueueHandle = osMessageQueueNew(10, sizeof(char), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of BaroTask */
  BaroTaskHandle = osThreadNew(StartBaroTask, NULL, &BaroTask_attributes);

  /* creation of LoggerTask */
  LoggerTaskHandle = osThreadNew(StartLoggerTask, NULL, &LoggerTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(StartCommTask, NULL, &CommTask_attributes);

  /* creation of BuzzerTask */
  BuzzerTaskHandle = osThreadNew(StartBuzzerTask, NULL, &BuzzerTask_attributes);

  /* creation of CommandTask */
  CommandTaskHandle = osThreadNew(StartCommandTask, NULL, &CommandTask_attributes);

  /* creation of CriticalCommand */
  CriticalCommandHandle = osThreadNew(StartCriticalCommandTask, NULL, &CriticalCommand_attributes);

  /* creation of GyroTask */
  GyroTaskHandle = osThreadNew(StartGyroTask, NULL, &GyroTask_attributes);

  /* creation of AccTask */
  AccTaskHandle = osThreadNew(StartAccTask, NULL, &AccTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|SD_CS_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin SD_CS_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|SD_CS_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	static GPIO_PinState led_state = GPIO_PIN_RESET;

	if (htim->Instance == TIM3) {

		led_state = (led_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;

		switch(sat_location)
		  {
			  case 1:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, led_state);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
				  break;
			  case 2:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, led_state);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, led_state);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
				  break;
			  case 3:
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, led_state);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, led_state);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, led_state);
				  break;
		  }
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin)
{
	if(GPIO_pin == USER_Btn_Pin)
	{
		if(sat_location < 3)
			sat_location++;
		else
			sat_location = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		switch(rx_byte)
		{
			case 'r': //read bpm
			case 'b': //buzzer
				osMessageQueuePut(criticalCommandQueueHandle, &rx_byte, 0, 0);
				break;
			case 'e': // erase log
				osMessageQueuePut(commandQueueHandle, &rx_byte, 0, 0);
				break;
		}

		HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
	}
}

bool ReadBarometer()
{
	  CommMessage msg;
	  BarometerData data;
	  if (!Barometer_Read(&data)) {
		  return false;
	  }

	  msg.type = MSG_TYPE_BAROMETER;
	  msg.payload.baro = data;
	  osMessageQueuePut(commQueueHandle, &msg, 0, 0);
	  osMessageQueuePut(loggerQueueHandle, &data, 0, 0);
	  return true;
}

bool ReadAccelerometer() //m/s^2
{
	CommMessage msg;
	msg.type = MSG_TYPE_ACCEL;

	switch(sat_location)
	{
		case 1:  // Em terra
			msg.payload.accel.x = 0.01f; // ruído
			msg.payload.accel.y = 0.02f; // ruído
			msg.payload.accel.z = 9.80f; // gravidade da terra
			break;

		case 2:  // Lançamento
			msg.payload.accel.x = 0.10f;  // vibração do foguete
			msg.payload.accel.y = 0.05f;  // vibração do foguete
			msg.payload.accel.z = 30.00f; // aceleração para escapar da gravidade
			break;

		case 3:  // Órbita
			msg.payload.accel.x = 0.01f; // ruído
			msg.payload.accel.y = 0.01f; // ruído
			msg.payload.accel.z = 0.00f; // microgravidade
			break;
	}

	osMessageQueuePut(commQueueHandle, &msg, 0, 0);
	return true;
}

bool ReadGyroscope() // º/s
{
	CommMessage msg;
	msg.type = MSG_TYPE_GYRO;

	switch(sat_location)
	{
		case 1:  // Em terra
			msg.payload.gyro.x = 0.00f; // estático
			msg.payload.gyro.y = 0.00f; // estático
			msg.payload.gyro.z = 0.01f; // ruído
			break;

		case 2:  // Lançamento
			msg.payload.gyro.x = 0.10f; // vibrações do foguete
			msg.payload.gyro.y = 0.05f; // vibrações do foguete
			msg.payload.gyro.z = 0.20f; // giro leve durante o lançamento
			break;

		case 3:  // Órbita
			msg.payload.gyro.x = 0.50f; // rotação lenta natural
			msg.payload.gyro.y = 0.30f; // rotação lenta natural
			msg.payload.gyro.z = 0.20f; // rotação lenta natural
			break;
	}

	osMessageQueuePut(commQueueHandle, &msg, 0, 0);
	return true;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBaroTask */
/**
* @brief Function implementing the BaroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBaroTask */
void StartBaroTask(void *argument)
{
  /* USER CODE BEGIN StartBaroTask */
  /* Infinite loop */
  for(;;)
  {
	  if(!ReadBarometer())
	  {
		  printf("Failed to read from barometer.\r\n");
		  osDelay(1000);
		  continue;
	  }

	  osDelay(5000);
  }
  /* USER CODE END StartBaroTask */
}

/* USER CODE BEGIN Header_StartLoggerTask */
/**
* @brief Function implementing the LoggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLoggerTask */
void StartLoggerTask(void *argument)
{
  /* USER CODE BEGIN StartLoggerTask */
  /* Infinite loop */
    BarometerData data;
	for(;;)
	{
		if(osMessageQueueGet(loggerQueueHandle, &data, NULL, osWaitForever) == osOK)
		{
			if (osMutexAcquire(sdMutexHandle, osWaitForever) == osOK)
			{
				LoggerService_LogBMPData(data);
				osMutexRelease(sdMutexHandle);
			}
			else
			{
				printf("Erro: mutex inválido ou corrompido!\n");
			}

		}
		//UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
		//printf("LoggerTask stack remaining: %lu bytes\r\n", highWaterMark * sizeof(StackType_t));
		osDelay(1);
	}
  /* USER CODE END StartLoggerTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  /* Infinite loop */
	CommMessage msg;
	for(;;)
	{
		if (osMessageQueueGet(commQueueHandle, &msg, NULL, osWaitForever) == osOK)
		{
			switch(msg.type)
			{
				case MSG_TYPE_BAROMETER:
					snprintf(msg.message, sizeof(msg.message),
						"Pressure=%.2f Pa, Temperature=%.2f °C, Altitude=%.2f m\r\n",
						msg.payload.baro.pressure,
						msg.payload.baro.temperature,
						msg.payload.baro.altitude);
					break;

				case MSG_TYPE_ACCEL:
					snprintf(msg.message, sizeof(msg.message),
						"ACC: X=%.2f Y=%.2f Z=%.2f\r\n",
						msg.payload.accel.x,
						msg.payload.accel.y,
						msg.payload.accel.z);
					break;

				case MSG_TYPE_GYRO:
					snprintf(msg.message, sizeof(msg.message),
						"GYRO: X=%.2f Y=%.2f Z=%.2f\r\n",
						msg.payload.gyro.x,
						msg.payload.gyro.y,
						msg.payload.gyro.z);
					break;

				default:
					snprintf(msg.message, sizeof(msg.message), "Unknown message\r\n");
					break;
			}

			HAL_UART_Transmit(&huart3, (uint8_t*)msg.message, strlen(msg.message), 100);
		}
		//UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
		//printf("CommTask stack remaining: %lu bytes\r\n", highWaterMark * sizeof(StackType_t));
		osDelay(1);
	}

  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartBuzzerTask */
/**
* @brief Function implementing the BuzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBuzzerTask */
void StartBuzzerTask(void *argument)
{
  /* USER CODE BEGIN StartBuzzerTask */
	printf("BuzzerTask initialized\r\n");
	uint32_t beep_interval = 1000;
  /* Infinite loop */
  for(;;)
  {
	  switch(sat_location)
	  {
		  case 1:
			  beep_interval = 1000;
			  break;
		  case 2:
			  beep_interval = 500;
			  break;
		  case 3:
			  beep_interval = 2000;
			  break;
	  }

	  if(buzzer_enabled)
	  {
		  // Liga o buzzer com duty cycle de 50%
		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, htim4.Init.Period / 2);

		  osDelay(200); // Som por 200ms

		  // Desliga o som (mantém PWM ativo, mas duty = 0)
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	  }
	  else
	  {
		  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	  }

	  osDelay(beep_interval);
  }
  /* USER CODE END StartBuzzerTask */
}

/* USER CODE BEGIN Header_StartCommandTask */
/**
* @brief Function implementing the CommandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommandTask */
void StartCommandTask(void *argument)
{
  /* USER CODE BEGIN StartCommandTask */
  /* Infinite loop */
  for(;;)
  {
	  uint8_t command;
	  if(osMessageQueueGet(commandQueueHandle, &command, NULL, osWaitForever) == osOK)
	  {
		    switch(command)
			{
				case 'e': // erase log
					if (osMutexAcquire(sdMutexHandle, osWaitForever) == osOK)
					{
						LoggerService_ClearBmpLogFile();
						osMutexRelease(sdMutexHandle);
					}
					else
					{
						printf("Erro: mutex inválido ou corrompido!\n");
					}

					break;
			}
	  }

	  osDelay(1);
  }
  /* USER CODE END StartCommandTask */
}

/* USER CODE BEGIN Header_StartCriticalCommandTask */
/**
* @brief Function implementing the CriticalCommand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCriticalCommandTask */
void StartCriticalCommandTask(void *argument)
{
  /* USER CODE BEGIN StartCriticalCommandTask */
  /* Infinite loop */

  for(;;)
  {
	  uint8_t command;
	  if(osMessageQueueGet(criticalCommandQueueHandle, &command, NULL, osWaitForever) == osOK)
	  {
		  switch(command)
		  {
	  		  case 'r': //read bpm
	  			  ReadBarometer();
	  			  break;
			  case 'b': //buzzer
				  buzzer_enabled = !buzzer_enabled;
				  break;
		  }
	  }

    osDelay(1);
  }
  /* USER CODE END StartCriticalCommandTask */
}

/* USER CODE BEGIN Header_StartGyroTask */
/**
* @brief Function implementing the GyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGyroTask */
void StartGyroTask(void *argument)
{
  /* USER CODE BEGIN StartGyroTask */

	uint32_t gyro_interval = 10000;
  /* Infinite loop */
	for(;;)
	{
		switch(sat_location)
		{
		  case 1:
			  gyro_interval = 10000;
			  break;
		  case 2:
			  gyro_interval = 5000;
			  break;
		  case 3:
			  gyro_interval = 1000;
			  break;
		}

		if(!ReadGyroscope())
		{
			printf("Failed to read from gyroscope.\r\n");
			osDelay(1000);
			continue;
		}

		osDelay(gyro_interval);
	}
  /* USER CODE END StartGyroTask */
}

/* USER CODE BEGIN Header_StartAccTask */
/**
* @brief Function implementing the AccTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAccTask */
void StartAccTask(void *argument)
{
  /* USER CODE BEGIN StartAccTask */
	uint32_t acc_interval = 10000;
  /* Infinite loop */
	for(;;)
	{
		switch(sat_location)
		{
		  case 1:
			  acc_interval = 5000;
			  break;
		  case 2:
			  acc_interval = 1000;
			  break;
		  case 3:
			  acc_interval = 10000;
			  break;
		}

		if(!ReadAccelerometer())
		{
			printf("Failed to read from acelerometer.\r\n");
			osDelay(1000);
			continue;
		}

		osDelay(acc_interval);
	}
  /* USER CODE END StartAccTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  SDCard_Deinit();
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
