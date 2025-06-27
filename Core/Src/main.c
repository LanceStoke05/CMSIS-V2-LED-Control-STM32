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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
typedef struct {
	uint8_t size;
	uint8_t payload[5];
	uint8_t checksum;
} UART_Message_t;
*/

typedef struct {
	uint8_t size;
	uint8_t instruction;
	uint8_t payload[4];
	uint8_t checksum;
} UART_Message_t;


typedef enum{
	MODE_0,       // Mode = 0
	MODE_1,       // Mode = 1
	MODE_2,       // Mode = 2
	QUERY_COUNT,  // Query for count
	QUERY_MODE,   // Query for mode
	COUNT_X       // Count = X
} UART_Instruction_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_MAX_DELAY 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADC_Task */
osThreadId_t ADC_TaskHandle;
const osThreadAttr_t ADC_Task_attributes = {
  .name = "ADC_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_5_Button */
osThreadId_t Task_5_ButtonHandle;
const osThreadAttr_t Task_5_Button_attributes = {
  .name = "Task_5_Button",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_Queue */
osMessageQueueId_t UART_QueueHandle;
const osMessageQueueAttr_t UART_Queue_attributes = {
  .name = "UART_Queue"
};
/* Definitions for countMutex */
osMutexId_t countMutexHandle;
const osMutexAttr_t countMutex_attributes = {
  .name = "countMutex"
};
/* Definitions for modeMutex */
osMutexId_t modeMutexHandle;
const osMutexAttr_t modeMutex_attributes = {
  .name = "modeMutex"
};
/* USER CODE BEGIN PV */

osThreadId_t ledTaskHandle;
osThreadId_t uartTaskHandle;
osThreadId_t flashTaskHandle;


const FLASH_EraseInitTypeDef flash_conf = {
		.TypeErase = FLASH_TYPEERASE_SECTORS,
		.Sector = FLASH_SECTOR_11,
		.NbSectors = 1,
		.VoltageRange = FLASH_VOLTAGE_RANGE_3
};



const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t flashTask_attributes = {
  .name = "flashTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


const int min_toggle_time = 100;
const int max_toggle_time = 500;
volatile int toggle_interval = 100;
volatile uint8_t button_mode_flag = 0;
volatile uint8_t button_count_flag = 0;
volatile uint32_t button_mode_tick = 0;
volatile uint32_t button_count_tick = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void Task_4_ADC(void *argument);
void Task_5_Button_func(void *argument);

/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void Task_1_UART(void * arguments);
void Task_2_Flash(void * arguments);
void Task_3_LED(void * arguments);
uint8_t count_nonzero_from_end(const uint8_t *data);
void Button_ISR_func_c(void);
void Button_ISR_func_m(void);
void UART_Send(char *data, uint8_t size);
uint8_t calculate_checksum(const uint8_t *data, size_t length, uint8_t instruction);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t rx_byte;
uint8_t tx_bytes[4] = {0};
//uint8_t my_buff[4] = {0,1,2,3};
uint8_t tx_byte = 0;
volatile uint32_t count;
volatile int mode;
const int flash_addr_start = 0x080E0000;
uint32_t adc1_data;
volatile uint8_t debug_count = 1;
volatile uint8_t debug_c, debug_q, debug_checksum;
int debug_queue_output[500] = {0};
int debug_queue_counter = 0;
int debug_queue_counter2 = 0;
int err_count = 0;
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //First boot protocol

  //Take count value from flash memory
  volatile int *mode_from_flash = &mode;
  *mode_from_flash = *(uint32_t *)(flash_addr_start + sizeof(uint32_t));

  //Take mode value from flash memory
  volatile int *counter_from_flash = &count;
  *counter_from_flash = *(uint32_t *)flash_addr_start;

  if (count < 0 || count > 0xFFFFFFFF){count = 100;}
  if (mode != 0 && mode != 1 && mode != 2){mode = 0;}

  //Check for zero conditions
  if (count == 0xFFFFFFFF){count = 100;}
  if (mode == 0xFFFFFFFF){mode = 0;}


  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of countMutex */
  countMutexHandle = osMutexNew(&countMutex_attributes);

  /* creation of modeMutex */
  modeMutexHandle = osMutexNew(&modeMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UART_Queue */
  UART_QueueHandle = osMessageQueueNew (100, sizeof(uint8_t), &UART_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ADC_Task */
  ADC_TaskHandle = osThreadNew(Task_4_ADC, NULL, &ADC_Task_attributes);

  /* creation of Task_5_Button */
  Task_5_ButtonHandle = osThreadNew(Task_5_Button_func, NULL, &Task_5_Button_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  ledTaskHandle = osThreadNew(Task_3_LED, NULL, &ledTask_attributes);
  flashTaskHandle = osThreadNew(Task_2_Flash, &count, &flashTask_attributes);
  uartTaskHandle = osThreadNew(Task_1_UART, &count, &uartTask_attributes);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_mode_Pin Button_count_Pin */
  GPIO_InitStruct.Pin = Button_mode_Pin|Button_count_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
void Task_1_UART(void * arguments)
{
	uint8_t check_start_byte = 0;
	uint8_t checksum = 0;
	uint8_t P_payload[4] = {0};
	uint32_t P_count = 0;
	for(;;)
	{
		uint32_t count_placeholder = 0;
		UART_Message_t msg_struct = {0};
		if(HAL_UART_GetState(&huart3) != HAL_UART_STATE_BUSY_TX)
		{
			if(osMessageQueueGet(UART_QueueHandle, &check_start_byte, NULL, 100)==osOK)
			{
				if (check_start_byte == 0xAA)
				{
					if(osMessageQueueGet(UART_QueueHandle, &msg_struct.size, NULL, 10)==osOK)
					{
						for (uint8_t bg = 0; bg < msg_struct.size; bg++)
						{
							osMessageQueueGet(UART_QueueHandle, &msg_struct.payload[bg], NULL, 10);
						}
						osMessageQueueGet(UART_QueueHandle, &msg_struct.checksum, NULL, 10);
						if (msg_struct.checksum == calculate_checksum(&msg_struct.payload, (size_t)msg_struct.size))
						{
							switch (msg_struct.payload[0])
							{
							case MODE_0:

								if(osMutexAcquire(modeMutexHandle, 20)==osOK)
								{
									mode = 0;
									osMutexRelease(modeMutexHandle);
								}
								break;

							case MODE_1:
								if(osMutexAcquire(modeMutexHandle, 20)==osOK)
								{
									mode = 1;
									osMutexRelease(modeMutexHandle);
								}
								break;

							case MODE_2:
								if(osMutexAcquire(modeMutexHandle, 20)==osOK)
								{
									mode = 2;
									osMutexRelease(modeMutexHandle);
								}
								break;

							case QUERY_COUNT:
								if(osMutexAcquire(countMutexHandle, 100)==osOK)
								{
									uint8_t count_bytes[4];
									count_bytes[0] = (count >> 24) & 0xFF;
									count_bytes[1] = (count >> 16) & 0xFF;
									count_bytes[2] = (count >> 8) & 0xFF;
									count_bytes[3] = count & 0xFF;
									osDelay(30);
									HAL_UART_Transmit_IT(&huart3, count_bytes, 4);
									osMutexRelease(countMutexHandle);
								}
								break;

							case QUERY_MODE:
								if(osMutexAcquire(modeMutexHandle, 100)==osOK)
								{
									uint8_t mode_placeholder = (uint8_t) mode;
									osDelay(30);
									HAL_UART_Transmit_IT(&huart3, &mode_placeholder, 1);
									osMutexRelease(modeMutexHandle);
								}
								break;

							case COUNT_X:
								P_count = 0;
								for (uint8_t k = 1; k < 5; k++)
								{
									P_count += ((msg_struct.payload[k])*pow(16, 2*(4-k)));
								}
								if(osMutexAcquire(countMutexHandle, 30)==osOK)
								{
									count = P_count;
									osMutexRelease(countMutexHandle);
								}
								break;
							default:
								break;
							}
						}
					}
				}
			}
		}
		osDelay(2);
	}
}
*/

void Task_1_UART(void * arguments)
{
	bool F_start = 0;
	bool F_size = 0;
	bool F_instruction = 0;
	bool F_payload = 0;
	bool F_checksum = 0;
	uint8_t Q_get = 0;
	UART_Message_t msg = {0};
	uint8_t counting_var = 0;

	for(;;)
	{	Q_get = 0;
		if(HAL_UART_GetState(&huart3) != HAL_UART_STATE_BUSY_TX){
			if(osMessageQueueGet(UART_QueueHandle, &Q_get, NULL, 30)==osOK) //Get byte from queue and put into Q_get
			{	//Checking start byte = 0xAA, F_start set if so
				if(F_start)
				{	//Checking accurate size byte = 1 or 5, F_size set if so
					if(F_size)
					{
						if(F_instruction)
						{
							if(msg.size == 1)
							{
								F_payload = true;
							}
							counting_var ++;
							if(counting_var == msg.size)
							{
								F_payload = true;
							}
							if(F_payload)
							{
								counting_var = 0;
								if(Q_get == calculate_checksum(&msg.payload[0], msg.size, msg.instruction))
								{
									switch (msg.instruction)
									{
									case MODE_0:

										if(osMutexAcquire(modeMutexHandle, 20)==osOK)
										{
											mode = 0;
											osMutexRelease(modeMutexHandle);
										}
										break;

									case MODE_1:
										if(osMutexAcquire(modeMutexHandle, 20)==osOK)
										{
											mode = 1;
											osMutexRelease(modeMutexHandle);
										}
										break;

									case MODE_2:
										if(osMutexAcquire(modeMutexHandle, 20)==osOK)
										{
											mode = 2;
											osMutexRelease(modeMutexHandle);
										}
										break;

									case QUERY_COUNT:
										if(osMutexAcquire(countMutexHandle, 100)==osOK)
										{
											uint8_t count_bytes[4];
											count_bytes[0] = (count >> 24) & 0xFF;
											count_bytes[1] = (count >> 16) & 0xFF;
											count_bytes[2] = (count >> 8) & 0xFF;
											count_bytes[3] = count & 0xFF;
											osDelay(30);
											HAL_UART_Transmit_IT(&huart3, count_bytes, 4);
											osMutexRelease(countMutexHandle);
										}
										break;

									case QUERY_MODE:
										if(osMutexAcquire(modeMutexHandle, 100)==osOK)
										{
											uint8_t mode_placeholder = (uint8_t) mode;
											osDelay(30);
											HAL_UART_Transmit_IT(&huart3, &mode_placeholder, 1);
											osMutexRelease(modeMutexHandle);
										}
										break;

									case COUNT_X:
										uint32_t P_count = 0;
										for (uint8_t k = 0; k < 4; k++)
										{
											P_count += ((msg.payload[k])*pow(16, 2*(3-k)));
										}
										if(osMutexAcquire(countMutexHandle, 30)==osOK)
										{
											count = P_count;
											osMutexRelease(countMutexHandle);
										}
										break;
									default:
										break;
									}
								}
								else //Checksum failed, reset initialization and send "checksum failed"
								{
									char fail[16] = "Checksum Failed";
									osDelay(30);
									HAL_UART_Transmit_IT(&huart3, &fail, strlen(fail));
								}
								F_checksum = false;
								F_instruction = false;
								F_payload = false;
								F_size = false;
								F_start = false;
								memset(&msg, 0, sizeof(msg));
							}
							else
							{
								msg.payload[counting_var - 1] = Q_get;
							}
						}
						else if(Q_get < COUNT_X || Q_get >= MODE_0)
						{
							msg.instruction = Q_get;
							F_instruction = true;
						}
						else
						{
							F_start = false;
							F_size = false;
							memset(&msg, 0, sizeof(msg));
						}
					}
					else if(Q_get == 1 || Q_get == 5)
					{
						msg.size = Q_get;
						F_size = true;
					}
					else
					{
						F_start = false;
						memset(&msg, 0, sizeof(msg));
					}
				}
				else if(Q_get == 0xAA)
				{
					F_start = true;
				}
			}
			else
			{
				F_checksum = false;
				F_instruction = false;
				F_payload = false;
				F_size = false;
				F_start = false;
				memset(&msg, 0, sizeof(msg));
			}
		}
		osDelay(2);
	}
}

void Task_2_Flash(void * arguments)
{
    static int last_mode = -1;
    static int last_count = -1;
    int current_mode, current_count;
    int sector_err = 0;

    for(;;)
    {
        if(osMutexAcquire(modeMutexHandle, osWaitForever) == osOK)
        {
            if(osMutexAcquire(countMutexHandle, osWaitForever) == osOK)
            {
                current_mode = mode;
                current_count = count;
                HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
                if(current_mode != last_mode || current_count != last_count)
                {
                    HAL_FLASH_Unlock();
                    HAL_FLASHEx_Erase(&flash_conf, &sector_err);
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr_start, (uint32_t)current_count);
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (flash_addr_start + sizeof(uint32_t)), (uint32_t)current_mode);
                    HAL_FLASH_Lock();

                    last_mode = current_mode;
                    last_count = current_count;
                }

                osMutexRelease(countMutexHandle);
            }

            osMutexRelease(modeMutexHandle);
        }
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        osDelay(10000);
    }
}

void Task_3_LED(void * arguments)
{
	uint32_t count1 = 100;

	for(;;)
	{
			if (mode == 0)
			{
				HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
			}
			else if (mode == 1) {
				HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
			}
			else
			{	{
					count1 = count;
					if (count1<100){count1 = 100;}
					if (count1>5000){count1 = 5000;}
					osDelay(count1);
					HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
				}
			}
		}
	}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {

        if (osMessageQueueGetSpace(UART_QueueHandle) > 0) {
        	osMessageQueuePut(UART_QueueHandle, &rx_byte, 0U, 0);
        }
        HAL_UART_Receive_IT(huart, &rx_byte, 1);

    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	osDelay(5);
}

uint8_t count_nonzero_from_end(const uint8_t *data) {
    uint8_t count = 0;
    for (int i = 3; i >= 0; i--) {
        if (data[i] != 0) {
            count++;
        } else {
            break;
        }
    }
    return count;
}

void Button_ISR_func_c(void)
{
	osMutexAcquire(countMutexHandle, osWaitForever);
	if (count <= 0xFFFFFFFF)
		  {
			  count += 300;
		  }
		  else
		  {
			count = 100;
		  }
	osMutexRelease(countMutexHandle);
}

void Button_ISR_func_m(void)
{
	osMutexAcquire(modeMutexHandle, osWaitForever);
	mode++;
	if (mode == 3){mode = 0;};
	if (mode < 0 || mode > 3){mode = 0;};
	osMutexRelease(modeMutexHandle);
}

uint8_t calculate_checksum(const uint8_t *data, size_t length, uint8_t instruction) {
    uint8_t checksum = 0;
    for(size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    checksum ^= (uint8_t)length ^ instruction;
    return checksum;
}

void UART_Send(char *data, uint8_t size)
{
	HAL_UART_Transmit_IT(&huart3, data, size);
}

uint32_t map(uint32_t adc_unconverted)
{
	return (uint8_t)(((adc_unconverted)*(5))/(4089));
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
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task_4_ADC */
/**
* @brief Function implementing the ADC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_4_ADC */
void Task_4_ADC(void *argument)
{
  /* USER CODE BEGIN Task_4_ADC */
  uint32_t adc1_data = 0;
  uint8_t adc1_data_converted = 2;
  uint8_t flag_adc = 5;
  /* Infinite loop */
  for(;;)
  {
    // Start ADC conversion
    HAL_ADC_Start(&hadc1);

    // Wait for conversion to complete
    if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {	if(flag_adc >0){flag_adc --;}
        adc1_data = HAL_ADC_GetValue(&hadc1);
        adc1_data_converted = map(adc1_data);
        if(flag_adc==0){
			if(osMutexAcquire(countMutexHandle, 10)==osOK)
			{
				if(adc1_data_converted<2 && count >= 100){count -= count/10; if(count < 100){count = 100;}}
				else if(adc1_data_converted>2 && count <= 5000){count += count/10; if(count > 5000){count = 5000;}}

				osMutexRelease(countMutexHandle);
			}
        }
    }

    // Stop ADC to ensure clean restart next time
    HAL_ADC_Stop(&hadc1);

    // Add a reasonable delay
    osDelay(200);
  }
  /* USER CODE END Task_4_ADC */
}

/* USER CODE BEGIN Header_Task_5_Button_func */
/**
* @brief Function implementing the Task_5_Button thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_5_Button_func */
void Task_5_Button_func(void *argument)
{
  /* USER CODE BEGIN Task_5_Button_func */
  /* Infinite loop */
  for(;;)
  {
	  if (button_count_flag)
	  			{	// __disable_irq();
	  				button_count_flag = 0;
	  				if(HAL_GPIO_ReadPin(Button_count_GPIO_Port, Button_count_Pin) == GPIO_PIN_SET)
	  				{	while(HAL_GPIO_ReadPin(Button_count_GPIO_Port, Button_count_Pin) == GPIO_PIN_SET){}
	  					if((HAL_GetTick() - button_count_tick) > DEBOUNCE_MAX_DELAY)
	  					{
	  						Button_ISR_func_c();

	  					}
	  				}
	  				//__enable_irq();
	  			}
	  			if (button_mode_flag)
	  			{	 //__disable_irq();
	  				button_mode_flag = 0;
	  				if(HAL_GPIO_ReadPin(Button_mode_GPIO_Port, Button_mode_Pin) == GPIO_PIN_SET)
	  				{	while(HAL_GPIO_ReadPin(Button_mode_GPIO_Port, Button_mode_Pin) == GPIO_PIN_SET){}
	  					if((HAL_GetTick() - button_mode_tick) > DEBOUNCE_MAX_DELAY)
	  					{
	  						Button_ISR_func_m();
	  					}
	  				}
	  				//__enable_irq();
	  			}

    osDelay(1);
  }
  /* USER CODE END Task_5_Button_func */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
