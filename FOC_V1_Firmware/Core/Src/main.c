/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <math.h>
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;

/* Definitions for StateTask */
osThreadId_t StateTaskHandle;
const osThreadAttr_t StateTask_attributes = {
  .name = "StateTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN PV */
uint16_t as5048a_tx_buffer = 0xFFFF;
uint16_t as5048a_rx_buffer = 0x0000;
volatile uint8_t start_motor = 0;
volatile uint16_t raw_adc_a, raw_adc_c;
volatile float offset_a = 2048.0f;
volatile float offset_c = 2048.0f;
volatile float theta = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_CAN1_Init(void);
void StartStateTask(void *argument);
void StartTelemetryTask(void *argument);

/* USER CODE BEGIN PFP */
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
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_CAN1_Init();

  /* Init scheduler */
  osKernelInitialize();

  /* Create the thread(s) */
  StateTaskHandle = osThreadNew(StartStateTask, NULL, &StateTask_attributes);
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);

  /* Start scheduler */
  osKernelStart();

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
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

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI3_Init(void)
{
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 40;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* USER CODE BEGIN 4 */
// Helper function for reading DRV8301 register
uint16_t DRV8301_ReadReg(uint8_t addr) {
	uint16_t tx_data;
	uint16_t rx_data = 0;

	tx_data = (1 << 15) | ((addr & 0x0F) << 11);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 1, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

	for(volatile int i=0; i<50; i++);

	tx_data = 0x0000;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 1, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

	return (rx_data & 0x07FF);
}

// Helper function for writing DRV8301 register
void DRV8301_WriteReg(uint8_t addr, uint16_t data) {
	uint16_t tx_data;
	uint16_t rx_data;

	tx_data = (0 << 15) | ((addr & 0xF) << 11) | (data & 0x07FF);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 1, 100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

	for(volatile int i = 0; i < 50; i++);
}

// SPI DMA Completion Callback
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI3) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	}
}

// Hardware ADC Interrupt Callback
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		// Read the raw injected ADC values (0 to 4095)
		raw_adc_a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		raw_adc_c = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartStateTask */
void StartStateTask(void *argument)
{
  MX_USB_DEVICE_Init();
  char usb_buffer[128];

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  osDelay(50);

  DRV8301_WriteReg(0x02, 0x0400);
  osDelay(10);
  DRV8301_WriteReg(0x03, 0x0004);
  osDelay(10);

  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc2);
  HAL_TIM_Base_Start(&htim1);
  osDelay(10);

  uint32_t sum_a = 0;
  uint32_t sum_c = 0;
  for (int i = 0; i < 1000; i++) {
	  sum_a += raw_adc_a;
	  sum_c += raw_adc_c;
	  osDelay(1);
  }
  offset_a = (float)sum_a / 1000.0f;
  offset_c = (float)sum_c / 1000.0f;

  while (start_motor == 0) {
	  int len = sprintf(usb_buffer, "Raw A: %u, Raw C: %u | Motor is OFF.\r\n", raw_adc_a, raw_adc_c);
	  CDC_Transmit_FS((uint8_t*)usb_buffer, len);
	  osDelay(1000);
  }

  int len = snprintf(usb_buffer, sizeof(usb_buffer),
		  	  	  	"|STARTING OPEN LOOP COMMUTATION | Offset A: %d, Offset C: %d|\r\n",
		  	  	  	(int)offset_a, (int)offset_c);
  CDC_Transmit_FS((uint8_t*)usb_buffer, len);
  osDelay(100);

  HAL_TIM_Base_Stop(&htim1);
  TIM1->CNT = 0;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_MOE_ENABLE(&htim1);

  const float PI = 3.14159265359f;
  float v_limit = 0.08f;

  // Alignment: lock the magnetic field at exactly 0 degrees
  float dutyA = 0.5f + (v_limit * sinf(0.0f));
  float dutyB = 0.5f + (v_limit * sinf(0.0f - (2.0f * PI / 3.0f)));
  float dutyC = 0.5f + (v_limit * sinf(0.0f + (2.0f * PI / 3.0f)));

  TIM1->CCR1 = (uint32_t)(dutyA * 4199.0f);
  TIM1->CCR2 = (uint32_t)(dutyB * 4199.0f);
  TIM1->CCR3 = (uint32_t)(dutyC * 4199.0f);

  // Wait 1.5 seconds for the physical rotor to snap into place
  osDelay(1500);

  // Spinning Motor 10 electrical revolutions per second 
  // Eaglepower 90kv motor has 20 pole pairs
  // (10 elec rps) / (20 pole pairs) = 0.5 rps (revolutions per second)
  uint32_t start_time = HAL_GetTick();
  float speed_rad_s = 10.0f * (2.0f * PI);
  float time_sec;

  for(;;)
  {
	uint32_t current_time = HAL_GetTick() - start_time;
	time_sec = (float)current_time / 1000.0f;

	theta = time_sec * speed_rad_s;
	theta = fmodf(theta, 2.0f * PI);

	dutyA = 0.5f + (v_limit * sinf(theta));
	dutyB = 0.5f + (v_limit * sinf(theta - (2.0f * PI / 3.0f)));
	dutyC = 0.5f + (v_limit * sinf(theta + (2.0f * PI / 3.0f)));

	TIM1->CCR1 = (uint32_t)(dutyA * 4199.0f);
	TIM1->CCR2 = (uint32_t)(dutyB * 4199.0f);
	TIM1->CCR3 = (uint32_t)(dutyC * 4199.0f);

	osDelay(2);
  }
}
/* USER CODE END Header_StartStateTask */

/* USER CODE BEGIN Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  char usb_buffer[128];

  for(;;) {
	  uint16_t raw_angle = as5048a_rx_buffer & 0x3FFF;

	  // Reading phase currents
	  float i_a = ((float)raw_adc_a - offset_a) * 0.040293f;
	  float i_c = ((float)raw_adc_c - offset_c) * 0.040293f;
	  float i_b = -(i_a + i_c);

	  // CLARKE TRANSFORMATION
	  // i_alpha = i_a
	  // i_beta = (1/sqrt(3) * i_a) + (2/sqrt(3) * i_b)
	  float i_alpha = i_a;
	  float i_beta = (0.577350269f * i_a) + (1.154700538f * i_b);

	  // PARK TRANSFORMATION
	  float c = cosf(theta);
	  float s = sinf(theta);

	  float i_d = (i_alpha * c) + (i_beta * s);
	  float i_q = -(i_alpha * s) + (i_beta * c);

	  // format CSV message for telemetry
	  int len = snprintf(usb_buffer, sizeof(usb_buffer),
			  	  	  	 "%u,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
			  	  	  	 raw_angle, i_a, i_b, i_c, i_d, i_q);

	  CDC_Transmit_FS((uint8_t*)usb_buffer, len);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*)&as5048a_tx_buffer, (uint8_t*)&as5048a_rx_buffer, 1);

	  osDelay(5);
  }
}
/* USER CODE END Header_StartTelemetryTask */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
