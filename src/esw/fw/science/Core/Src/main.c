/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <heater.hpp>
#include <spectral.hpp>
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "adc_sensor.h"
#include "diag_temp_sensor.h"
#include "smbus.h"
#include "toggle_gpio.h"

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

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */


osThreadId_t ReceiveMessagesHandle;
const osThreadAttr_t ReceiveMessages_attributes = {
  .name = "ReceiveMessages",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

osThreadId_t SpectralTaskHandle;
const osThreadAttr_t SpectralTask_attributes = {
  .name = "SpectralTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

osThreadId_t ThermistorAndAutoShutoffTaskHandle;
const osThreadAttr_t ThermistorAndAutoShutoffTask_attributes = {
  .name = "ThermistorAndAutoShutoffTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

osThreadId_t HeaterUpdatesTaskHandle;
const osThreadAttr_t HeaterUpdatesTask_attributes = {
  .name = "HeaterUpdatesTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);

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
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  //Create all our stuff!

  //initialize spectral sensor
  Spectral* spectral_sensor[3];
  SMBus* buses[3];
  for (int i = 0; i < 3; ++i){
	  buses[i] = new_smbus(&hi2c1);
	  spectral_sensor[i] = new_spectral(buses[i]);
  }

  set_active_spectral_sensor(&hi2c1, 0);

  //initialize thermistors NOT NEEDED BECAUSE EVERY THERMISTOR IS ASSIGNED TO A HEATER?
  /*ADCSensor* adc_sensors[3];
  DiagTempSensor* thermistors[3];
  int thermistor_channels [3] = {0, 1, 2};
  for (int i = 0; i < 3; ++i){
	  adc_sensors[i] = new_adc_sensor(&hadc, total_channels);
	  thermistors[i] = new_diag_temp_sensor(adc_sensors[i], thermistor_channels[i]);
  }*/

  //MUX is hi2c1


  //initialize leds
  Toggle_GPIO* uv_led[3];
  GPIO_TypeDef* uv_led_pins[3] = {GPIOA, GPIOA, GPIOA};
  uint16_t uv_led_pin_numbers[3] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2};
  for (int i = 0; i < 3; ++i){
	  uv_led[i] = new_toggle_gpio(uv_led_pins[i], uv_led_pin_numbers[i]);
  }


  //initialize white leds
  Toggle_GPIO* white_leds[3];
  GPIO_TypeDef* white_led_pins[3] = {GPIOA, GPIOA, GPIOC};
  uint16_t white_led_pin_numbers[3] = {GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_4};
  for (int i = 0; i < 3; ++i){
	  white_leds[i] = new_toggle_gpio(white_led_pins[i], white_led_pin_numbers[i]);
  }

  //initialize debug leds
  Toggle_GPIO* debug_leds[3];
  GPIO_TypeDef* debug_led_pins[3] = {GPIOC, GPIOC, GPIOC};
  uint16_t debug_led_pin_numbers[3] = {GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
  for (int i = 0; i < 3; ++i){
  	  debug_leds[i] = new_toggle_gpio(debug_led_pins[i], debug_led_pin_numbers[i]);
  }

  /*
   * Pins for these not known yet!
  Toggle_GPIO* raman_laser = new_toggle_gpio();
  Toggle_GPIO* uv_bulb = new_toggle_gpio();
  */

  //PLACEHOLDERS
  int total_channels = 8;
  int thermistor_channel_ns[3] = {0, 1, 2};
  int thermistor_channel_bs[3] = {0, 1, 2};

  //There are B and N heaters
  //Each heater needs a Toggle_GPIO struct and a thermistor
  //Each Toggle_GPIO is initialized with the pin and number

  Heater* heater_bs[3];
  Toggle_GPIO* heater_b_toggles[3];
  GPIO_TypeDef* heater_b_pins[3] = {GPIOB, GPIOB, GPIOA};
  uint16_t heater_b_pin_numbers[3] = {GPIO_PIN_13, GPIO_PIN_15, GPIO_PIN_8};
  DiagTempSensor* thermistor_bs[3];
  ADCSensor* adc_sensor_bs[3];

  Heater* heater_ns[3];
  Toggle_GPIO* heater_n_toggles[3];
  GPIO_TypeDef* heater_n_pins[3] = {GPIOB, GPIOC, GPIOA};
  uint16_t heater_n_pin_numbers[3] = {GPIO_PIN_14, GPIO_PIN_6, GPIO_PIN_9};
  DiagTempSensor* thermistor_ns[3];
  ADCSensor* adc_sensor_ns[3];

  for (int i = 0; i < 3; ++i){
  	  adc_sensor_bs[i] = new_adc_sensor(&hadc1, total_channels);
  	  thermistor_bs[i] = new_diag_temp_sensor(adc_sensor_bs[i], thermistor_channel_bs[i]);
  	  heater_b_toggles[i] = new_toggle_gpio(heater_b_pins[i], heater_b_pin_numbers[i]);
  	  heater_bs[i] = new_heater(heater_b_toggles[i], thermistor_bs[i]);
  	  adc_sensor_ns[i] = new_adc_sensor(&hadc1, total_channels);
  	  thermistor_ns[i] = new_diag_temp_sensor(adc_sensor_ns[i], thermistor_channel_ns[i]);
  	  heater_n_toggles[i] = new_toggle_gpio(heater_n_pins[i], heater_n_pin_numbers[i]);
      heater_ns[i] = new_heater(heater_n_toggles[i], thermistor_ns[i]);
  }

  init();




  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  ReceiveMessagesHandle = osThreadNew(ReceiveMessagesTask, NULL, &ReceiveMessages_attributes);
  SpectralTaskHandle = osThreadNew(SpectralTaskTask, NULL, &SpectralTask_attributes);
  ThermistorAndAutoShutoffTaskHandle = osThreadNew(ThermistorAndAutoShutoffTask, NULL, &ThermistorAndAutoShutoffTask_attributes);
  HeaterUpdatesTaskHandle = osThreadNew(HeaterUpdatesTaskTask, NULL, &HeaterUpdatesTask_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DEBUG_LED_0_Pin|DEBUG_LED_1_Pin|DEBUG_LED_2_Pin|WHITE_LED_0_Pin
                          |HEATER_N1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, UV_LED_2_Pin|UV_LED_1_Pin|UV_LED_0_Pin|WHITE_LED_2_Pin
                          |WHITE_LED_1_Pin|HEATER_B0_Pin|HEATER_N0_Pin|CAN_STANDBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, HEATER_B2_Pin|HEATER_N2_Pin|HEATER_B1_Pin|I2C_MUX_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DEBUG_LED_0_Pin DEBUG_LED_1_Pin DEBUG_LED_2_Pin WHITE_LED_0_Pin
                           HEATER_N1_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_0_Pin|DEBUG_LED_1_Pin|DEBUG_LED_2_Pin|WHITE_LED_0_Pin
                          |HEATER_N1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : UV_LED_2_Pin UV_LED_1_Pin UV_LED_0_Pin WHITE_LED_2_Pin
                           WHITE_LED_1_Pin HEATER_B0_Pin HEATER_N0_Pin CAN_STANDBY_Pin */
  GPIO_InitStruct.Pin = UV_LED_2_Pin|UV_LED_1_Pin|UV_LED_0_Pin|WHITE_LED_2_Pin
                          |WHITE_LED_1_Pin|HEATER_B0_Pin|HEATER_N0_Pin|CAN_STANDBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HEATER_B2_Pin HEATER_N2_Pin HEATER_B1_Pin I2C_MUX_RST_Pin */
  GPIO_InitStruct.Pin = HEATER_B2_Pin|HEATER_N2_Pin|HEATER_B1_Pin|I2C_MUX_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ReceiveMessages(void* argument) {
	uint32_t tick = osKernelGetTickCount();
	for(;;) {
		tick += osKernelGetTickFreq(); // 1 Hz
		receive_message();
		osDelayUntil(tick);
	}
}

void SpectralTask(void const * argument) {
	uint32_t tick = osKernelGetTickCount();
	for(;;) {
		tick += osKernelGetTickFreq(); // 1 Hz

		update_and_send_spectral();
		osDelayUntil(tick);
	}
}

void ThermistorAndAutoShutoffTask(void const * argument) {
	uint32_t tick = osKernelGetTickCount();
	for(;;) {
		tick += osKernelGetTickFreq(); // 1 Hz

		update_and_send_thermistor_and_auto_shutoff_if_applicable();
		osDelayUntil(tick);
	}
}

void HeaterUpdatesTask(void const * argument) {
	uint32_t tick = osKernelGetTickCount();
	for(;;) {
		tick += osKernelGetTickFreq(); // 1 Hz

		update_and_send_heater();
		osDelayUntil(tick);
	}
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
