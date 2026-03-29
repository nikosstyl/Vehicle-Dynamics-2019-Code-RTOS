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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "asm330lhh_reg.h"
#include "config.h"
#include "string.h"
#include "stdarg.h"
#include "flash_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AXIS_NUM 3 /* 3-Axis measurement*/
#define SENSOR_BUS hi2c1
#define ACCEL_DATA_RATE ASM330LHH_XL_ODR_1667Hz	/* 6.6 kHz accelerometer data rate */
#define GYRO_DATA_RATE ASM330LHH_GY_ODR_1667Hz	/* 6.6 kHz gyroscope data rate */
#define IMU_SAMPLES_NUM 3

#define EVENT_CAN_TX_DONE 0x1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

/* Definitions for main */
osThreadId_t mainHandle;
const osThreadAttr_t main_attributes = {
  .name = "main",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for setParameters */
osThreadId_t setParametersHandle;
const osThreadAttr_t setParameters_attributes = {
  .name = "setParameters",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for rxParamQueue */
osMessageQueueId_t rxParamQueueHandle;
const osMessageQueueAttr_t rxParamQueue_attributes = {
  .name = "rxParamQueue"
};
/* Definitions for adcDataSemaphore */
osSemaphoreId_t adcDataSemaphoreHandle;
const osSemaphoreAttr_t adcDataSemaphore_attributes = {
  .name = "adcDataSemaphore"
};
/* Definitions for i2cTxSemaphore */
osSemaphoreId_t i2cTxSemaphoreHandle;
const osSemaphoreAttr_t i2cTxSemaphore_attributes = {
  .name = "i2cTxSemaphore"
};
/* Definitions for i2cRxSemaphore */
osSemaphoreId_t i2cRxSemaphoreHandle;
const osSemaphoreAttr_t i2cRxSemaphore_attributes = {
  .name = "i2cRxSemaphore"
};
/* Definitions for canTxSemaphore */
osSemaphoreId_t canTxSemaphoreHandle;
const osSemaphoreAttr_t canTxSemaphore_attributes = {
  .name = "canTxSemaphore"
};
/* USER CODE BEGIN PV */

/* Callback function for ADC conversion */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

/* Callback functions for CAN TX */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan);

/* Callback function for CAN RX */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

/* Swaps the ADC signals to their correct positions. */
void ADC_swap(ADC_Samples_t *ret);

/* Function that sends data over CAN Bus CORRECTLY */
HAL_StatusTypeDef Send_CAN_Msg(uint32_t id, uint32_t dlc, uint8_t* aData);

void SimpleFilter(ADC_Samples_t *values, axis3bit16_t *accel, axis3bit16_t *gyro);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
void mainTask(void *argument);
void setParametersTask(void *argument);

/* USER CODE BEGIN PFP */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

void ADC_swap(ADC_Samples_t *ret);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;	/* CAN Header */
uint32_t mailbox;				/* CAN Bus mailbox */
ADC_Samples_t ADC_Samples={0}, Final_ADC_Values={0};		/* ADC Data */

static const canBusConfig_t defaultAlphaConfig = {.alpha = DEFAULT_FILTER_ALPHA, .signal_to_smooth=POS_ADC_CHANNEL_1};

// This is a simple float array that holds all the alpha configurations.
// They are read from the memory during startup (toDo) or get set by CAN Bus
// Maybe a reset to the memory contents would be nice? Idk let's see.
float alphaConfigs[TOTAL_POSITIONS]; 

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Print useful information during start-up.
  ItmPrintf("Vehicle Dynamics Board, 2019, Centaurus Racing Team\n");
  ItmPrintf("Firmware Version: %s\nCompiled at: %s\n\n", FIRMWARE_VERSION, COMPILE_DATETIME);
  ItmPrintf("Compiled with the following variables:\n");
  #if IS_COG
  ItmPrintf("\tCOG Board\n\tCANBUS_ID_1: %x\n\tCANBUS_ID_2: %d\n\tCANBUS_ID_3: %x\n\tCANBUS_ID_4: %x\n", CANBUS_ID_1, CANBUS_ID_2, CANBUS_ID_3, CANBUS_ID_4);
  #else
  ItmPrintf("\tCANBUS_ID_1: %x\n\tCANBUS_ID_2: %d\n", CANBUS_ID_1, CANBUS_ID_2);
  #endif

	uint8_t retries = 0;
	while (retries < 4) {
		if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
			retries++;
		}
		else {
			break;
		}
		if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
			retries++;
		}
		else {
			break;
		}
	}

	HAL_StatusTypeDef status = HAL_ADC_Start(&hadc2);
	if (status != HAL_OK) {
		Error_Handler();
	}
	status = HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)ADC_Samples.i16, 8);
	if (status != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
		Error_Handler();
	}

  // Read the alpha configs during bootup
  float tmpConfigs[TOTAL_POSITIONS] = {0};
  Flash_Read(FLASH_DATA_ADDR, tmpConfigs, sizeof(tmpConfigs));
  for (POSITION_INDEX i=0;i<TOTAL_POSITIONS;i++) {
    if (tmpConfigs[i] == 0) {
      tmpConfigs[i] = DEFAULT_FILTER_ALPHA;
    }
  }
  memcpy(alphaConfigs, tmpConfigs, sizeof(alphaConfigs));
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of adcDataSemaphore */
  adcDataSemaphoreHandle = osSemaphoreNew(1, 0, &adcDataSemaphore_attributes);

  /* creation of i2cTxSemaphore */
  i2cTxSemaphoreHandle = osSemaphoreNew(1, 1, &i2cTxSemaphore_attributes);

  /* creation of i2cRxSemaphore */
  i2cRxSemaphoreHandle = osSemaphoreNew(1, 1, &i2cRxSemaphore_attributes);

  /* creation of canTxSemaphore */
  canTxSemaphoreHandle = osSemaphoreNew(3, 3, &canTxSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of rxParamQueue */
  rxParamQueueHandle = osMessageQueueNew (5, sizeof(canBusConfig_t), &rxParamQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of main */
  mainHandle = osThreadNew(mainTask, NULL, &main_attributes);

  /* creation of setParameters */
  setParametersHandle = osThreadNew(setParametersTask, NULL, &setParameters_attributes);

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
	while (1) {
    
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV256;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_12_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  	// Configure CAN RX filter to accept EXTENDED IDs from 0x300 to 0x33F
	CAN_FilterTypeDef rxFilterConfig = {0};

	// Use filter bank 0
	rxFilterConfig.FilterBank = 0;

	// Use mask mode (ID & Mask comparison)
	rxFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;

	// Use 32-bit scale for one full extended ID filter
	rxFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	// Assign accepted messages to FIFO0
	rxFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;

	// Enable the filter
	rxFilterConfig.FilterActivation = CAN_FILTER_ENABLE;

	/*
	* Extended (29-bit) ID filters are left-shifted by 3 bits.
	* Also, the IDE bit (bit 2) must be set to 1 to indicate extended IDs.
	*
	* Base ID:  0x300
	* Mask:     0x7C0 (ignore lower 6 bits)
	* Accepts all EXT IDs from 0x300 to 0x33F.
	*/

	// Set the base extended ID
	uint32_t baseId = (0x300 << 3) | (1 << 2); // include IDE=1
	uint32_t maskId = (0x7C0 << 3) | (1 << 2); // mask for extended IDs only

	rxFilterConfig.FilterIdHigh = (baseId >> 16) & 0xFFFF;
	rxFilterConfig.FilterIdLow  = baseId & 0xFFFF;
	rxFilterConfig.FilterMaskIdHigh = (maskId >> 16) & 0xFFFF;
	rxFilterConfig.FilterMaskIdLow  = maskId & 0xFFFF;

	// Apply the filter
	if (HAL_CAN_ConfigFilter(&hcan, &rxFilterConfig) != HAL_OK)
	{
		// Configuration Error
		Error_Handler();
	}

  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.Timing = 0x00201D2B;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	osSemaphoreRelease(adcDataSemaphoreHandle);
}

void ADC_swap(ADC_Samples_t *ret) {
	ret->i16[POS_ADC_CHANNEL_1] = Final_ADC_Values.i16[POS_ADC_CHANNEL_1];
	ret->i16[POS_ADC_CHANNEL_2] = Final_ADC_Values.i16[POS_ADC_CHANNEL_3];
	ret->i16[POS_ADC_CHANNEL_3] = Final_ADC_Values.i16[POS_ADC_CHANNEL_2];
	ret->i16[POS_ADC_CHANNEL_4] = Final_ADC_Values.i16[POS_ADC_CHANNEL_4];
	ret->i16[POS_ADC_CHANNEL_5] = Final_ADC_Values.i16[POS_ADC_CHANNEL_6];
	ret->i16[POS_ADC_CHANNEL_6] = Final_ADC_Values.i16[POS_ADC_CHANNEL_8];
	ret->i16[POS_ADC_CHANNEL_7] = Final_ADC_Values.i16[POS_ADC_CHANNEL_5];
	ret->i16[POS_ADC_CHANNEL_8] = Final_ADC_Values.i16[POS_ADC_CHANNEL_7];
}

HAL_StatusTypeDef Send_CAN_Msg(uint32_t id, uint32_t dlc, uint8_t* aData) {
	HAL_StatusTypeDef status;
	
  if (dlc > 8) { // MAX bytes a single can msg can send once.
    return HAL_ERROR;
  }
  
  TxHeader.ExtId = id;
	TxHeader.DLC = dlc;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	uint32_t mailbox;
	
	if (osSemaphoreAcquire(canTxSemaphoreHandle, pdMS_TO_TICKS(10)) != osOK) {
		return HAL_TIMEOUT;
	}

	status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, aData, &mailbox);
	return status;
}

static inline void CAN_TX_Callback() {
	osSemaphoreRelease(canTxSemaphoreHandle);
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	CAN_TX_Callback();
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	CAN_TX_Callback();
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	CAN_TX_Callback();
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan){
	CAN_TX_Callback();
}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan){
	CAN_TX_Callback();
}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan){
	CAN_TX_Callback();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8] = {0};

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		// Reception Error
		return;
	}

	if (RxHeader.ExtId != CANBUS_RCV_ADDR) {
		// msg not targeting our board, abort.
		return;
	}

  if (RxHeader.DLC != 3) {
    // We want to read just 2 params
    return;
  }

	canBusConfig_t param = {0};
	param.signal_to_smooth    = RxData[0]; // Number ranging from 0 to TOTAL_POSITIONS - 1
  param.alpha               = (float)RxData[1]; // Must convert this to 0...1 value in the proper task.
  param.permanently_stored  = RxData[2]; // 1 to permanently store this alpha configuration
  
  if (param.signal_to_smooth >= TOTAL_POSITIONS) {
    return;
  }

	// timeout must be 0, we're in ISR context
	osMessageQueuePut(rxParamQueueHandle, &param, 0, 0);
}

void SimpleFilter(ADC_Samples_t *values, axis3bit16_t *accel, axis3bit16_t *gyro) {
  static float filtered[TOTAL_POSITIONS];
  static bool first_run = true;

  if (first_run) { // Set to zero on first run
    first_run = false;
    memset(filtered, 0, sizeof(filtered));
  }

  // alphaConfigs is a global variable set by an RTOS task. It is not set frequently,
  // thus guarding this variable is not required.
  // (it's a good practice to do though, I'm just lazy)
	for (POSITION_INDEX i = POS_ADC_CHANNEL_1; i <= POS_ADC_CHANNEL_8; i++) {
		filtered[i] += alphaConfigs[i] * ((float)values->i16[i] - filtered[i]);
		values->i16[i] = (int16_t)filtered[i];
	}

  for (POSITION_INDEX i = POS_IMU_ACCEL_X, j=0; i<= POS_IMU_ACCEL_Z; i++,j++) {
    filtered[i] += alphaConfigs[i] * ((float)accel->i16[j] - filtered[i]);
    accel->i16[j] = (int16_t) filtered[i];
  }

  for (POSITION_INDEX i = POS_IMU_GYRO_X, j=0; i<= POS_IMU_GYRO_Z; i++,j++) {
    filtered[i] += alphaConfigs[i] * ((float)gyro->i16[j] - filtered[i]);
    gyro->i16[j] = (int16_t) filtered[i];
  }

}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
*/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	osSemaphoreAcquire(i2cTxSemaphoreHandle, osWaitForever);
	return(HAL_I2C_Mem_Write_DMA(&hi2c1, ASM330LHH_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len));

}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	osSemaphoreAcquire(i2cRxSemaphoreHandle, osWaitForever);
	return(HAL_I2C_Mem_Read_DMA(&hi2c1, ASM330LHH_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len));

}

void initializeIMU(stmdev_ctx_t *dev_ctx) {
	asm330lhh_device_conf_set(dev_ctx, PROPERTY_ENABLE);			/* Start device configuration */
	asm330lhh_block_data_update_set(dev_ctx, PROPERTY_ENABLE);	/* Enable Block Data Update */
	asm330lhh_xl_data_rate_set(dev_ctx, ACCEL_DATA_RATE);	/* Set Accelerometer Output Data Rate */
	asm330lhh_gy_data_rate_set(dev_ctx, GYRO_DATA_RATE);	/* Set Gyroscope Output Data Rate */
	asm330lhh_xl_full_scale_set(dev_ctx, ASM330LHH_4g);			 /* Set 4G full scale for accelerometer. */
	asm330lhh_gy_full_scale_set(dev_ctx, ASM330LHH_2000dps);	/* Set 2000 dps full scale for gyroscope. */
	asm330lhh_xl_hp_path_on_out_set(dev_ctx, ASM330LHH_LP_ODR_DIV_100);	/* Configure filtering chain(No aux interface) */
	asm330lhh_xl_filter_lp2_set(dev_ctx, PROPERTY_ENABLE);				/*  Accelerometer - LPF1 + LPF2 path */
}

void readFromIMU(stmdev_ctx_t *dev_ctx, axis3bit16_t *accel, axis3bit16_t *gyro) {
	uint8_t availableData = 0;
  for (int i=0; i<5 && availableData==0; i++) { // Try to get data 5 times
	  asm330lhh_xl_flag_data_ready_get(dev_ctx, &availableData); // returns 0 for no error
  }
	if (!availableData) {
		return;
	}
	asm330lhh_acceleration_raw_get(dev_ctx, accel->i16);
	asm330lhh_angular_rate_raw_get(dev_ctx, gyro->i16);

	for (int i = 0; i < AXIS_NUM; i++) {
		accel->i16[i] = asm330lhh_from_fs4g_to_mg(accel->i16[i]);
		gyro->i16[i] = asm330lhh_from_fs2000dps_to_mdps(gyro->i16[i]);
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
	osSemaphoreRelease(i2cTxSemaphoreHandle);
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	osSemaphoreRelease(i2cRxSemaphoreHandle);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	Error_Handler();
}

int ItmPrintf(const char *format, ...) {
  char tmpBuffer[2048]={'\0'}; 
  
  va_list args;
  va_start(args, format);
  int retval = vsnprintf(tmpBuffer, sizeof(tmpBuffer), format, args);
  if (retval < 0 || retval > sizeof(tmpBuffer)) {
    ITM_SendChar('?');
  }
  else {
    for (int i=0;i<retval;i++) {
      ITM_SendChar(tmpBuffer[i]);
    }
  }
  va_end(args);
  return retval;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_mainTask */
/**
  * @brief  Main function that sends data to the CAN Bus
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_mainTask */
void mainTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	#if IS_COG==1
	stmdev_ctx_t dev_ctx;
	dev_ctx.write_reg = platform_write;	/* Define the write function for the IMU library. */
	dev_ctx.read_reg = platform_read; /* Define the read function for the IMU library. */
	dev_ctx.handle = &SENSOR_BUS;

	initializeIMU(&dev_ctx);
	#endif

	while (1) {
		ADC_Samples_t values = {0};
		osSemaphoreAcquire(adcDataSemaphoreHandle, pdMS_TO_TICKS(5));
		Final_ADC_Values = ADC_Samples; // Buffer the ADC values during safe read
		ADC_swap(&values);
    
		#if IS_COG==0
    SimpleFilter(&values, NULL, NULL);
    
		Send_CAN_Msg(CANBUS_ID_1, 8, &values.u8[0]); // Sends ADC data vol1
		Send_CAN_Msg(CANBUS_ID_2, 8, &values.u8[8]); // Sends ADC data vol2
    #elif IS_COG==1
		axis3bit16_t accel, gyro;
		readFromIMU(&dev_ctx, &accel, &gyro);
    
    SimpleFilter(&values, &accel, &gyro);

    Send_CAN_Msg(CANBUS_ID_1, 6, &accel.u8[0]); // Sends accel data
		Send_CAN_Msg(CANBUS_ID_2, 6, &gyro.u8[0]); // Sends gyro data
		Send_CAN_Msg(CANBUS_ID_3, 8, &values.u8[0]); // Sends ADC data vol1
		Send_CAN_Msg(CANBUS_ID_4, 8, &values.u8[8]); // Sends ADC data vol2
		#endif

		osDelay(pdMS_TO_TICKS(10));
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_setParametersTask */
/**
* @brief Function implementing the setParameters thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_setParametersTask */
void setParametersTask(void *argument)
{
  /* USER CODE BEGIN setParametersTask */
  /* Infinite loop */
	while (1) {
		canBusConfig_t rxParam = defaultAlphaConfig;
		osMessageQueueGet(rxParamQueueHandle, &rxParam, NULL, osWaitForever);
		
    rxParam.alpha = rxParam.alpha / 255.0f; // Converting this to 0...1 number
    alphaConfigs[rxParam.signal_to_smooth] = rxParam.alpha;
    
    if (rxParam.permanently_stored) {
      float tmpConfigs[TOTAL_POSITIONS];
      memcpy(tmpConfigs, alphaConfigs, sizeof(tmpConfigs));
      Flash_Write(FLASH_DATA_ADDR, tmpConfigs, sizeof(tmpConfigs));
    }
	}
  /* USER CODE END setParametersTask */
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
  while (1)
  {
	NVIC_SystemReset();
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
