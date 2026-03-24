/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* Union used in driver of the IMU. */
typedef union{
	int16_t i16[3]; // 0:x, 1:y, 2:z
	uint8_t u8[6];
}axis3bit16_t;

/* Union used when converting ADC values. */
typedef union{
	int16_t i16[8];
	uint8_t u8[16];
}ADC_Samples_t;

typedef struct _canBusConfig {
	int16_t multiplier;
	int16_t divider;
	int16_t offset;
	uint8_t index;
}canBusConfig_t;

enum POSITION_INDEX {
	POS_ADC_CHANNEL_1 = 0,
	POS_ADC_CHANNEL_2,
	POS_ADC_CHANNEL_3,
	POS_ADC_CHANNEL_4,
	POS_ADC_CHANNEL_5,
	POS_ADC_CHANNEL_6,
	POS_ADC_CHANNEL_7,
	POS_ADC_CHANNEL_8,
	POS_IMU_ACCEL_X,
	POS_IMU_ACCEL_Y,
	POS_IMU_ACCEL_Z,
	POS_IMU_GYRO_X,
	POS_IMU_GYRO_Y,
	POS_IMU_GYRO_Z
};

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/**
 * @brief A printf-like function that sends data
 *        over the ITM bus. The arguments are exactly the same 
 *        as in printf.
 */
int ItmPrintf(const char *format, ...);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

#define ADC_CHANNELS_NUM 8
#define IMU_CHANNELS_NUM 6
#define TOTAL_CHANNELS_NUM (ADC_CHANNELS_NUM + IMU_CHANNELS_NUM)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
