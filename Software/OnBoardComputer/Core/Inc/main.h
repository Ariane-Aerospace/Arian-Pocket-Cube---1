/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void WriteOnSD(int n, uint8_t *Data, uint8_t Lth);
uint16_t CheckForFiles();
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct{
	uint32_t Ticks;
	uint16_t Vbat;
	uint16_t I_DS;
	uint16_t I_CH;
	uint16_t DS18_temp;
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
	int16_t MagX;
	int16_t MagY;
	int16_t MagZ;
	uint32_t MS5607_pres;
	float Alttitude;
	float Longtitude;
	float Height;
}Telemetry;

typedef struct{
	uint8_t DS18B20_OK;
	uint8_t LSM6_OK;
	uint8_t MS56_OK;
	uint8_t QMC_OK;
	uint8_t Radio_OK;
	uint8_t UCAM_OK;
}Health;

typedef struct{
	uint32_t DS18B20_REQ;
	uint32_t LSM6DS3_REQ;
	uint32_t QMC5883_REQ;
	uint32_t MS5607_REQ;
	uint32_t GPS_GetFull;
	uint32_t GPS_GetShort;

	uint32_t SD_WriteFull;
	uint32_t SD_WriteGPS_Full;

	uint32_t RF_SendMain;
	uint32_t RF_SendIMU;

	uint32_t GetPicture;


}SoftTimers;
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_CS_Pin GPIO_PIN_13
#define SD_CS_GPIO_Port GPIOC
#define CAM_RESET_Pin GPIO_PIN_4
#define CAM_RESET_GPIO_Port GPIOA
#define LSM_CS_Pin GPIO_PIN_10
#define LSM_CS_GPIO_Port GPIOB
#define DS18B20_DQ_Pin GPIO_PIN_8
#define DS18B20_DQ_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define Transmit_OK			0x00
#define Transmit_ERROR		0x01
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
