/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SMA_Filter/SMA_filter_lib.h"
#include "DS18B20/DS18B20.h"
#include "LSM6DS3/LSM6DS3.h"
#include "QMC5883L/QMC5883L.h"
#include "MS5607/MS5607.h"
#include "RadioTransmit/Transmitter.h"
#include "UCAM3/UCAM3.h"
#include <stdio.h>
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
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
Telemetry telemetry;
Health health;
SoftTimers softTimers;

uint16_t ADC_data[3];
uint16_t SMA_Filter_Buffer_Vbat[SMA_FILTER_ORDER] = { 0, };
uint16_t SMA_Filter_Buffer_ICHARGE[SMA_FILTER_ORDER] = { 0, };
uint16_t SMA_Filter_Buffer_IDISCHARGE[SMA_FILTER_ORDER] = { 0, };

FATFS fs;
FIL fil;
FRESULT fresult;
char buffer[1024];
UINT br, bw;
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
uint8_t NameLength = 35;
uint16_t PicNum = 0;

uint16_t ADC_Raw[3] = {0x00, 0x00, 0x00};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
float ADC_ConvertVBat(uint16_t VbatRaw);
float ADC_Convert_IDS(uint16_t I_raw);
float ADC_Convert_ICH(uint16_t I_raw);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Delay_us(uint16_t Time) {
	__HAL_TIM_SET_COUNTER(&htim11, 0);

	while (__HAL_TIM_GET_COUNTER(&htim11) < Time);
}

void StructsInit() {

}

uint8_t TransmitRadio(uint8_t *Data, uint8_t Length) {
	uint8_t CommData[3] = { 0x0D, 0x01, Length };

	HAL_StatusTypeDef Status = HAL_I2C_Master_Transmit(&hi2c1, 0x00, CommData, 3, 1000);

	if (Status != HAL_OK) {
		return Transmit_ERROR;
	}

	Delay_us(1000);
	HAL_I2C_Master_Transmit(&hi2c1, 0x00, Data, Length, 1000);

	return Transmit_OK;
}

uint8_t GetAllGPS()
{
	UINT bw;
	uint8_t CommData[3] = { 0x0D, 0x03, 0x00 };

	HAL_StatusTypeDef Status = HAL_I2C_Master_Transmit(&hi2c1, 0x00, CommData, 3, 1000);

	if (Status != HAL_OK) {
		return Transmit_ERROR;
	}

	uint8_t Lth;
	HAL_I2C_Master_Receive(&hi2c1, 0x00, &Lth, 1, 1000);
	uint8_t AllData[Lth];
	HAL_I2C_Master_Receive(&hi2c1, 0x00, AllData, Lth, 1000);

	fresult = f_open(&fil, "AllGPS.bin", FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	fresult = f_write(&fil, AllData, Lth, &bw);
	fresult = f_close(&fil);

	return Transmit_OK;
}

uint8_t GetGPS(float *Height, float *Longitude, float *Attitude) {
	uint8_t CommData[3] = { 0x0D, 0x02, 0x00 };
	uint8_t Data[12];
	HAL_StatusTypeDef Status = HAL_I2C_Master_Transmit(&hi2c1, 0x00, CommData,
			3, 1000);

	if (Status != HAL_OK) {
		return Transmit_ERROR;
	}

	HAL_I2C_Master_Receive(&hi2c1, 0x00, Data, 12, 1000);

	*Height = (Data[0] | Data[1] | Data[2] | Data[3]);
	*Longitude = (Data[4] | Data[5] | Data[6] | Data[7]);
	*Attitude = (Data[8] | Data[9] | Data[10] | Data[11]);

	return Transmit_OK;
}

void HealthMonitor() {
	if (health.DS18B20_OK == 0) {
		HAL_TIM_Base_Start(&htim11);
		if (DS18_Init(DS18_SKIP_ROM) == DS18_OK) {
			DS18_StartConv(DS18_SKIP_ROM);
			health.DS18B20_OK = 1;
		}
	}
	if (health.LSM6_OK == 0) {
		LSM6DS3_Init();
		health.LSM6_OK = 1;
	}
	if (health.QMC_OK == 0) {
		QMC5883L_Init(QMC5883L_ODR_100HZ, QMC5883L_RNG_2G, QMC5883L_OSR_512,
		QMC5883L_MODE_CONT);
		health.QMC_OK = 1;
	}
	if (health.MS56_OK == 0) {
		MS5607_Init();
		health.MS56_OK = 1;
	}
	if(health.UCAM_OK == 0){
		if(UCAM3_Init() == UCAM3_OK)
		{
			health.UCAM_OK = 1;
		}
	}

}
void GetData() {

	if (softTimers.DS18B20_REQ > 800) {

		uint16_t temp = 0;

		DS18_GetData(DS18_SKIP_ROM, &temp);
		DS18_StartConv(DS18_SKIP_ROM);
		telemetry.DS18_temp = temp;
		softTimers.DS18B20_REQ = 0;

	}
	if (softTimers.LSM6DS3_REQ > 20) {
		int16_t LSM6DS3_Acc[3];
		int16_t LSM6DS3_Gyro[3];
		LSM6DS3_GetData(LSM6DS3_Acc, LSM6DS3_Gyro);
		telemetry.AccX = LSM6DS3_Acc[0];
		telemetry.GyroX = LSM6DS3_Gyro[0];
		telemetry.AccY = LSM6DS3_Acc[1];
		telemetry.GyroY = LSM6DS3_Gyro[1];
		telemetry.AccZ = LSM6DS3_Acc[2];
		telemetry.GyroZ = LSM6DS3_Gyro[2];
		softTimers.LSM6DS3_REQ = 0;
	}
	if (softTimers.QMC5883_REQ > 100) {
		int16_t MagX = 0;
		int16_t MagY = 0;
		int16_t MagZ = 0;
		QMC5883L_Read_Mag_Data(&MagX, &MagY, &MagZ);
		telemetry.MagX = MagX;
		telemetry.MagY = MagY;
		telemetry.MagZ = MagZ;
		softTimers.QMC5883_REQ = 0;
	}
	if (softTimers.MS5607_REQ > 100) {
		uint32_t MS56_Pres = 0;
		MS5607_GetPres(&MS56_Pres);
		telemetry.MS5607_pres = MS56_Pres;
		softTimers.MS5607_REQ = 0;
	}
	if (softTimers.GPS_GetFull > 5000) {
		softTimers.GPS_GetFull = 0;
		GetAllGPS();
	}
	if (softTimers.GPS_GetShort > 1000) {

	}
	if (softTimers.GetPicture > 30000) {
		if(UCAM3_GetPucture(PicNum) != UCAM3_OK)
		{
			health.UCAM_OK = 0;
		}
		PicNum++;
		softTimers.GetPicture = 0;
	}
}
void SendData() {

	if (softTimers.SD_WriteFull > 100) {
		uint8_t data[48];
		SetAllData(&telemetry, data);

		fresult = f_open(&fil, "MainTLM.bin", FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		fresult = f_write(&fil, data, 48, &bw);
		fresult = f_close(&fil);
	}

	if (softTimers.RF_SendMain > 500) {
		softTimers.RF_SendMain = 0;
		uint8_t data[24];
		SetRadioData(&telemetry, data);

		TransmitRadio(data, 24);
	}

}

uint16_t CheckForFiles() {
	char fname[NameLength];
	uint16_t n = 0;

	sprintf(&fname[0], "%d.jpeg", n);
	fresult = f_stat(fname, NULL);

	while (fresult != FR_NO_FILE) {
		char fBuffname[NameLength];

		for (int i = 0; i < NameLength; i++) {
			fBuffname[i] = fname[i];
		}

		n++;
		sprintf(&fBuffname[0], "%d.jpeg", n);
		fresult = f_stat(fBuffname, NULL);
	}

	return n;
}

void WriteOnSD(int n, uint8_t *Data, uint8_t Lth) {
	UINT bw;

	char fname[NameLength];
	sprintf(&fname[0], "%d.jpeg", n);
	fresult = f_open(&fil, &fname[0],
			FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	fresult = f_write(&fil, Data, Lth, &bw);

	fresult = f_close(&fil);
}
uint8_t Status = UCAM3_ERROR;
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
	StructsInit();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, SET);
	LSM6DS3_CS_Prepare();

	HAL_Delay(100);


	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim11);
	fresult = f_mount(&fs, "", 0);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Raw, 3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



	while (1) {
		HealthMonitor();
		GetData();
		SendData();


		telemetry.Vbat = ADC_Raw[0];
		telemetry.I_CH = ADC_Raw[1];
		telemetry.I_DS = ADC_Raw[2];
		telemetry.Ticks = __HAL_TIM_GET_COUNTER(&htim2);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 84-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CAM_RESET_Pin|DS18B20_DQ_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAM_RESET_Pin */
  GPIO_InitStruct.Pin = CAM_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAM_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM_CS_Pin */
  GPIO_InitStruct.Pin = LSM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LSM_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_DQ_Pin */
  GPIO_InitStruct.Pin = DS18B20_DQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DS18B20_DQ_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void) {
	softTimers.DS18B20_REQ++;
	softTimers.LSM6DS3_REQ++;
	softTimers.QMC5883_REQ++;
	softTimers.MS5607_REQ++;
	softTimers.GPS_GetFull++;
	softTimers.GPS_GetShort++;

	softTimers.SD_WriteFull++;
	softTimers.SD_WriteGPS_Full++;

	softTimers.RF_SendMain++;
	softTimers.RF_SendIMU++;

	softTimers.GetPicture++;
}
/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	 if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
	 {
	 telemetry.Vbat = SMA_FILTER_Get_Value(SMA_Filter_Buffer_Vbat, &ADC_data[0]);
	 telemetry.I_CH = SMA_FILTER_Get_Value(SMA_Filter_Buffer_ICHARGE, &ADC_data[1]);
	 telemetry.I_DS = SMA_FILTER_Get_Value(SMA_Filter_Buffer_IDISCHARGE, &ADC_data[2]);
	 }
}
*/
float ADC_ConvertVBat(uint16_t VbatRaw) {
	return 2 * (VbatRaw * 3.3f / 4096.0f);
}
float ADC_Convert_IDS(uint16_t I_raw) {
	return I_raw * 0.0289 - 5.5502 + 10;
}
float ADC_Convert_ICH(uint16_t I_raw) {
	return I_raw * 0.0289 - 5.5502 - 10;
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
	while (1) {
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
