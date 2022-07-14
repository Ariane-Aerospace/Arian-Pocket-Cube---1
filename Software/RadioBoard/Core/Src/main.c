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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t id;
	uint8_t type;
	uint8_t parLth;
	uint8_t params[10];

}CMD;

enum State
{
	Start = 0,
	Error = -2,
	Fin = -1,

	Beg_mark = 1,	// $GPGGA,
	time	 = 2,	// hhmmss.ss,
	attitude = 3,	// llll.ll,
	attitudeDIR = 4,// a,

	longtitude = 5,		// yyyyy.yy,
	longtitudeDIR = 6,	// a,
	gps_quality = 7,	// x,
	satteites = 8,		// xx,

	HDOP = 9,		// x.x,
	height = 10,	// x.x,
	heightUNIT = 11,// M,
	geo_sep = 12,	// x.x,

	geo_sepUNITS = 13,// M,
	age_diff = 14,	// x.x,
	ref_st = 15,	// xxxx*
	chsum = 16,		// hh\n\r
};


typedef struct
{
	double time;
	double attitude;
	char attitudeDIR;
	double longtitude;
	char longtitudeDIR;
	uint8_t gps_quality;
	uint8_t satteites;
	double HDOP;
	double height;
	char heightUNIT;
	double geo_sep;
	char geo_sepUNITS;
	double age_diff;
	uint16_t ref_st;
	uint8_t chsum;
} GGA;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OBC_CTRL 	2
#define SYNC 		1
#define ERROR 		0
#define BUF_SZ 		32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

CMD queue[10];
uint8_t Queue_size = 0;

uint32_t TIM_GNSS_Send = 0;


uint8_t pGNSS_Data[512];
uint8_t pRF_Data[100];
uint8_t gnss_size = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t GNSS_decoder(char* str, GGA* Ret);
uint32_t parse(char* s, uint32_t l, enum State S, GGA* struc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SYSTICK_Callback(void)
{
	TIM_GNSS_Send++;
}
uint8_t XOR(uint8_t* data, uint8_t lth)
{
	return 5;
}
void ProcessCMD(CMD cmd)
{
	if(cmd.type == SYNC)
	{
		uint8_t answer[7];
		answer[0] = 0x0D;
		answer[1] = 'C';
		answer[2] = cmd.id;
		answer[3] = cmd.type;
		answer[4] = 0;		//Длина данных
		answer[5] = XOR(answer, 5);
		HAL_UART_Transmit_IT(&huart1, answer, 6);
	}
	if(cmd.type == OBC_CTRL)
	{
		//ToProg(0), RST(1), RST_L(2), RST_H(3), BOOT_L(4), BOOT_H(5)
		if(cmd.params[0] == 0)//ToProg
		{
			HAL_GPIO_WritePin(PROG_BOOT_GPIO_Port, PROG_BOOT_Pin, SET);
			HAL_GPIO_WritePin(PROG_RST_GPIO_Port, PROG_RST_Pin, RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(PROG_RST_GPIO_Port, PROG_RST_Pin, SET);
			HAL_GPIO_WritePin(PROG_BOOT_GPIO_Port, PROG_BOOT_Pin, RESET);
		}
		else if(cmd.params[0] == 1)//RST
		{
			HAL_GPIO_WritePin(PROG_RST_GPIO_Port, PROG_RST_Pin, RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(PROG_RST_GPIO_Port, PROG_RST_Pin, SET);
		}
		else if(cmd.params[0] == 2)//RST_L
		{
			HAL_GPIO_WritePin(PROG_RST_GPIO_Port, PROG_RST_Pin, RESET);
		}
		else if(cmd.params[0] == 3)//RST_H
		{
			HAL_GPIO_WritePin(PROG_RST_GPIO_Port, PROG_RST_Pin, SET);
		}
		else if(cmd.params[0] == 4)//BOOT_L
		{
			HAL_GPIO_WritePin(PROG_BOOT_GPIO_Port, PROG_BOOT_Pin, RESET);
		}
		else if(cmd.params[0] == 5)//BOOT_H
		{
			HAL_GPIO_WritePin(PROG_BOOT_GPIO_Port, PROG_BOOT_Pin, SET);
		}
		uint8_t answer[7];
		answer[0] = 0x0D;
		answer[1] = 'C';
		answer[2] = cmd.id;
		answer[3] = cmd.type;
		answer[4] = 1;		//Длина данных
		answer[6] = 0;
		answer[6] = XOR(answer, 6);
		HAL_UART_Transmit_IT(&huart1, answer, 7);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART1) //Radio
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, pRF_Data, 100);
		if(pRF_Data[0] == 0x0D && pRF_Data[1] == 'C')
		{
			if(XOR(pRF_Data, Size) == pRF_Data[Size-1])
			{
				CMD newCMD;
				newCMD.id = pRF_Data[2];
				newCMD.type = pRF_Data[3];
				newCMD.parLth = pRF_Data[4];
				for(uint8_t parCnt = 0; parCnt < newCMD.parLth; parCnt++)
					newCMD.params[parCnt] = pRF_Data[5+parCnt];


				queue[Queue_size] = newCMD;
				Queue_size++;

			}else
			{
				//bad CRC
			}
		}
		else
		{
			//bad cmd
		}
	}
	if(huart->Instance == USART2) //GPS
	{
		//GGA pGNSS_Data2;
		//GNSS_decoder((char*)&pGNSS_Data[0], &pGNSS_Data2 );
		gnss_size = Size;

		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, pGNSS_Data, 512);

	}
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
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, SET);
  //HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, SET); //Для JDY
  HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, RESET); //Для E220

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, pGNSS_Data, 512); //GPS
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, pRF_Data, 100); //RF

  /*while(1)
  {
	  uint8_t data[3] = { 0xAA, 0xBB, 0xCC };
	  HAL_UART_Transmit(&huart1, data, 3, 1000);

	  HAL_Delay(1000);
  }*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(Queue_size > 0)
	  {
		  ProcessCMD(queue[Queue_size-1]);
		  Queue_size--;
	  }
	  if(TIM_GNSS_Send > 2000)
	  {
		  HAL_UART_Transmit(&huart1, pGNSS_Data, gnss_size, 1000);
		  TIM_GNSS_Send = 0;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 170;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, M1_Pin|M0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PROG_BOOT_Pin|PROG_RST_Pin|MEM_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : M1_Pin M0_Pin */
  GPIO_InitStruct.Pin = M1_Pin|M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PROG_BOOT_Pin PROG_RST_Pin MEM_CS_Pin */
  GPIO_InitStruct.Pin = PROG_BOOT_Pin|PROG_RST_Pin|MEM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_AUX_Pin */
  GPIO_InitStruct.Pin = LORA_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_AUX_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint32_t parse(char* s, uint32_t l, enum State S, GGA* struc)
{
	char buf[BUF_SZ];
	strncpy(&buf[0], s, l);
	buf[l] = 0;
	uint16_t ret = 0;
	switch (S)
	{
		case time:
			struc->time = strtod(buf,NULL);
			break;
		case attitude:
			struc->attitude = strtod(buf,NULL);
			break;
		case longtitude:
			struc->longtitude = strtod(buf,NULL);
			break;
		case HDOP:
			struc->HDOP = strtod(buf,NULL);
			break;
		case height:
			struc->height = strtod(buf,NULL);
			break;
		case geo_sep:
			struc->geo_sep = strtod(buf,NULL);
			break;
		case age_diff:
			struc->age_diff = strtod(buf,NULL);
			break;

		case attitudeDIR:
			struc->attitudeDIR = buf[0];
			break;
		case longtitudeDIR:
			struc->longtitudeDIR = buf[0];
			break;
		case heightUNIT:
			struc->heightUNIT = buf[0];
			break;
		case geo_sepUNITS:
			struc->geo_sepUNITS = buf[0];
			break;

		case gps_quality:
			struc->gps_quality = atoi(buf);
			break;
		case satteites:
			struc->satteites = atoi(buf);
			break;
		case ref_st:
			struc->ref_st = atoi(buf);
			break;

		case chsum:
			sscanf(buf, "%x", (unsigned int *)&ret);
			break;
		case Beg_mark:
			if (strcmp(buf,"gpgga") != 0)
				ret = -1;
			break;
		default:
			ret = -1;
	}
	return ret;
}

uint8_t GNSS_decoder(char* str, GGA* Ret)
{
	uint32_t i = 0;
	uint32_t len = 0;
	enum State Current = Start;
	while (Current != Error && str[i] != 0 && Current != Fin)
	{
		str[i] = tolower(str[i]);
		switch (str[i])
		{
			case '$':
				Current = Beg_mark;
				break;
			case ',':
				if (Current != Start && Current != Error && Current != Fin)
				{
					uint8_t parse_r = parse(str + i - len, len, Current, Ret);
					if (parse_r < 0)
						Current = Error;
					else
					{
						Current ++;
						len = 0;
					}
				}
				break;
			case 'a' ... 'z':
				if (
					(Current != Beg_mark && Current != attitudeDIR && Current != longtitudeDIR &&
					Current != heightUNIT && Current != geo_sepUNITS && Current != chsum) ||
					(Current == chsum && !isxdigit((unsigned char)str[i])) )
					Current = Error;
				else
					len++;
				break;
			case '-':
			case '0' ... '9':
			case '.':
				len ++;
				break;
			case '*':
				if (Current == ref_st)
					{
						parse(str + i - len, len, Current, Ret);
						len = 0;
						Current = chsum;
					}
				else
					Current = Error;
				break;
			case '\n':
			case '\r':
			case '\0':
				if (Current == chsum)
				{
					parse(str + i - len, len, Current, Ret);
					Current = Fin;
				}
				else
					Current = Error;
		}
		i ++;
	}
	return Current = Fin;
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
