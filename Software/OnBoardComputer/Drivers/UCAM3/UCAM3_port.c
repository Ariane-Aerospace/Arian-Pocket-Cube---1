#include "UCAM3_port.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"


extern UART_HandleTypeDef huart2;

void UCAM3_Transmit(uint8_t* Data, uint8_t Length)
{
	HAL_UART_Transmit(&huart2, Data, Length, 10000);
}

void UCAM3_Delay(uint16_t Ms)
{
	HAL_Delay(Ms);
}

uint8_t UCAM3_Receive(uint8_t* Data, uint8_t Length, uint32_t TimeOut)
{
	return HAL_UART_Receive(&huart2, Data, Length, TimeOut);
}
