#include "MSR_Port.h"
#include "stdint.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;


void __MSR_USER_CS_Low()
{
	HAL_GPIO_WritePin(CS_RF_GPIO_Port, CS_RF_Pin, RESET);
}
void __MSR_USER_CS_High()
{
	HAL_GPIO_WritePin(CS_RF_GPIO_Port, CS_RF_Pin, SET);
}
void __MSR_USER_SPI_TxRx(uint8_t* TXdata, uint8_t* RXdata, uint8_t lth)
{
	HAL_SPI_TransmitReceive(&hspi1, TXdata, RXdata, lth, 1000);
}
