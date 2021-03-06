#include "LSM6DS3_port.h"
extern SPI_HandleTypeDef hspi1;

void LSM6DS3_USER_CS_Activate()
{
	HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_RESET);
}

void LSM6DS3_USER_CS_Deactivate()
{
	HAL_GPIO_WritePin(LSM_CS_GPIO_Port, LSM_CS_Pin, GPIO_PIN_SET);
}
uint8_t LSM6DS3_USER_SPI_RxTx(uint8_t txData)
{
	uint8_t rxData = 0xFF;
	HAL_SPI_TransmitReceive(&hspi1, &txData, &rxData, 1, 2000);
	return rxData;
}
