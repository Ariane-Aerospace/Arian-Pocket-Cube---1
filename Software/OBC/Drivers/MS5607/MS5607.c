#include "MS5607.h"




extern I2C_HandleTypeDef hi2c1;

uint8_t MS5607_Init()
{
	uint8_t txData =  0x1E ;
	HAL_I2C_Master_Transmit(&hi2c1, MS5607_DEV_ADDR_7B, &txData, 1, 100); //Reset CMD

	txData =  0x48;
	HAL_I2C_Master_Transmit(&hi2c1, MS5607_DEV_ADDR_7B, &txData, 1, 100); //Start temp conv


	return MS5607_OK;

}

void MS5607_GetData(uint32_t* temp, uint32_t* pres)
{

	uint8_t txData =  0x58;
	HAL_I2C_Master_Transmit(&hi2c1, MS5607_DEV_ADDR_7B, &txData, 1, 100); //Start temp conv

	HAL_Delay(10);

	txData =  0x00;
	HAL_I2C_Master_Transmit(&hi2c1, MS5607_DEV_ADDR_7B, &txData, 1, 100);

	uint8_t rxData[3];
	HAL_I2C_Master_Receive(&hi2c1, MS5607_DEV_ADDR_7B, rxData, 3, 100);
	*temp = rxData[0]<<16 | rxData[1]<<8 |  rxData[2];


	txData =  0x48;
	HAL_I2C_Master_Transmit(&hi2c1, MS5607_DEV_ADDR_7B, &txData, 1, 100); //Start temp conv

	HAL_Delay(10);

	txData =  0x00;
	HAL_I2C_Master_Transmit(&hi2c1, MS5607_DEV_ADDR_7B, &txData, 1, 100);

	HAL_I2C_Master_Receive(&hi2c1, MS5607_DEV_ADDR_7B, rxData, 3, 100);
	*pres = rxData[0]<<16 | rxData[1]<<8 |  rxData[2];



}
void MS5607_GetPres(uint32_t* pres)
{

	uint8_t txData =  0x00;
	uint8_t rxData[3];
	HAL_I2C_Master_Transmit(&hi2c1, MS5607_DEV_ADDR_7B, &txData, 1, 100);

	HAL_I2C_Master_Receive(&hi2c1, MS5607_DEV_ADDR_7B, rxData, 3, 100);
	*pres = rxData[0]<<16 | rxData[1]<<8 |  rxData[2];

	txData =  0x48;
	HAL_I2C_Master_Transmit(&hi2c1, MS5607_DEV_ADDR_7B, &txData, 1, 100); //Start pres conv

}
