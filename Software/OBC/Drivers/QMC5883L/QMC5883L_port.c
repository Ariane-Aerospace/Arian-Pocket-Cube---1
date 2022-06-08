#include "QMC5883L_port.h"
#include "QMC5883L_macro.h"
extern I2C_HandleTypeDef hi2c1;


void _QMC5883L_Reg_Write(uint8_t Addr, uint8_t Data)
{
	HAL_I2C_Mem_Write(&hi2c1, QMC5883L_I2C_ADDRESS, Addr, 1, &Data, 1, 1000);
}

uint8_t _QMC5883L_Reg_Read(uint8_t Addr)
{
	uint8_t Data;

	HAL_I2C_Mem_Read(&hi2c1, QMC5883L_I2C_ADDRESS, Addr, 1, &Data, 1, 1000);

	return Data;
}

void _QMC5883L_Delay(uint32_t ms)
{
	HAL_Delay(ms);
}
