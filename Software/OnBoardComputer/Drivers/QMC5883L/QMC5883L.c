#include "QMC5883L.h"
#include "QMC5883L_port.h"
#include "QMC5883L_macro.h"


uint8_t QMC5883L_Init(uint8_t odr, uint8_t rng, uint8_t osr, uint8_t mode)
{

	/*_QMC5883L_Reg_Write(QMC5883L_CTRL1, 0xAA);
	uint8_t data = _QMC5883L_Reg_Read(QMC5883L_CTRL1);

	if(data != 0xAA) return QMC5883L_ERROR;

	_QMC5883L_Reg_Write(QMC5883L_CTRL2, 1 << 7);
	_QMC5883L_Delay(5);*/

	_QMC5883L_Reg_Write(QMC5883L_CTRL1, osr << 6 | rng << 4 | odr << 2 |  mode << 0);
	_QMC5883L_Reg_Write(QMC5883L_PSRR, 0x01);

	return QMC5883L_OK;
}


void QMC5883L_Read_Mag_Data(int16_t *MagX, int16_t *MagY, int16_t *MagZ)
{
	*MagX=((int16_t)_QMC5883L_Reg_Read(QMC5883L_DATA_READ_X_LSB) | (((int16_t)_QMC5883L_Reg_Read(QMC5883L_DATA_READ_X_MSB))<<8));
	*MagY=((int16_t)_QMC5883L_Reg_Read(QMC5883L_DATA_READ_Y_LSB) | (((int16_t)_QMC5883L_Reg_Read(QMC5883L_DATA_READ_Y_MSB))<<8));
	*MagZ=((int16_t)_QMC5883L_Reg_Read(QMC5883L_DATA_READ_Z_LSB) | (((int16_t)_QMC5883L_Reg_Read(QMC5883L_DATA_READ_Z_MSB))<<8));
}
