#include "QMC5883L.h"
#include "QMC5883L_port.h"
#include "QMC5883L_macro.h"


uint8_t QMC5883L_Init(uint8_t odr, uint8_t rng, uint8_t osr, uint8_t mode)
{
	/*_QMC5883L_Reg_Write(QMC5883L_CTRL2, 1 << 7);
	_QMC5883L_Delay(5);

	_QMC5883L_Reg_Write(QMC5883L_CTRL1, 0xAA);
	uint8_t Data = _QMC5883L_Reg_Read(QMC5883L_CTRL1);

	if(data != 0xAA) return QMC5883L_ERROR;

	_QMC5883L_Reg_Write(QMC5883L_CTRL1, mode << 0 || odr << 2 || rng << 4 || osr << 6);
	_QMC5883L_Reg_Write(QMC5883L_CTRL2, mode << 0 || odr << 2 || rng << 4 || osr << 6);
*/
	return QMC5883L_OK;
}
