#include "LSM6DS3.h"
#include <math.h>

uint8_t 	LSM6DS3_ReadRegister(uint8_t regAddr);
void 		LSM6DS3_WriteRegister(uint8_t regAddr, uint8_t data);

void LSM6DS3_CS_Prepare()
{
	LSM6DS3_USER_CS_Deactivate();
}
uint8_t LSM6DS3_Init()
{
	LSM6DS3_USER_CS_Activate();
	uint8_t ID = LSM6DS3_ReadRegister(LSM6DS3_REG_ID);
	LSM6DS3_USER_CS_Deactivate();
	if((ID != 0x6A))
	{
		return LSM6DS3_ERROR;
	}

	LSM6DS3_USER_CS_Activate();
	LSM6DS3_WriteRegister(0x10, (1<<7)|(1<<2));
	LSM6DS3_USER_CS_Deactivate();

	LSM6DS3_USER_CS_Activate();
	LSM6DS3_WriteRegister(0x11, (1<<7)|(1<<2)|(1<<3));
	LSM6DS3_USER_CS_Deactivate();

	return LSM6DS3_OK;
}

void LSM6DS3_GetData(int16_t* AccelData, int16_t* GyroData)
{


	uint8_t tmpData[12];

	//Start ACCEL read
	LSM6DS3_USER_CS_Activate();
	tmpData[0] = LSM6DS3_ReadRegister(0x28);
	LSM6DS3_USER_CS_Deactivate();

	LSM6DS3_USER_CS_Activate();
	tmpData[1] = LSM6DS3_ReadRegister(0x29);
	LSM6DS3_USER_CS_Deactivate();

	LSM6DS3_USER_CS_Activate();
	tmpData[2] = LSM6DS3_ReadRegister(0x2A);
	LSM6DS3_USER_CS_Deactivate();

	LSM6DS3_USER_CS_Activate();
	tmpData[3] = LSM6DS3_ReadRegister(0x2B);
	LSM6DS3_USER_CS_Deactivate();

	LSM6DS3_USER_CS_Activate();
	tmpData[4] = LSM6DS3_ReadRegister(0x2C);
	LSM6DS3_USER_CS_Deactivate();

	LSM6DS3_USER_CS_Activate();
	tmpData[5] = LSM6DS3_ReadRegister(0x2D);
	LSM6DS3_USER_CS_Deactivate();


	//Start GYRO read
	LSM6DS3_USER_CS_Activate();
	tmpData[6] = LSM6DS3_ReadRegister(0x22);
	LSM6DS3_USER_CS_Deactivate();

	LSM6DS3_USER_CS_Activate();
	tmpData[7] = LSM6DS3_ReadRegister(0x23);
	LSM6DS3_USER_CS_Deactivate();

	LSM6DS3_USER_CS_Activate();
	tmpData[8] = LSM6DS3_ReadRegister(0x24);
	LSM6DS3_USER_CS_Deactivate();

	LSM6DS3_USER_CS_Activate();
	tmpData[9] = LSM6DS3_ReadRegister(0x25);
	LSM6DS3_USER_CS_Deactivate();

	LSM6DS3_USER_CS_Activate();
	tmpData[10] = LSM6DS3_ReadRegister(0x26);
	LSM6DS3_USER_CS_Deactivate();

	LSM6DS3_USER_CS_Activate();
	tmpData[11] = LSM6DS3_ReadRegister(0x27);
	LSM6DS3_USER_CS_Deactivate();


	//Write data
	AccelData[0] = (tmpData[1] << 8) | tmpData[0];	//ACCEL X
	AccelData[1] = (tmpData[3] << 8) | tmpData[2];	//ACCEL Y
	AccelData[2] = (tmpData[5] << 8) | tmpData[4];	//ACCEL Z

	GyroData[0] = (tmpData[7] << 8) | tmpData[6];	//GYRO X
	GyroData[1] = (tmpData[9] << 8) | tmpData[8];	//GYRO Y
	GyroData[2] = (tmpData[11] << 8) | tmpData[10];//GYRO Z

}

void LSM6DS3_AccelConvert(int16_t* pAccelRaw, float* pAccelG)
{
	pAccelG[0] = pAccelRaw[0] * 0.488f / 1000.0f;
	pAccelG[1] = pAccelRaw[1] * 0.488f / 1000.0f;
	pAccelG[2] = pAccelRaw[2] * 0.488f / 1000.0f;
	pAccelG[3] = sqrt( (pAccelG[0]*pAccelG[0] ) + (pAccelG[1]*pAccelG[1]) + (pAccelG[2]*pAccelG[2]) );
}
void LSM6DS3_GyroConvert(int16_t* pGyroRaw, float* pGyroDS)
{
	pGyroDS[0] = pGyroRaw[0] * 17.5f / 1000.0f;
	pGyroDS[1] = pGyroRaw[1] * 17.5f / 1000.0f;
	pGyroDS[2] = pGyroRaw[2] * 17.5f / 1000.0f;
	pGyroDS[3] = sqrt( (pGyroDS[0]*pGyroDS[0]) + (pGyroDS[1]*pGyroDS[1]) + (pGyroDS[2]*pGyroDS[2]) );
}







uint8_t LSM6DS3_ReadRegister(uint8_t regAddr){

	LSM6DS3_USER_SPI_RxTx(regAddr | (1<<7));
	return LSM6DS3_USER_SPI_RxTx(0xFF);
}
void LSM6DS3_WriteRegister(uint8_t regAddr, uint8_t data){
	LSM6DS3_USER_SPI_RxTx(regAddr & ~(1<<7));
	LSM6DS3_USER_SPI_RxTx(data);
}
