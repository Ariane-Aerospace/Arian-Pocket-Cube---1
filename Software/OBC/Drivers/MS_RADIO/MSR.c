#include "MSR_Port.h"
#include "MSR.h"

void MSR_Init()
{
 //This system is`t need a initialization
}
void MSR_CS_Prepare()
{
	__MSR_USER_CS_High();
}
void MSR_SendData(uint8_t* data)
{
	uint8_t none[32];
	uint8_t cmd = 0xAA;

	__MSR_USER_CS_Low();
	__MSR_USER_SPI_TxRx(&cmd, none, 1);
	__MSR_USER_SPI_TxRx(data, none, 32);
	__MSR_USER_CS_High();
}
void MSR_GetData(uint8_t* data)
{

}
void MSR_GetGPS()
{

}
