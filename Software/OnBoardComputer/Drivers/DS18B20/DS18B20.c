/*
 * DS18B20.c
 *
 *  Created on: Jan 11, 2021
 *      Author: Sergei
 */

#include "DS18B20.h"
#include "DS18B20_macro.h"
#include "DS18B20/DS18B20_port.h"

void DS18_WriteByte(uint8_t dt);
uint8_t DS18_ReadByte();


uint8_t DS18_Init(uint64_t id)
{
	DS18_USER_SysInit();



	DS18_USER_GPIO_HIZ_IN();
	if(DS18_ResetPulse_sens_detect() == DS18_ERROR) return DS18_ERROR;

	if(id == DS18_SKIP_ROM){

		DS18_WriteByte(DS18_SKIP_ROM_CMD);
		DS18_USER_DelayMicroSec(1);
		DS18_WriteByte(DS18_W_SCRATCHPAD_CMD);
		DS18_USER_DelayMicroSec(1);
		DS18_WriteByte(0x64);
		DS18_USER_DelayMicroSec(1);
		DS18_WriteByte(0x9E);

		//Resolution 12 bit
		DS18_WriteByte(RESOLUTION_12BIT);
		DS18_USER_DelayMicroSec(100);
		return DS18_OK;
	}

  return DS18_ERROR;

}


uint8_t DS18_StartConv(uint64_t id){
	DS18_ResetPulse_sens_detect();

	if(id == DS18_SKIP_ROM)
	{
	//SKIP ROM
	DS18_WriteByte(DS18_SKIP_ROM_CMD);

	}else return DS18_ERROR;

	//CONVERT T
	DS18_WriteByte(DS18_T_CONV_CMD);
	return DS18_OK;
}

uint8_t DS18_GetData(uint64_t id, uint16_t* temp ){
	uint8_t Data[8];
	DS18_ResetPulse_sens_detect();
	if(id == DS18_SKIP_ROM)
	{
		//SKIP ROM
		DS18_WriteByte(DS18_SKIP_ROM_CMD);

	}else return DS18_ERROR;


	DS18_WriteByte(DS18_R_SCRATCHPAD_CMD);

	for(uint8_t i=0;i<8;i++)
	{
		Data[i] = DS18_ReadByte();
	}

	*temp = Data[1] << 8 | Data[0];
	return DS18_OK;
}
float DS18_TempConvert(uint16_t rawTemp)
{
	return (float)rawTemp/16.0f;
}

/************************************************************************************************************/
/************************************************************************************************************/
/************************************************************************************************************/
void DS18_WriteBit(uint8_t bit){
	DS18_USER_GPIO_ToGround();

	DS18_USER_DelayMicroSec(bit ? 3 : 65);

	DS18_USER_GPIO_HIZ_IN();

	DS18_USER_DelayMicroSec(bit ? 65 : 3);
}

void DS18_WriteByte(uint8_t dt){
	  for (uint8_t i = 0; i < 8; i++)
	  {
		DS18_WriteBit((dt >> i) & 1);
	    //Delay Protection
		DS18_USER_DelayMicroSec(5);
	  }
}
uint8_t DS18_ReadBit(){

	  DS18_USER_GPIO_ToGround();//???????????? ??????????????
	  DS18_USER_DelayMicroSec(2);
	  DS18_USER_GPIO_HIZ_IN();//?????????????? ??????????????
	  DS18_USER_DelayMicroSec(13);

	  return DS18_USER_GPIO_HIZ_IN();
}
uint8_t DS18_ReadByte(){
	  uint8_t data = 0;

	  for (uint8_t i = 0; i <= 7; i++)
	  {
		  data += DS18_ReadBit() << i;
		  DS18_USER_DelayMicroSec(45);
	  }
	  return data;
}



uint8_t DS18_ResetPulse_sens_detect(){
	uint8_t answer = 1;
	DS18_USER_GPIO_HIZ_IN();

	DS18_USER_GPIO_ToGround();
	DS18_USER_DelayMicroSec(480);
	DS18_USER_GPIO_HIZ_IN();

	DS18_USER_DelayMicroSec(65);
	answer = DS18_USER_GPIO_HIZ_IN();
	DS18_USER_DelayMicroSec(240);

	return answer ? DS18_ERROR : DS18_OK;
}
