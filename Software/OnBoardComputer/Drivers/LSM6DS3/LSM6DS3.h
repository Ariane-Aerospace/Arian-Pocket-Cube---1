#ifndef LSM6DS3_H_
#define LSM6DS3_H_

#include "stm32f4xx_hal.h"
#include "LSM6DS3_port.h"
#include "LSM6DS3_macro.h"


void 		LSM6DS3_CS_Prepare();
uint8_t 	LSM6DS3_Init();
void 		LSM6DS3_GetData(int16_t* AccelData, int16_t* GyroData);
void 		LSM6DS3_AccelConvert(int16_t* pAccelRaw, float* pAccelG);
void 		LSM6DS3_GyroConvert(int16_t* pGyroRaw, float* pGyroDS);
#endif
