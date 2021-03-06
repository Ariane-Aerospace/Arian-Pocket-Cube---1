/*
 * DS18B20.h
 *
 *  Created on: Jan 11, 2021
 *      Author: Sergei
 */

#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include <stdint.h>

#define DS18_OK 0
#define DS18_SKIP_ROM 		0
#define DS18_ERROR			1

uint8_t DS18_ResetPulse_sens_detect();
uint8_t DS18_Init(uint64_t id);
uint8_t DS18_StartConv(uint64_t id);
uint8_t DS18_GetData(uint64_t id, uint16_t* temp);
float   DS18_TempConvert(uint16_t rawTemp);

#endif /* INC_DS18B20_H_ */
