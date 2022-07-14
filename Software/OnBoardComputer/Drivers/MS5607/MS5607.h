/*
 * MS5607.h
 *
 *  Created on: 1 февр. 2021 г.
 *      Author: Sergei
 */

#ifndef MS5607_MS5607_H_
#define MS5607_MS5607_H_

#include "MS5607_port.h"
#include "MS5607_macro.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
uint8_t MS5607_Init();
void MS5607_GetPres(uint32_t* pres);
void MS5607_GetData(uint32_t* temp, uint32_t* pres);


#endif /* MS5607_MS5607_H_ */
