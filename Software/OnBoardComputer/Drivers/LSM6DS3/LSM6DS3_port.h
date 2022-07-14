#ifndef LSM6DS3_PORT_H_
#define LSM6DS3_PORT_H_

#include "stm32f4xx_hal.h"
#include "main.h"

void 	LSM6DS3_USER_CS_Activate();
void 	LSM6DS3_USER_CS_Deactivate();
uint8_t LSM6DS3_USER_SPI_RxTx(uint8_t txData);


#endif
