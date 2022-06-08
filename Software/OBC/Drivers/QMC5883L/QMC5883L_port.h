#ifndef QMC5883L_QMC5883L_PORT_H_
#define QMC5883L_QMC5883L_PORT_H_

#include <stdint.h>
#include "main.h"

void _QMC5883L_Reg_Write(uint8_t Addr, uint8_t Data);

uint8_t _QMC5883L_Reg_Read(uint8_t Addr);

#endif /* QMC5883L_QMC5883L_PORT_H_ */
