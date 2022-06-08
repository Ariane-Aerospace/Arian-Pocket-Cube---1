/*
 * MSR_Port.h
 *
 *  Created on: Jan 3, 2022
 *      Author: Sergey
 */

#ifndef MS_RADIO_MSR_PORT_H_
#define MS_RADIO_MSR_PORT_H_

#include <stdint.h>

void __MSR_USER_CS_Low();
void __MSR_USER_CS_High();
void __MSR_USER_SPI_TxRx(uint8_t* TXdata, uint8_t* RXdata, uint8_t lth);

#endif /* MS_RADIO_MSR_PORT_H_ */
