#ifndef UCAM3_UCAM3_PORT_H_
#define UCAM3_UCAM3_PORT_H_

#include <stdint.h>

void UCAM3_Transmit(uint8_t* Data, uint8_t Length);
uint8_t UCAM3_Receive(uint8_t* Data, uint8_t Length, uint32_t TimeOut);
void UCAM3_Delay(uint16_t Ms);

#endif /* UCAM3_UCAM3_PORT_H_ */
