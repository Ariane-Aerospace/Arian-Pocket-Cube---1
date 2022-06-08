
#ifndef MS_RADIO_MSR_H_
#define MS_RADIO_MSR_H_

#include <stdint.h>

void MSR_Init();
void MSR_CS_Prepare();
void MSR_SendData(uint8_t* data);
void MSR_GetData(uint8_t* data);
void MSR_GetGPS();

#endif /* MS_RADIO_MSR_H_ */
