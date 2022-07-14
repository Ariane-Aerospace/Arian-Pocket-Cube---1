#ifndef UCAM3_UCAM3_H_
#define UCAM3_UCAM3_H_

#include <stdint.h>
#include "main.h"

#define UCAM3_OK							0x00
#define UCAM3_ERROR							0x01

#define UCAM3_ACKh							0xAA
#define UCAM3_ACKl							0x0E

#define UCAM3_ImageFormat_8bit				0x03
#define UCAM3_ImageFormat_16bit_CrYCbY		0x03
#define UCAM3_ImageFormat_16bit_RGB			0x03
#define UCAM3_ImageFormat_JPEG				0x07

#define UCAM3_RAWResolution_80x60			0x01
#define UCAM3_RAWResolution_160x120			0x03
#define UCAM3_RAWResolution_128x128			0x09
#define UCAM3_RAWResolution_128x96			0x0B
#define UCAM3_RAWResolution_JPEG			0x00

#define UCAM3_JPEGResolution_160x128		0x03
#define UCAM3_JPEGResolution_320x240		0x05
#define UCAM3_JPEGResolution_640x480		0x07
#define UCAM3_JPEGResolution_RAW			0x00

#define UCAM3_PictureType_Snapshot			0x01
#define UCAM3_PictureType_RAW				0x02
#define UCAM3_PictureType_JPEG				0x05

#define UCAM3_HardReset						0x00
#define UCAM3_Reset							0x01

void ResetCam();
uint8_t UCAM3_InitPort();
uint8_t UCAM3_Init();
uint8_t UCAM3_Sync();
uint8_t UCAM3_GetPucture(uint16_t n);


#endif /* UCAM3_UCAM3_H_ */
