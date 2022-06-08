#ifndef QMC5883L_QMC5883L_H_
#define QMC5883L_QMC5883L_H_

#include <stdint.h>

#define QMC5883L_ODR_10HZ	0b00
#define QMC5883L_ODR_50HZ	0b01
#define QMC5883L_ODR_100HZ	0b10
#define QMC5883L_ODR_200HZ	0b11

#define QMC5883L_RNG_2G		0b00
#define QMC5883L_RNG_8G		0b01

#define QMC5883L_OSR_512	0b00
#define QMC5883L_OSR_256	0b01
#define QMC5883L_OSR_128	0b10
#define QMC5883L_OSR_64		0b11

#define QMC5883L_MODE_CONT	0b01
#define QMC5883L_MODE_STBY	0b00

#define QMC5883L_OK			0
#define QMC5883L_ERROR 		1

#endif /* QMC5883L_QMC5883L_H_ */