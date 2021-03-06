/*
 * DS18B20_macro.h
 *
 *  Created on: Jan 11, 2021
 *      Author: Sergei
 */

#ifndef INC_DS18B20_MACRO_H_
#define INC_DS18B20_MACRO_H_

#define RESOLUTION_9BIT 0x1F
#define RESOLUTION_10BIT 0x3F
#define RESOLUTION_11BIT 0x5F
#define RESOLUTION_12BIT 0x7F

#define DS18_T_CONV_CMD			0x44
#define DS18_SKIP_ROM_CMD		0xCC
#define DS18_W_SCRATCHPAD_CMD	0x4E
#define DS18_R_SCRATCHPAD_CMD	0xBE

#endif /* INC_DS18B20_MACRO_H_ */
