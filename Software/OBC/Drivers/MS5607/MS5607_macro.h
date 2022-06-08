/*
 * MS5607_macro.h
 *
 *  Created on: 1 февр. 2021 г.
 *      Author: Sergei
 */

#ifndef MS5607_MS5607_MACRO_H_
#define MS5607_MS5607_MACRO_H_

#define MS5607_REG_ID_ADDR
#define MS5607_DEFAULT_ID


#define MS5607_CSB_LOGIC_LVL 0
#define MS5607_DEV_ADDR_BASE 0b111011

#define MS5607_DEV_ADDR_7B (MS5607_DEV_ADDR_BASE<<2 | !MS5607_CSB_LOGIC_LVL<<1)


#define MS5607_OK 0
#define MS5607_ERROR 1

#endif /* MS5607_MS5607_MACRO_H_ */
