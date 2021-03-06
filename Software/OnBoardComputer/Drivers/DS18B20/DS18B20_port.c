
#include "DS18B20_port.h"
#include "DS18B20_macro.h"

extern TIM_HandleTypeDef htim11;

uint8_t DS18_USER_GPIO_HIZ_IN() {
	HAL_GPIO_WritePin(DS18_DQ_GPIO_Port, DS18_DQ_Pin, SET);
	return (HAL_GPIO_ReadPin(DS18_DQ_GPIO_Port, DS18_DQ_Pin));
}
void DS18_USER_GPIO_ToGround() {
	HAL_GPIO_WritePin(DS18_DQ_GPIO_Port, DS18_DQ_Pin, RESET);
}
void DS18_USER_DelayMicroSec(uint32_t micros) {
	__HAL_TIM_SET_COUNTER(&htim11, 0);
	while(__HAL_TIM_GET_COUNTER(&htim11) < micros);
}
void DS18_USER_SysInit() {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	HAL_GPIO_WritePin(DS18_DQ_GPIO_Port, DS18_DQ_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = DS18_DQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DS18_DQ_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_Init(DS18_DQ_GPIO_Port, &GPIO_InitStruct);

	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 84-1;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 65535;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(&htim11);

	HAL_TIM_Base_Start(&htim11);

}
