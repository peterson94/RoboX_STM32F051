/*
 * battery_meas.c
 *
 *  Created on: Jul 20, 2023
 *      Author: bhupetur
 */


#include "main.h"
#include "user_functions.h"

uint32_t Battery_Meas(ADC_HandleTypeDef* hadc)
{
	uint32_t BAT;

	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 10);
	BAT = HAL_ADC_GetValue(hadc);

	BAT = BAT * 8.057 * 2.47 / 1000;

	return BAT;
}
