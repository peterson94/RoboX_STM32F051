/*
 * speed_control.c
 *
 *  Created on: Jul 20, 2023
 *      Author: bhupetur
 */


#include "main.h"
#include "user_functions.h"

void Speed_Control(TIM_HandleTypeDef *htim, float base, Wheel *act_1, Wheel *act_2)
{

	static float err_1 = 0.0f;
	static float err_1_sum = 0.0f;
	static float kp_1 = 0.3f;
	static float ki_1 = 0.0002f;

	static float err_2 = 0.0f;
	static float err_2_sum = 0.0f;
	static float kp_2 = 0.3f;
	static float ki_2 = 0.0002f;

	float ACT_1 = 0.0f;
	float ACT_2 = 0.0f;

	if (base)
	{
		err_1 = 20000 * (base - act_1->RPM)/base;
		err_2 = 20000 * (base - act_2->RPM)/base;

		ACT_1 = kp_1*err_1 + ki_1*err_1_sum;
		ACT_2 = kp_2*err_2 + ki_2*err_2_sum;

		if(ACT_1 > 0)
			htim->Instance->CCR1 = (uint32_t)ACT_1;
		else htim->Instance->CCR1 = 0x0;

		if(ACT_2 > 0)
			htim->Instance->CCR2 = (uint32_t)ACT_2;
		else htim->Instance->CCR2 = 0x0;

		//htim->Instance->CCR1 = 2500;
		//htim->Instance->CCR2 = 2500;
		err_1_sum += err_1;
		err_2_sum += err_2;
	}

	else
	{
		err_1 = 0;
		err_2 = 0;

		err_1_sum = 0;
		err_2_sum = 0;

		htim->Instance->CCR1 = 4000-1;
		htim->Instance->CCR2 = 4000-1;
	}
}
