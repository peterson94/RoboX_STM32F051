/*
 * speed_calc.c
 *
 *  Created on: Jul 20, 2023
 *      Author: bhupetur
 */


#include "main.h"
#include "user_functions.h"

uint32_t Speed_Calc(Wheel *wheel)
{
	if( (wheel->IC_Val_1) > (wheel->IC_Val_0) )
		return ( 3000 / ( (wheel->IC_Val_1) - (wheel->IC_Val_0) ) );
	else return wheel->RPM;
}
