/*
 * lcd_init.c
 *
 *  Created on: Jul 20, 2023
 *      Author: bhupetur
 */


#include "main.h"
#include "user_functions.h"
#include <math.h>

void LCD_Init(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

	// 8 bit initialization
	HAL_Delay(50);  // wait for >40ms
	LCD_Send_CMD (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	LCD_Send_CMD (0x30);
	HAL_Delay(1);  // wait for >100us
	LCD_Send_CMD (0x30);
	HAL_Delay(10);
	LCD_Send_CMD (0x30);  // 8bit mode
	HAL_Delay(10);

  // display initialization
	LCD_Send_CMD (0x38); // Function set --> DL=1 (8 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	LCD_Send_CMD (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	LCD_Send_CMD (0x01);  // clear display
	HAL_Delay(1);
	LCD_Send_CMD (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	LCD_Send_CMD (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void __LCD_Check_BSF()
{
	// SET GPIO_PIN_7 (D7) to input mode
	GPIOC->MODER &= ~(0x00000003 << 14);
	// RS=0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	// RW=1
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

	do
	{
		// EN=1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_Delay(1);
		// EN=0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_Delay(1);

	} while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7));

	// RW=0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	// SET GPIO_PIN_7 (D7) to output mode
	GPIOC->MODER &= ~(0x00000002 << 14);
}

void _LCD_Send(uint8_t BUFF, GPIO_PinState RS)
{
	uint32_t lcd_data;

	// Check busy-flag
	//__LCD_Check_BSF();

	//if RS==0 -> cmd register is selected, if RS==1-> data register is selected
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RS);

	//set enable pin high before sending data
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

	lcd_data = GPIOC->ODR & ~(0xFF);
	lcd_data |= BUFF;

	//set data pins
	GPIOC->ODR = lcd_data;

	//set enable from high to low, send data out
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	delay_us(50);
	//HAL_Delay(1);

}

void LCD_Send_Variable(int var, int ndigit)
{
	int i;

	if (var == 0)
	{
		_LCD_Send(0x30, GPIO_PIN_SET);
	}

	else
	{
		if (var < 0)
		{
			_LCD_Send(0x2D, GPIO_PIN_SET);
			var = -1 *var ;
		}

		for (i=pow(10,ndigit-1); i>0; i=i/10)
			_LCD_Send(0x30+(var/i)%10, GPIO_PIN_SET);
	}
}

void LCD_Send_String (char *str)
{
	while (*str) _LCD_Send(*str++, GPIO_PIN_SET);
}

void LCD_Send_CMD(uint8_t command)
{
	_LCD_Send(command, GPIO_PIN_RESET);
}

void LCD_Send_Char(uint8_t command)
{
	_LCD_Send(command, GPIO_PIN_SET);
}
