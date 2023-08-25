/*
 * user_functions.h
 *
 *  Created on: Jul 20, 2023
 *      Author: bhupetur
 */

#ifndef USER_FUNCTIONS_H_
#define USER_FUNCTIONS_H_

typedef struct Wheel
{
uint32_t RPM;
uint32_t IC_Val_1;
uint32_t IC_Val_0;
} Wheel;

uint32_t Motor_Set(uint8_t);
uint32_t Battery_Meas(ADC_HandleTypeDef *hadc);
void Speed_Control(TIM_HandleTypeDef *htim, float base, Wheel *act_1, Wheel *act_2);
uint32_t Speed_Calc(Wheel *wheel);
void delay_us(uint16_t);

/* LCD HANDLER DEFINE */
#define LCD_CLEAR                 ((uint8_t)0x01)
#define LCD_RETURN                ((uint8_t)0x02)
#define LCD_NEWLINE               ((uint8_t)0xC0)
#define LCD_SHIFT_RIGHT           ((uint8_t)0x14)
#define LCD_SHIFT_LEFT            ((uint8_t)0x10)

void LCD_Init(void);
void LCD_Send_Char(uint8_t);
void LCD_Send_Variable(int,int);
void LCD_Send_String(char*);
void LCD_Send_CMD(uint8_t);



#endif /* USER_FUNCTIONS_H_ */
