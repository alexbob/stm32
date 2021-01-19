#include "stm32f4xx_hal.h"

void LCD_Init (void);   // initialize lcd

/*
 * 	Send CMD to LCD (rs = 0)
 */
void LCD_Send_Cmd (char cmd);  // send command to the lcd

/*
 * sends bute of data to LCD
 *
 */
void LCD_Send_Data (char data);  // send data to the lcd

/*
 *  sends line to LCD
 *  	str - line to print (line will be cut if not fit)
 *  	line - display line # for string, wrapping if line > than 3
 *  	position - position from left to right starting from 0
 *
 */
void LCD_Send_String (char *str, uint8_t line, uint8_t position);


void LCD_Clear (void);
