/*
 * PCF8574_LCD.h
 *
 *  Created on: Mar 24, 2022
 *      Author: alexbobkov
 */

#ifndef INC_PCF8574_LCD_H_
#define INC_PCF8574_LCD_H_

#include "stm32f4xx_hal.h"

#define LCD_2004_ADDRESS 0x4E


void LCD_Init ( I2C_HandleTypeDef ); 					// init
void LCD_Send_Cmd ( char, I2C_HandleTypeDef );   		// send command
void LCD_Send_Data ( char, I2C_HandleTypeDef ); 			// send data
void LCD_Send_String ( char *, I2C_HandleTypeDef );		// send string
void LCD_Clear ( I2C_HandleTypeDef);						// cler screen
void LCD_Put_Cur ( int row, int col, I2C_HandleTypeDef );	// cursor position


#endif /* INC_PCF8574_LCD_H_ */
