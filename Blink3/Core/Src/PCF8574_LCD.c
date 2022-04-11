/*
 * PCF8574_LCD.c
 *
 * LCD Driver
 *
 *
 *  Created on: Mar 24, 2022
 *      Author: alexbobkov
 */
#include "PCF8574_LCD.h"
#include "stm32f4xx_hal.h"

// Init LCD controller
void LCD_Init (I2C_HandleTypeDef i2c)
{
	HAL_Delay(1000);
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	LCD_Send_Cmd (0x30, i2c);
	HAL_Delay(5);  // wait for >4.1ms
	LCD_Send_Cmd (0x30, i2c);
	HAL_Delay(1);  // wait for >100us
	LCD_Send_Cmd (0x30, i2c);
	HAL_Delay(10);
	LCD_Send_Cmd (0x20, i2c);  // 4bit mode
	HAL_Delay(10);

  // display initialization
	LCD_Send_Cmd (0x28, i2c); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	LCD_Send_Cmd (0x08, i2c); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	LCD_Send_Cmd (0x01, i2c);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	LCD_Send_Cmd (0x06, i2c); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	LCD_Send_Cmd (0x0C, i2c); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

// Send command

void LCD_Send_Cmd (char cmd, I2C_HandleTypeDef i2c) {

	char data_u, data_l;
//	HAL_StatusTypeDef ret;
	uint8_t data_t[4];


	data_u = ( cmd & 0xf0 );
	data_l = (( cmd << 4 ) & 0xf0 );
	data_t[0] = data_u | 0x0C;  //en=1, rs=0
	data_t[1] = data_u | 0x08;  //en=0, rs=0
	data_t[2] = data_l | 0x0C;  //en=1, rs=0
	data_t[3] = data_l | 0x08;  //en=0, rs=0

	HAL_I2C_Master_Transmit( &i2c, LCD_2004_ADDRESS, (uint8_t*) data_t, 4, 100);
}

// Send Data
void LCD_Send_Data (char data, I2C_HandleTypeDef i2c)
{
	char data_u, data_l;
//	HAL_StatusTypeDef ret;
	uint8_t data_t[4];

	data_u = ( data & 0xf0 );
	data_l = (( data << 4 ) & 0xf0);
	data_t[0] = data_u | 0x0D;  //en=1, rs=1
	data_t[1] = data_u | 0x09;  //en=0, rs=1
	data_t[2] = data_l | 0x0D;  //en=1, rs=1
	data_t[3] = data_l | 0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&i2c, LCD_2004_ADDRESS, (uint8_t *) data_t, 4, 100);
}

// send string
void LCD_Send_String (char *str, I2C_HandleTypeDef i2c)
{
	while (*str) LCD_Send_Data (*str++, i2c);
}

// clear screen
void LCD_Clear (I2C_HandleTypeDef i2c)
{
	LCD_Send_Cmd (0x80, i2c);
	for (int i=0; i<70; i++)
	{
		LCD_Send_Data (' ', i2c);
	}
}

// setc cursor
void LCD_Put_Cur ( int row, int col, I2C_HandleTypeDef i2c)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    LCD_Send_Cmd (col, i2c);
}


