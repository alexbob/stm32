/*
 * ili9341.c
 *
 *  Created on: Jan 7, 2021
 *      Author: Alex
 */
// -------


#include "ili9341.h"

uint16_t 	X_SIZE = 0;
uint16_t 	Y_SIZE = 0;
uint32_t	dtt = 0 ;


// _-------
void TFT9341_Delay(uint32_t dly)
{

	for( uint32_t i = 0; i < dly; i++ );
}

static inline void DelayMicro(uint32_t volatile  micros)
{
	micros *= (SystemCoreClock/1000000)/8;
	while( micros--);

}

void TFT9341_SendCommand( uint8_t cmd)
{
	ADDR_CMD = cmd;
//	*(volatile uint8_t *) (0x60000000) = cmd;
}

void TFT9341_SendData( uint8_t dta)
{
//	*(volatile uint8_t *) (0x60000000) = dta;

	ADDR_DATA = dta;
	TFT9341_Delay(1);
//	DelayMicro(50);
}

uint32_t TFT9341_ReadReg( uint8_t reg)
{
	uint32_t 	reg_data;
	uint8_t 	x;

	TFT9341_SendCommand(reg);

	// read first byte
	HAL_Delay(50);
	x = ADDR_DATA;
//	x = *(volatile uint8_t *) (0x60010000);

	reg_data = x;
	reg_data <<= 8;

	// second byte
//	DelayMicro(1);
	HAL_Delay(2);

	x = ADDR_DATA;
//	x = *(volatile uint8_t *) (0x60010000);

	reg_data |= x;
	reg_data <<= 8;

	// third byte

//	DelayMicro(1);
	HAL_Delay(2);

	x = ADDR_DATA;
//	x = *(volatile uint8_t *) (0x60010000);

	reg_data |= x;
	reg_data <<= 8;

	// forth byte

	HAL_Delay(2);
//	DelayMicro(1);
	x = ADDR_DATA;
//	x = *(volatile uint8_t *) (0x60010000);

	reg_data |= x;

	// reg = 0xEF additional 2 bytes ?
	if( reg == 0xEF) {
		reg_data <<= 8;
		DelayMicro(5);
		x = ADDR_DATA;
		reg_data |= x;
	}

//	DelayMicro(150);
	HAL_Delay(100);
	return reg_data;
}

void TFT9341_SetRotation( uint8_t r)
{
	TFT9341_SendCommand(0x36);
	switch(r) {
		case 0:
			TFT9341_SendData(0x48);
			X_SIZE = 240;
			Y_SIZE = 320;
			break;
		case 1:
			TFT9341_SendData(0x28);
			X_SIZE = 320;
			Y_SIZE = 240;
			break;
		case 2:
			TFT9341_SendData(0x88);
			X_SIZE = 240;
			Y_SIZE = 320;
			break;
		case 3:
			TFT9341_SendData(0xE8);
			X_SIZE = 320;
			Y_SIZE = 240;
			break;

	}
}

void TFT9341_Reset( void )
{
	RESET_ACTIVE;
	HAL_Delay(100);
	RESET_IDLE;

	TFT9341_SendCommand(0x01);

//	for ( uint8_t i = 0; i < 3; i++) TFT9341_SendCommand(0xFF);

}

void TFT9341_Init( void )
{

	char str[15];

	TFT9341_Reset();
	HAL_Delay(1000);
	dtt = TFT9341_ReadReg(TFT9341_CMD_READ_ID); // read display model number
	LCD_Clear();

	sprintf( str, "0x%08lX", (unsigned long) dtt);
	LCD_Send_String(str, 0, 0);


	TFT9341_SendCommand(0x01);
	DelayMicro(1);

}


