
/** Put this in the src folder **/

#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

void LCD_Send_Cmd ( char cmd )
{
  char data_u, data_l;

	uint8_t data_t[4];
	data_u = ( cmd & 0xf0 );
	data_l = ( ( cmd<<4)&0xf0 );
	data_t[0] = data_u | 0x0C;  //en=1, rs=0
	data_t[1] = data_u | 0x08;  //en=0, rs=0
	data_t[2] = data_l | 0x0C;  //en=1, rs=0
	data_t[3] = data_l | 0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit ( &hi2c1, SLAVE_ADDRESS_LCD, ( uint8_t * ) data_t, 4, 100 );
}

void LCD_Send_Data ( char data )
{
	char data_u, data_l;
	uint8_t data_t[4];

	data_u = ( data & 0xf0 );
	data_l = ( ( data << 4 ) & 0xf0 );
	data_t[0] = data_u | 0x0D;  //en=1, rs=1
	data_t[1] = data_u | 0x09;  //en=0, rs=1
	data_t[2] = data_l | 0x0D;  //en=1, rs=1
	data_t[3] = data_l | 0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit ( &hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100 );
}

void LCD_Clear( void )
{
	LCD_Send_Cmd ( 0x00 );
	for ( int i = 0; i < 100; i++ ) {
		LCD_Send_Data (' ');
	}
}

void LCD_Init(void)
{
	// 4 bit initialisation
	HAL_Delay( 50 );  // wait for >40ms
	LCD_Send_Cmd( 0x30 );
	HAL_Delay( 5);  // wait for >4.1ms
	LCD_Send_Cmd( 0x30 );
	HAL_Delay( 1);  // wait for >100us
	LCD_Send_Cmd( 0x30 );
	HAL_Delay( 10 );
	LCD_Send_Cmd( 0x20 );  // 4bit mode
	HAL_Delay( 10 );

  // dislay initialisation
	LCD_Send_Cmd ( 0x28 ); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	LCD_Send_Cmd ( 0x08 ); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	LCD_Send_Cmd ( 0x01 );  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	LCD_Send_Cmd ( 0x06 ); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	LCD_Send_Cmd ( 0x0C ); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void LCD_Send_String (char *str)
{
	while ( *str )
		LCD_Send_Data (*str++);
}
