#include "stm32f1xx_hal.h"

void LCD_Init (void);   // initialize lcd

void LCD_Send_Cmd (char cmd);  // send command to the lcd

void LCD_Send_Data (char data);  // send data to the lcd

void LCD_Send_String (char *str);  // send string to the lcd

void LCD_Clear (void);
