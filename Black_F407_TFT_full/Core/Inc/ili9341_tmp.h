/*
 * ili9341.h
 *
 *  Created on: Jan 7, 2021
 *      Author: Alex
 */

#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include "i2c-lcd.h"
#include <stdio.h>

// ---------

//#define ADDR_CMD 	 *(uint32_t *)0x60000000
//#define ADDR_DATA 	 *(uint32_t *)0x60010000

#define ADDR_CMD 	*(volatile uint8_t*) (0x60000000)
#define ADDR_DATA	*(volatile uint8_t*) (0x60010000)

#define swap (1, b) { int16_t t = a; a = b; b = t }
#define RESET_ACTIVE HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET)
#define RESET_IDLE	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET)

// ----------


// New color definitions.  thanks to Bodmer
#define BLACK       0x0000      /*   0,   0,   0 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define RED         0xF800      /* 255,   0,   0 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define CYAN        0x07FF      /*   0, 255, 255 */
#define MAGENTA     0xF81F      /* 255,   0, 255 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */



//#define TFT_NAVY        0x000F      /*   0,   0, 128 */
//#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
//#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
//#define TFT_MAROON      0x7800      /* 128,   0,   0 */
//#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
//#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
//#define TFT_LIGHTGREY   0xC618      /* 192, 192, 192 */
//#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
//#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
//#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
//#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
//#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
//#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
//#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
//#define TFT_PINK        0xFC9F


// ----- CMDs ----
#define TFT9341_CMD_READ_ID 0xD3


void TFT9341_SetRotation( uint8_t r);
void TFT9341_Init( void );

#endif /* INC_ILI9341_H_ */
