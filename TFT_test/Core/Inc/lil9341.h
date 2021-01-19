/*
 * lil9341.h
 *
 *  Created on: Jan 6, 2021
 *      Author: Alex
 */

#ifndef INC_LIL9341_H_
#define INC_LIL9341_H_

#include "stm32f4xx_hal.h"
#include <stdlib.h>


#define ADDR_CMD *(unit8_t*)0x60000000
#define ADDR_DATA *(unit8_t*)0x60010000

#define swap(a,b) {int16_t t = a; a = b; b=t}
#define RESET_ACTIVE HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET)
#define RESET_IDLE HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET)

#define BLACK       0x0000      /*   0,   0,   0 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define RED         0xF800      /* 255,   0,   0 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define CYAN        0x07FF      /*   0, 255, 255 */
#define MAGENTA     0xF81F      /* 255,   0, 255 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */

//#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
//#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
//#define TFT_MAROON      0x7800      /* 128,   0,   0 */
//#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
//#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
//#define TFT_LIGHTGREY   0xC618      /* 192, 192, 192 */
//#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
//#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
//#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
//#define TFT_PINK        0xFC9F

void TFT9341_Init( void );

#endif /* INC_LIL9341_H_ */
