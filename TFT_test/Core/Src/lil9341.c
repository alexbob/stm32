/*
 * lil9341.c
 *
 *  Created on: Jan 6, 2021
 *      Author: Alex
 */

#include "lil9341.h"

void TFT9341_Init( void )
{

}

void TFT_Delay( dly )
{

	for (uint32_t i = 0; i < dly; i++ );
}

__STATIC_INLINE void Delay_Micro( unit32_t __IO micros)
{
	micros *=(SystemCoreClock/1000000)/5;
	while (micros--);
}
