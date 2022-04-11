/*
 * ATH10.c
 *
 *  Created on: Mar 27, 2022
 *      Author: alexbobkov
 */


// Variables for AHT10


#include "stm32f4xx_hal.h"
#include "ATH10.h"


//uint8_t AHT10_Switcher = 255;


// Additional vars


float ATH10_Get_Temperature (I2C_HandleTypeDef i2c, uint8_t aht10Addr, uint8_t TempHum) {

	uint8_t 	AHT10_RX_Data[6]; //
	uint32_t 	AHT10_ADC_Raw;

	uint8_t 	AHT10_TmpHum_Cmd[3] = {0xAC, 0x33, 0x00};
	float		result = 0;


		HAL_I2C_Master_Transmit(&i2c, aht10Addr, AHT10_TmpHum_Cmd, 1, 10	);  // should be via interrupt
		HAL_I2C_Master_Receive(&i2c, aht10Addr, (uint8_t*) AHT10_RX_Data, 6, 10); // should be via interrupt
		if (~AHT10_RX_Data[0] & 0x80) {
			if (TempHum) {  // Convert to Temperature in °C
				AHT10_ADC_Raw = (((uint32_t) AHT10_RX_Data[3] & 15) << 16) | ((uint32_t) AHT10_RX_Data[4] << 8) | AHT10_RX_Data[5];
				result = (uint8_t) (AHT10_ADC_Raw * 200.00 / 1048576.00) - 50.00;
			} else { 	// Convert to Relative Humidity in %
				AHT10_ADC_Raw = ((uint32_t) AHT10_RX_Data[1] << 12) | ((uint32_t) AHT10_RX_Data[2] << 4) | (AHT10_RX_Data[3] >> 4);
				result = (uint8_t) (AHT10_ADC_Raw * 100.00 / 1048576.00);
			}
		}


	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // for debug

	return(result);

}

char * _float_to_char(float x, char *p) {
    char *s = p + 5; // go to end of buffer CHAR_BUFF_SIZE
    uint16_t decimals;  // variable to store the decimals
    int units;  // variable to store the units (part to left of decimal place)
    if (x < 0) { // take care of negative numbers
        decimals = (int)(x * -100) % 100; // make 1000 for 3 decimals etc.
        units = (int)(-1 * x);
    } else { // positive numbers
        decimals = (int)(x * 100) % 100;
        units = (int)x;
    }

    *--s = (decimals % 10) + '0';
    decimals /= 10; // repeat for as many decimal places as you need
    *--s = (decimals % 10) + '0';
    *--s = '.';

    while (units > 0) {
        *--s = (units % 10) + '0';
        units /= 10;
    }
    if (x < 0) *--s = '-'; // unary minus sign for negative numbers
    return s;
}
