/*
 * ATH10.h
 *
 *  Created on: Mar 27, 2022
 *      Author: alexbobkov
 */

#ifndef INC_ATH10_H_
#define INC_ATH10_H_

char * _float_to_char(float x, char *p);

float ATH10_Get_Temperature( I2C_HandleTypeDef i2c, uint8_t aht10Addr, uint8_t TempHum ); // Read temp
float ATH10_Get_Humidity( I2C_HandleTypeDef i2c, uint8_t aht10Addr, uint8_t TempHum );	// Read Hum


#endif /* INC_ATH10_H_ */
