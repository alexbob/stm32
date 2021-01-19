/*
*  AHT10 - A Humidity Library for STM32.
*
*/
#include <stdint.h>

#ifndef AHT10_H
#define AHT10_H



typedef unsigned char Sensor_CMD;
extern I2C_HandleTypeDef hi2c1;

float AHT10_Get_Temperature( void );
float AHT10_Get_Humidity( void );
void AHT10_Read_Sensor(void);
void AHT10_Send_Command(void);

#endif
