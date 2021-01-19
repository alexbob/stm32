/*
 * ATH10 temperature & humidity sensor
 *
 *  Alex
 *
 */

#include "main.h"
#include "AHT10.h"

/* Variables for AHT10 */
uint8_t AHT10_RX_Data[6];
//uint32_t AHT10_ADC_RawT;
uint32_t AHT10_ADC_Raw;

float AHT10_Temperature;
float AHT10_Humidity;

uint8_t AHT10_TmpHum_Cmd[3] = { 0xAC, 0x33, 0x00 };

#define AHT10_ADRESS (0x38 << 1) // 0b1110000; Adress[7-bit]Wite/Read[1-bit]

/* Additional vars */
//uint8_t T_100ms = 255;   // time flag
uint8_t AHT10_Switcher = 255;


float AHT10_Get_Temperature(void) {

	AHT10_Send_Command();
	HAL_Delay(100);
	AHT10_Read_Sensor();

	//			if (~AHT10_RX_Data[0] & 0x80) {
	/* Convert to Temperature in °C */
	AHT10_ADC_Raw = (((uint32_t) AHT10_RX_Data[3] & 0x0F) << 16)
			| ((uint32_t) AHT10_RX_Data[4] << 8) | AHT10_RX_Data[5];
	AHT10_Temperature = (float) (AHT10_ADC_Raw * 200.00 / 1048576.00) - 50.00;

	return AHT10_Temperature;

}

float AHT10_Get_Humidity(void) {



	/* Convert to Relative Humidity in % */
	AHT10_ADC_Raw = ((uint32_t) AHT10_RX_Data[1] << 12)
			| ((uint32_t) AHT10_RX_Data[2] << 4) | (AHT10_RX_Data[3] >> 4);
	AHT10_Humidity = (float) (AHT10_ADC_Raw * 100.00 / 1048576.00);

	return AHT10_Humidity;
}

void AHT10_Read_Sensor(void) {

	HAL_I2C_Master_Receive(&hi2c1, AHT10_ADRESS | 0x01, AHT10_RX_Data, 6, 50);
}

void AHT10_Send_Command(void) {
	HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADRESS, (uint8_t*) AHT10_TmpHum_Cmd, 3, 50);

}
