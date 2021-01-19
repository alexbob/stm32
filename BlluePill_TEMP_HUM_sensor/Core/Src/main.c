/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "i2c-lcd.h"
#include <stdio.h>
//#include "AHT10.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Variables for AHT10 */
uint8_t AHT10_RX_Data[6];
uint32_t AHT10_ADC_RawT;
uint32_t AHT10_ADC_Raw;

float AHT10_Temperature;
float AHT10_Humidity;
typedef unsigned char Sensor_CMD;

Sensor_CMD eSensorCalibrateCmd[3] = {0xE1, 0x08, 0x00};
Sensor_CMD eSensorNormalCmd[3]    = {0xA8, 0x00, 0x00};
Sensor_CMD eSensorMeasureCmd[3]   = {0xAC, 0x33, 0x00};
Sensor_CMD eSensorResetCmd        = 0xBA;


uint8_t AHT10_TmpHum_Cmd[3] = {0xAC, 0x33, 0x00};

#define AHT10_ADRESS (0x38 << 1) // 0b1110000; Adress[7-bit]Wite/Read[1-bit]


/* Additional vars */
uint8_t T_100ms = 255;
uint8_t AHT10_Switcher = 255;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == TIM4)
 {
	/* Set every 100ms */
	T_100ms = 255;
 }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  char temp[10];	// temperature sensor
  char humi[10];	// humidity sensor

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


  /* TIM4 Int. every 100ms */
  HAL_TIM_Base_Start_IT(&htim4);

  // init LCD
  lcd_init ();


  lcd_send_cmd ( 0x80 | 0x00 );
  lcd_send_string("AHT10 Sensor test");

/*
	Init
*/

 // HAL_I2C_Master_Transmit_IT(&hi2c1, AHT10_ADRESS, (uint8_t*)eSensorCalibrateCmd, 3); /* Send command (trigger measuremetns) + parameters */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if(T_100ms)
	{
		if(AHT10_Switcher)
			{
				HAL_I2C_Master_Transmit(&hi2c1, AHT10_ADRESS, (uint8_t*)AHT10_TmpHum_Cmd, 3, 50); /* Send command (trigger measuremetns) + parameters */
			}
		else
			{
				HAL_I2C_Master_Receive(&hi2c1, AHT10_ADRESS | 0x01, AHT10_RX_Data, 6, 50); /* Receive data: STATUS[1]:HIMIDITY[2.5]:TEMPERATURE[2.5] */
			}

		if(~AHT10_RX_Data[0] & 0x80)
		{
			/* Convert to Temperature in °C */
			AHT10_ADC_RawT = (((uint32_t)AHT10_RX_Data[3] & 0x0F) << 16) | ((uint32_t)AHT10_RX_Data[4] << 8) | AHT10_RX_Data[5];
			AHT10_Temperature = (float)(AHT10_ADC_RawT * 200.00 / 1048576.00) - 50.00;

			/* Convert to Relative Humidity in % */
			AHT10_ADC_Raw = ((uint32_t)AHT10_RX_Data[1] << 12) | ((uint32_t)AHT10_RX_Data[2] << 4) | (AHT10_RX_Data[3] >> 4);
			AHT10_Humidity = (float)(AHT10_ADC_Raw * 100.00 / 1048576.00);
			//memset(AHT10_RX_Data, 0, sizeof AHT10_RX_Data);
		}

		AHT10_Switcher = ~AHT10_Switcher; /* Invert */
		GPIOC->ODR ^= GPIO_ODR_ODR13; /* Green LED */

		sprintf(temp, "%.2f", AHT10_Temperature);
  //		sprintf(temp, "%lu", AHT10_Temperature);

  		sprintf(humi, "%.2f", AHT10_Humidity);

  		lcd_send_cmd (0x80|0x40);
  		lcd_send_string(temp);

  		lcd_send_cmd (0x80|0x1C);
 		lcd_send_string(humi);


  		//lcd_send_cmd (0x80|0x54);
  		//lcd_send_string(tmp);


		T_100ms = 0; /* Nulify */
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
