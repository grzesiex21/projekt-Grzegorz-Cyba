/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <hts221_register_map.h>

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//#define LPS22HB_I2C_ADDRESS (0b01011101<<1)
#define HTS221_I2C_ADDRESS  (0b01011111<<1)



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static volatile bool data_ready = false;
static volatile bool read_data = false;
static volatile bool reading_completed = false;
static volatile bool blink_ready = false;
static volatile bool button_pushed = false;


static int16_t temperature;
static uint16_t humidity;

static char str[64] = {0};




volatile int16_t H0_T0_out, H1_T0_out, H_T_out;
volatile int16_t H0_rh, H1_rh;

volatile int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
volatile int16_t T0_degC, T1_degC;
uint8_t buffer[4], tmp;
int32_t tmp32;
uint8_t buf;

uint8_t buffer_hts221[64];
//float _hts221HumiditySlope;
// float _hts221HumidityZero;
// float _hts221TemperatureSlope;
// float _hts221TemperatureZero;



void hts221_read(uint8_t adress, uint8_t *dest, uint8_t num)
{
	HAL_I2C_Mem_Read(&hi2c1, HTS221_I2C_ADDRESS, adress, I2C_MEMADD_SIZE_8BIT, dest, num, HAL_MAX_DELAY);
}



void hts221_read_IT(uint8_t adress, uint8_t num)
{
//	  HAL_I2C_Mem_Read_IT(&hi2c1, HTS221_I2C_ADDRESS, (uint16_t)adress, I2C_MEMADD_SIZE_8BIT, buffer_hts221, num);
	  HAL_I2C_Mem_Read_IT(&hi2c1, HTS221_I2C_ADDRESS, adress, I2C_MEMADD_SIZE_8BIT, buffer_hts221, num);
//	  HAL_I2C_Master_Receive_IT(&hi2c1, HTS221_I2C_ADDRESS, buffer_hts221, num);
}

//void hts221_read_IT(uint8_t adress, uint8_t *dest, uint8_t num)
//{
//	  HAL_I2C_Mem_Read_IT(&hi2c1, HTS221_I2C_ADDRESS, adress, 1, dest, num);
//}

void hts221_write(uint8_t adress, uint8_t value)
{
	  HAL_I2C_Mem_Write(&hi2c1, HTS221_I2C_ADDRESS, adress, 1, &value, 1, HAL_MAX_DELAY);
}

void hts221_take_one_shot()
{
	hts221_read(HTS221_CTRL_REG2, &buf, 1);
	if(buf & 0x01){
	  	sprintf(str, "chuja nie trigger\n\r");
		HAL_UART_Transmit_IT(&huart2,  (uint8_t *)str,  strlen(str));
	} else {
	  	sprintf(str, "Trigger.\n\r");
		HAL_UART_Transmit_IT(&huart2,  (uint8_t *)str,  strlen(str));
		hts221_write(HTS221_CTRL_REG2, 0x01);
	}
}

void hts221_init_hum_calibration()
{
	hts221_read(HTS221_H0_rH_x2, buffer, 2);
	H0_rh = buffer[0]>>1;
	H1_rh = buffer[1]>>1;
	hts221_read(HTS221_H0_T0_OUT_L, buffer, 2);
	H0_T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
	hts221_read(HTS221_H1_T0_OUT_L, buffer, 2);
	H1_T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

//	hts221_read(HTS221_H0_rH_x2, buffer, 2);
//	H0_rh = buffer[0];
//	H1_rh = buffer[1];
//	hts221_read(HTS221_H0_T0_OUT_L, buffer, 2);
//	H0_T0_out = (((int16_t)buffer[1])<<8) | (int16_t)buffer[0];
//	hts221_read(HTS221_H1_T0_OUT_L, buffer, 2);
//	H1_T0_out = (((int16_t)buffer[1])<<8) | (int16_t)buffer[0];
//  _hts221HumiditySlope = (H1_rh - H0_rh) / (2.0 * (H1_T0_out - H0_T0_out));
//  _hts221HumidityZero = (H0_rh / 2.0) - _hts221HumiditySlope * H0_T0_out;
}

void hts221_init_temp_calibration()
{
	hts221_read(HTS221_T0_degC_x8, buffer, 2);
	hts221_read(HTS221_T1_T0_msb, &tmp, 1);
	T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buffer[0]);
	T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buffer[1]);
	T0_degC = T0_degC_x8_u16>>3;
	T1_degC = T1_degC_x8_u16>>3;
	hts221_read(HTS221_T0_OUT_L, buffer, 4);
	T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
	T1_out = (((uint16_t)buffer[3])<<8) | (uint16_t)buffer[2];


//	hts221_read(HTS221_T0_degC_x8, buffer, 2);
//	hts221_read(HTS221_T1_T0_msb, &tmp, 1);
//	T0_degC = ((uint16_t)buffer[0]) | ((uint16_t)(tmp & 0x03) << 8);
//	T1_degC = ((uint16_t)buffer[1]) | ((uint16_t)(tmp & 0x0C) << 6);
//
//	hts221_read(HTS221_T0_OUT_L, buffer, 4);
//	T0_out = (((int16_t)buffer[1])<<8) | (int16_t)buffer[0];
//	T1_out = (((int16_t)buffer[3])<<8) | (int16_t)buffer[2];
//
//	_hts221TemperatureSlope = (T1_degC-T0_degC) / (8.f * (T1_out - T0_out));
//	_hts221TemperatureZero = (T0_degC / 8.f) - _hts221TemperatureSlope * T0_out;


//	hts221_read(HTS221_TEMP_OUT_L, buffer, 4);
//	T_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
//	tmp32 = (( int32_t)(T_out - T0_out)) * (( int32_t)(T1_degC - T0_degC)*10);
//	*value = tmp32 /(T1_out - T0_out) + T0_degC*10;

}

void hts221_get_temp_hum(int16_t *value_t, uint16_t *value_h)
{
//	hts221_read_IT(HTS221_HUMIDITY_OUT_L, 4);
//	hts221_read(HTS221_HUMIDITY_OUT_L, buffer_hts221, 4);
	T_out = (((uint16_t)buffer_hts221[3])<<8) | (uint16_t)buffer_hts221[2];
	H_T_out = (((uint16_t)buffer_hts221[1])<<8) | (uint16_t)buffer_hts221[0];

//*value_t = T_out;
//*value_h = H_T_out;

	tmp32 = (( int32_t)(T_out - T0_out)) * (( int32_t)(T1_degC - T0_degC)*10);
	*value_t = (int16_t)(tmp32 /(T1_out - T0_out) + (T0_degC*10));
	tmp32 = ((int32_t)(H_T_out - H0_T0_out)) * ((int32_t)(H1_rh - H0_rh)*10);
	*value_h = (uint16_t)(tmp/(H1_T0_out - H0_T0_out) + H0_rh*10);
	if(*value_h>1000) *value_h = 1000;

//	H_T_out = (((int16_t)buffer_hts221[1])<<8) | (int16_t)buffer_hts221[0];
//	T_out = (((int16_t)buffer_hts221[3])<<8) | (int16_t)buffer_hts221[2];
//	*value_t = T_out*_hts221TemperatureSlope+_hts221TemperatureZero;
//	*value_h = H_T_out*_hts221HumiditySlope+_hts221HumidityZero;
}

void hts221_begin()
{
	//  hts221_write(HTS221_REBOOT);
//	hts221_write(HTS221_CTRL_REG1, (0<<7));  // deactive mode
	hts221_write(HTS221_CTRL_REG2, (1<<7)); // reboot
	hts221_write(HTS221_CTRL_REG3, (1<<2)); // data ready signal
//	hts221_write(HTS221_CTRL_REG3, (1<<7)); // data ready active low signal
	hts221_write(HTS221_CTRL_REG1, (1<<7) | (1<<2) | (0<<0)); // active mode | BDU enable | one shot mode enable
//	hts221_write(HTS221_CTRL_REG2, (1<<7)); // reboot
//	hts221_read(HTS221_CTRL_REG1, &buf, 1);
//	hts221_read(HTS221_CTRL_REG2, &buf, 1);
//	hts221_read(HTS221_CTRL_REG3, &buf, 1);
//	HAL_Delay(100);

//	hts221_write(HTS221_CTRL_REG1, (1<<0)); // oddr 1Hz


//		hts221_read(HTS221_HUMIDITY_OUT_L, buffer_hts221, 4);
//	    hts221_read_IT(HTS221_HUMIDITY_OUT_L, 4); // pierwszy odczyt

	//  hts221_write(HTS221_CTRL_REG1, (1<<7) | (1<<2)); // active mode | output registers not updated until MSB and LSB reading
//	  hts221_write(HTS221_CTRL_REG1, (1<<7));
//	  hts221_write(HTS221_AV_CONF, (7<<0)); // jakies resolution
//	hts221_write(HTS221_CTRL_REG1, (0<<7) | (0<<2)); // disable active mode | output registers not updated until MSB and LSB reading
//	hts221_write(HTS221_CTRL_REG1, (1<<7)); // active mode | output registers not updated until MSB and LSB reading

	hts221_init_temp_calibration();
	hts221_init_hum_calibration();
//	hts221_get_temp_hum(&temperature, &humidity);

//	hts221_write(HTS221_CTRL_REG2, (1<<7)); // reboot
//	hts221_write(HTS221_CTRL_REG3, (1<<2)); // data ready signal
//	hts221_write(HTS221_CTRL_REG1, (1<<7) | (1<<2)); // active mode | output registers not updated until MSB and LSB reading
//	hts221_write(HTS221_CTRL_REG1, (uint8_t)0x84); // active mode | output registers not updated until MSB and LSB reading

}



void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c1)
	{
		reading_completed = true;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == HTS221_DRDY_Pin)
	{
		data_ready = true;
	}
	if(GPIO_Pin == B1_Pin)
	{
		button_pushed = true;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim4) // 26 Hz
	{
		blink_ready = true;
	}
	if(htim==&htim2) // 1 Hz
	{
		read_data = true;

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint8_t who_i_am;
  hts221_read(HTS221_WHO_AM_I, &who_i_am, 1);

  if(who_i_am == 0b10111100) {
	  	sprintf(str, "Wykryto czujnik.\n\r");
		HAL_UART_Transmit_IT(&huart2,  (uint8_t *)str,  strlen(str));
  }
  hts221_begin();
  hts221_read_IT(HTS221_HUMIDITY_OUT_L, 4);
//	hts221_read(HTS221_HUMIDITY_OUT_L, buffer_hts221, 4);


  while (1)
  {
	  if(blink_ready) {
		  blink_ready = false;
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  }

	  if(button_pushed) {
		  button_pushed = false;
		  sprintf(str, "Wcisnieto przycisk.\n\r");
		  HAL_UART_Transmit_IT(&huart2,  (uint8_t *)str,  strlen(str));
	  }

	  if (read_data) {
		  read_data = false;
		  hts221_take_one_shot();
//			hts221_read_IT(HTS221_HUMIDITY_OUT_L, 4);
//			hts221_get_temp_hum(&temperature, &humidity);
	  }
////
//	  if(HAL_GPIO_ReadPin(HTS221_DRDY_GPIO_Port, HTS221_DRDY_Pin) == true)
//		  {
//			data_ready = false;
//	//		hts221_read_IT(HTS221_HUMIDITY_OUT_L, 4);
//	//			  		hts221_get_temperature(&temperature);
//			hts221_read_IT(HTS221_HUMIDITY_OUT_L, 4);
//
////			hts221_get_temp_hum(&temperature, &humidity);
//		  }

//	  hts221_read(HTS221_STATUS_REG, &buf, 1);
//	  	if(buf & 0x03){
//	  		data_ready = true;
//	  	}

	  if(data_ready == true) {
		data_ready = false;
		hts221_read_IT(HTS221_HUMIDITY_OUT_L, 4);
//		hts221_read(HTS221_HUMIDITY_OUT_L, buffer_hts221, 4);
	  }
	  if(reading_completed) {
		hts221_get_temp_hum(&temperature, &humidity);

		reading_completed = false;
		sprintf(str, "Temp: %+5.2f C \t Hum: %5.2f %% \n\r", temperature/10.f, humidity/10.f);
		HAL_UART_Transmit_IT(&huart2,  (uint8_t *)str,  strlen(str));
	  }


//	  if(reading_completed == true)
//	  	  {
//	  		reading_completed = false; // czyścimy bo inaczej klapa
//
////	  		hts221_get_temperature(&temperature);
////	  		hts221_read_IT(HTS221_TEMP_OUT_L, 2);
////	  		hts221_read(HTS221_TEMP_OUT_L, buffer_hts221, 2);
////	  		hts221_read_IT(HTS221_HUMIDITY_OUT_L, 4);
//
//	  		  humidity = (((uint32_t)buffer_hts221[1])<<8) | buffer_hts221[0];
//	  		  temperature = (((int32_t)buffer_hts221[3])<<8) | buffer_hts221[2];

//
//	  		  sprintf(str, "Temp: %+10.2ld C \tHum: %+10.5d %%\n\r", temperature, humidity);
//	  		  HAL_UART_Transmit_IT(&huart2,  (uint8_t *)str,  strlen(str));
//
//	  	  }


//
//	  if(read_data == true)
//	  	  {
//	  		read_data = false; // czyścimy bo inaczej klapa
//
////	  		hts221_init_temp_calibration(&temperature);
//	  		hts221_get_temperature(&temperature);
////	  		hts221_read_IT(HTS221_TEMP_OUT_L, 2);
////	  		hts221_read(HTS221_TEMP_OUT_L, buffer_hts221, 2);
////	  		hts221_read_IT(HTS221_HUMIDITY_OUT_H, 2);
//
////	  		  humidity = (((uint32_t)buffer_hts221[1])<<8) | buffer_hts221[0];
////	  		  temperature = (((uint32_t)buffer_hts221[1])<<8) | buffer_hts221[0];
////	  		  pressure_SI = pressure/4096.f;
////	  		  temperature_SI = temperature/100.f;
//
////	  		  sprintf(str, "Hum: %+10.5d C\n\r", humidity);
//	  		  sprintf(str, "Temp: %+10.2d C\n\r", temperature);
////	  		  sprintf(str, "Hello");
//	  		  HAL_UART_Transmit_IT(&huart2,  (uint8_t *)str,  strlen(str));
//	  	  }

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
