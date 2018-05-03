/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define RADIO_DIO_0_PORT                          GPIOA
#define RADIO_DIO_0_PIN                           GPIO_PIN_10 //GPIO_PIN_15 GPIO_PIN_10

#define RADIO_DIO_1_PORT                          GPIOB
#define RADIO_DIO_1_PIN                           GPIO_PIN_3

#define RADIO_DIO_2_PORT                          GPIOB
#define RADIO_DIO_2_PIN                           GPIO_PIN_5

#define RADIO_DIO_3_PORT                          GPIOB
#define RADIO_DIO_3_PIN                           GPIO_PIN_4


#define RADIO_MOSI_PORT      	GPIOA
#define RADIO_MOSI_PIN      	GPIO_PIN_7

#define RADIO_MISO_PORT     	GPIOA
#define RADIO_MISO_PIN       	GPIO_PIN_6

#define RADIO_SCLK_PORT      	GPIOA
#define RADIO_SCLK_PIN       	GPIO_PIN_5

#define RADIO_NSS_PORT 			GPIOB //GPIOB//	GPIOA
#define RADIO_NSS_PIN 			GPIO_PIN_6 // GPIO_PIN_6//	GPIO_PIN_4

#define RADIO_RESET_PORT 			GPIOA
#define RADIO_RESET_PIN 			GPIO_PIN_0

#define RADIO_ANT_SWITCH1_PORT	GPIOA
#define RADIO_ANT_SWITCH1_PIN 	GPIO_PIN_11 //GPIO_PIN_11 GPIO_PIN_1
#define RADIO_ANT_SWITCH2_PORT	GPIOA
#define RADIO_ANT_SWITCH2_PIN 	GPIO_PIN_8

#define SPI_CLK_ENABLE()     	__HAL_RCC_SPI1_CLK_ENABLE()
#define SPI1_AF             	GPIO_AF0_SPI1  

#define RTC_OUTPUT       			DBG_RTC_OUTPUT

#define RTC_Alarm_IRQn       	RTC_IRQn

typedef struct
{
	int16_t lux;
	int16_t pressure;
	int16_t temperature;
	int16_t humidity;
	int16_t accelerometer_x;
	int16_t accelerometer_y;
	int16_t accelerometer_z;
	int16_t magnetometer_x;
	int16_t magnetometer_y;
	int16_t magnetometer_z;

} Sensro_TypeDef;

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
