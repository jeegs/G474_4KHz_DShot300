/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	icm20602, bmp390L, hmc5983, SE100_HMC5983, SE100_M8N, FS_iA6B, BATTERY, NONE
} Sensor;

typedef enum {
	ROLL, PITCH, YAW, ALT, LAT, LNG, POS, AUTONOMOUS, RTH, WAYPOINTS, CIRCULAR, FOLLOWME
} Which;

typedef enum {
	Manual, Auto
} Flight;

typedef enum {
	_3S, _4S
} Battery;

//extern Battery batteryType;
//
//extern uint32_t loop_counter;
//extern uint16_t loop_start;//For DEBUG:
//extern uint16_t used_clocks;//For DEBUG:
//
extern uint8_t ARMED;
extern uint8_t FM;
//
//extern uint16_t Motor1, Motor2, Motor3, Motor4;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_CS_Pin LL_GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define HC12_SET_Pin LL_GPIO_PIN_0
#define HC12_SET_GPIO_Port GPIOB
#define SPI2_CS_Pin LL_GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define LED_Green_Pin LL_GPIO_PIN_8
#define LED_Green_GPIO_Port GPIOA
#define LED_Red_Pin LL_GPIO_PIN_9
#define LED_Red_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
// 제어 루프 주기 (TIM7: 160MHz / (PSC 39 + 1) = 4MHz 카운터, ARR 999 -> 4kHz)
// 루프 주기 상수는 여기 한 곳에서만 정의하고 main/imu/rc 에서 공통으로 사용한다.
#define FC_LOOP_HZ      4000.0f
#define FC_LOOP_DT      0.00025f   // 1 / FC_LOOP_HZ [s]
#define FC_LOOP_CLOCKS  1000u      // TIM7 카운트 수 (0.25us 단위) = 250us
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
