/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 Eemeli Pursiainen.
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
#include "stm32l0xx_hal.h"
#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_ll_crs.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_rtc.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"

#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern SPI_HandleTypeDef hspi1;
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
void AdcGrpRegularOverrunError_Callback(void);
void EnterStopMode(void);
void wakeUp_Callback(void);
void setc(void);
int main(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
typedef struct {
	uint8_t *ptr;
	uint16_t size;
	uint16_t counter;
} bufInfo;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
