/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define MY_CAN_ID (10)			// CAN ID OF THIS DEVICE

#define MOTOR_PWM_TIMER htim8		// TIMER USED FOR PULSE-WIDTH MODULATION FOR MOTOR CONTROL
#define MOTOR_PWM_CHANNEL_1 TIM_CHANNEL_3	// USED PWM CHANNELS
#define MOTOR_PWM_CHANNEL_2 TIM_CHANNEL_4
#define MY_MOTOR_PWM_PERIOD (8000)	// MAXIMUM PWM TIMER VALUE

#define ENCODER_TIMER htim3			// TIMER USED FOR THE ENCODER
#define ENCODER_PERIOD (65535)		// MAXIMUM VALUE OF THE ENCODER

#define USER_GENERAL_PURPOSE_TIMER htim7	// TIMER USED FOR MOST SYSTEM TASKS: READING ENCODER, CALCULATING PD VALUE
#define USER_GENERAL_PURPOSE_TIMER_FREQUENCY (20)	// FRQUENCY OF UGP TIMER. HERTZ

#define SLEEP_PIN_GROUP GPIOB			// SLEEP PIN IS USED TO CONTROL THE MOTOR DRIVER. IT MUST BE HIGH FOR MOTOR TO ROLL
#define SLEEP_PIN_NUMBER GPIO_PIN_5
#define VREF_PIN_GROUP GPIOA			// VREF IS THE PIN LIMITING MOTOR'S MAXIMUM POWER. JUST SET TO HIGH RIGHT NOW.
#define VREF_PIN_NUMBER GPIO_PIN_4

#define ODOM_TICKS_PER_RADIAN (1518.0933)	//odom ticks per radian

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
