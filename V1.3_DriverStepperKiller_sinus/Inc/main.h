/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

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
#define PWR_LED_BLUE_Pin GPIO_PIN_13
#define PWR_LED_BLUE_GPIO_Port GPIOC
#define VBAT_Pin GPIO_PIN_1
#define VBAT_GPIO_Port GPIOA
#define VBAT_EXTI_IRQn EXTI1_IRQn
#define BAT_LOW_LVL_Pin GPIO_PIN_4
#define BAT_LOW_LVL_GPIO_Port GPIOA
#define M3__EN_Pin GPIO_PIN_12
#define M3__EN_GPIO_Port GPIOB
#define M2__EN_Pin GPIO_PIN_13
#define M2__EN_GPIO_Port GPIOB
#define M1__EN_Pin GPIO_PIN_14
#define M1__EN_GPIO_Port GPIOB
#define MOT3_Pin GPIO_PIN_8
#define MOT3_GPIO_Port GPIOA
#define MOT2_Pin GPIO_PIN_9
#define MOT2_GPIO_Port GPIOA
#define MOT1_Pin GPIO_PIN_10
#define MOT1_GPIO_Port GPIOA
#define HLFB3_Pin GPIO_PIN_15
#define HLFB3_GPIO_Port GPIOA
#define HLFB2_Pin GPIO_PIN_3
#define HLFB2_GPIO_Port GPIOB
#define HLFB1_Pin GPIO_PIN_4
#define HLFB1_GPIO_Port GPIOB
#define M3_ROT_Pin GPIO_PIN_5
#define M3_ROT_GPIO_Port GPIOB
#define M2_ROT_Pin GPIO_PIN_8
#define M2_ROT_GPIO_Port GPIOB
#define M1_ROT_Pin GPIO_PIN_9
#define M1_ROT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define ROTATE_CW		GPIO_PIN_SET
#define ROTATE_CCW	GPIO_PIN_RESET

#define FORWARD		GPIO_PIN_SET
#define BACKWARD	GPIO_PIN_RESET

#define ON 		GPIO_PIN_RESET
#define OFF 		GPIO_PIN_SET


#define TRUE 1
#define FALSE 0
#define MAX_RCR_VAL		200  /*(According to datasheet, the RCR register(timer register) can be set to a max value of 255) */

#define TIMER_MIN_PERIOD_INCREMENT	125 //ns
#define TIMER_MAX_PERIOD_VALUE			65536 // (65536*125)ns

#define MINUTE_TO_US							(uint32_t)(MINUTE_TO_SECOND*SECOND_TO_US)
#define MINUTE_TO_SECOND				60
#define SECOND_TO_US							(uint32_t)(SECOND_TO_MILISECOND*MILISECOND_TO_US)
#define SECOND_TO_MILISECOND		1000
#define MILISECOND_TO_US					(uint32_t)(1000*ONE_US)
#define ONE_US	1
#define US_TO_NS		1000
#define MILISECOND_TO_NS					(uint32_t)(MILISECOND_TO_US*US_TO_NS)
#define SECOND_TO_NS		(uint32_t)(SECOND_TO_MILISECOND*MILISECOND_TO_NS)

#define MOT_TIME_DISABLE_ENABLE		15

/* The following macros specifies a certain character in the received message over USB to be checked in order to take the right action */

// Configuration macros
#define SET_STEP_RESOLUTION 'R'
#define SET_AXIS_LEN	'L'
#define SET_SCREW_STEP	'P'
#define SET_LOAD_MASS		'm'
#define SET_TRAJECTORY_REP	'x'

// Comand macros
#define STEPS_TO_BE_PERFORMED 's'
#define DISTANCE_TO_BE_PERFORMED 'D'
#define MOT_ANGULAR_VEL 'W'
#define MASS_LINEAR_VEL 'V'
#define MOT_ELAPSED_TIME	'E'
#define MOT_TIME_INTERVAL	'I'
#define MOT_ACC_TIME	'>'
#define MOT_DEC_TIME	'<'
// Motor step rezolution options
#define STEPS_PER_REV_200 			(uint32_t)200
#define STEPS_PER_REV_400 			(uint32_t)400
#define STEPS_PER_REV_800 			(uint32_t)800
#define STEPS_PER_REV_1600 				(uint32_t)1600
#define STEPS_PER_REV_3200 				(uint32_t)3200
#define STEPS_PER_REV_6400 				(uint32_t)6400
#define STEPS_PER_REV_12800 				(uint32_t)12800
#define STEPS_PER_REV_25600 				(uint32_t)25600
#define STEPS_PER_REV_51200 				(uint32_t)51200

// PWM min period according to motor rezolution
#define PWM_MIN_PERIOD_AT_200_US		128
#define PWM_MIN_PERIOD_AT_400_US		64
#define PWM_MIN_PERIOD_AT_800_US		32
#define PWM_MIN_PERIOD_AT_1600_US			16
#define PWM_MIN_PERIOD_AT_3200_US			8
#define PWM_MIN_PERIOD_AT_6400_US			4
#define PWM_MIN_PERIOD_AT_12800_US			4
#define PWM_MIN_PERIOD_AT_25600_US			4
#define PWM_MIN_PERIOD_AT_51200_US			4

// Macro for the number of steps to be performed for one mechanical rotation. This value depends on motor type
#define STEPS_FOR_1_MECHANICAL_ROTATION		6400

#define ROWS		1
#define COLUMNS		201

#define NO_OF_REP			1
#define VALUE_OF_PWM_PERIOD		0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
