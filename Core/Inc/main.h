/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define UNLOCK_TW_VALVE_Pin GPIO_PIN_0
#define UNLOCK_TW_VALVE_GPIO_Port GPIOC
#define WORK_LIGHT_Pin GPIO_PIN_11
#define WORK_LIGHT_GPIO_Port GPIOF
#define SEAT_LAMP_Pin GPIO_PIN_12
#define SEAT_LAMP_GPIO_Port GPIOF
#define WORK_LIGHTF13_Pin GPIO_PIN_13
#define WORK_LIGHTF13_GPIO_Port GPIOF
#define SIDE_SHIFT_R_Pin GPIO_PIN_14
#define SIDE_SHIFT_R_GPIO_Port GPIOF
#define SIDE_SHIFT_L_Pin GPIO_PIN_15
#define SIDE_SHIFT_L_GPIO_Port GPIOF
#define TILT_LOCK_VALVE_Pin GPIO_PIN_7
#define TILT_LOCK_VALVE_GPIO_Port GPIOE
#define LOCK_TW_VALVE_Pin GPIO_PIN_8
#define LOCK_TW_VALVE_GPIO_Port GPIOE
#define PWM1_IN_TO_20_FOOT_Pin GPIO_PIN_9
#define PWM1_IN_TO_20_FOOT_GPIO_Port GPIOE
#define TILT_LOCK_VALVEE10_Pin GPIO_PIN_10
#define TILT_LOCK_VALVEE10_GPIO_Port GPIOE
#define PWM2_ROTATE_CW_Pin GPIO_PIN_11
#define PWM2_ROTATE_CW_GPIO_Port GPIOE
#define LOCK_TW_LIGHT_Pin GPIO_PIN_12
#define LOCK_TW_LIGHT_GPIO_Port GPIOE
#define PWM3_ROTATE_CCW_Pin GPIO_PIN_13
#define PWM3_ROTATE_CCW_GPIO_Port GPIOE
#define PWM4_OUT_TO_40_FOOT_Pin GPIO_PIN_14
#define PWM4_OUT_TO_40_FOOT_GPIO_Port GPIOE
#define WORK_LIGHTE15_Pin GPIO_PIN_15
#define WORK_LIGHTE15_GPIO_Port GPIOE
#define UNLOCK_TW_LIGHT_Pin GPIO_PIN_12
#define UNLOCK_TW_LIGHT_GPIO_Port GPIOB
#define CENTRAL_LUBRICANT_Pin GPIO_PIN_13
#define CENTRAL_LUBRICANT_GPIO_Port GPIOB
#define SPREADER_ALARM_Pin GPIO_PIN_14
#define SPREADER_ALARM_GPIO_Port GPIOB
#define SENSOR_30_35_STOP_Pin GPIO_PIN_2
#define SENSOR_30_35_STOP_GPIO_Port GPIOG
#define UNLOCKED_TW__LEFT_Pin GPIO_PIN_6
#define UNLOCKED_TW__LEFT_GPIO_Port GPIOG
#define LOCKED_TW_LEFT_Pin GPIO_PIN_8
#define LOCKED_TW_LEFT_GPIO_Port GPIOG
#define LOCKED_TW_RIGHT_Pin GPIO_PIN_10
#define LOCKED_TW_RIGHT_GPIO_Port GPIOC
#define DAMPING_20_40_FOOT_Pin GPIO_PIN_12
#define DAMPING_20_40_FOOT_GPIO_Port GPIOC
#define SENSOR_SEAT_REAR__RIGHT_Pin GPIO_PIN_3
#define SENSOR_SEAT_REAR__RIGHT_GPIO_Port GPIOD
#define UNLOCKED_TW_RIGHT_Pin GPIO_PIN_4
#define UNLOCKED_TW_RIGHT_GPIO_Port GPIOD
#define SENSOR_SEAT_FRONT_RIGHT_Pin GPIO_PIN_5
#define SENSOR_SEAT_FRONT_RIGHT_GPIO_Port GPIOD
#define SENSOR_SEAT_REAR_LEFT_Pin GPIO_PIN_6
#define SENSOR_SEAT_REAR_LEFT_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
