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
typedef struct anglePoints {
	float yaw;
	float pitch;
	float roll;
} anglePoints;
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
#define DebugOut_Pin GPIO_PIN_4
#define DebugOut_GPIO_Port GPIOE
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define X 0
#define Y 1
#define Z 2
#define YAW 0
#define PITCH 1
#define ROLL 2
#define THROTTLE 3
#define MPUAddress 113

/*
#define STATE_OFF 0
#define STATE_LAUNCHING 2
#define STATE_ON 1
#define STATE_CALIBRATING 3
*/

typedef enum {
	STATE_OFF = 0,
	STATE_ON = 1,
	STATE_LAUNCHING = 2,
	STATE_CALIBRATING = 3
} States;

#define SWITCH_ON 1
#define SWITCH_OFF 0

#define MIN_PULSE_OUTPUT 1000
#define MAX_PULSE_OUTPUT 2000

#define MAX_PITCH_INPUT 500.0f
#define MAX_YAW_INPUT 500.0f
#define MAX_ROLL_INPUT 500.0f

#define MAX_PITCH_POINT 20.0f
#define MAX_YAW_POINT 30.0f
#define MAX_ROLL_POINT 20.0f

/*
#define PITCH_BOTTOM_TRESHOLD 1492.0f
#define PITCH_UPPER_TRESHOLD 1508.0f
#define YAW_BOTTOM_TRESHOLD 1492.0f
#define YAW_UPPER_TRESHOLD 1508.0f
#define ROLL_BOTTOM_TRESHOLD 1492.0f
#define ROLL_UPPER_TRESHOLD 1508.0f
*/

#define PITCH_BOTTOM_TRESHOLD 1500.0f
#define PITCH_UPPER_TRESHOLD 1500.0f
#define YAW_BOTTOM_TRESHOLD 1492.0f
#define YAW_UPPER_TRESHOLD 1508.0f
#define ROLL_BOTTOM_TRESHOLD 1500.0f
#define ROLL_UPPER_TRESHOLD 1500.0f

#define RAD_TO_DEG 57.29578f

#define FAILSAFE_TRESHOLD 500

#define DEBUG 1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
