/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
void MX_IWDG_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM14_PSC 63
#define TIM14_AAR 9999
#define ADC_NTC_MOTOR_Pin GPIO_PIN_0
#define ADC_NTC_MOTOR_GPIO_Port GPIOA
#define ENDSTOP_CW_Pin GPIO_PIN_1
#define ENDSTOP_CW_GPIO_Port GPIOA
#define ENDSTOP_CW_EXTI_IRQn EXTI0_1_IRQn
#define ENDSTOP_CCW_Pin GPIO_PIN_2
#define ENDSTOP_CCW_GPIO_Port GPIOA
#define ENDSTOP_CCW_EXTI_IRQn EXTI2_3_IRQn
#define M_AIOUT_Pin GPIO_PIN_3
#define M_AIOUT_GPIO_Port GPIOA
#define M_VREF_Pin GPIO_PIN_4
#define M_VREF_GPIO_Port GPIOA
#define ENC_Z__SCK_Pin GPIO_PIN_5
#define ENC_Z__SCK_GPIO_Port GPIOA
#define ENC_A__MISO__SDA_Pin GPIO_PIN_6
#define ENC_A__MISO__SDA_GPIO_Port GPIOA
#define ENC_B__MOSI__SCL_Pin GPIO_PIN_7
#define ENC_B__MOSI__SCL_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB
#define ADC_VM_Pin GPIO_PIN_1
#define ADC_VM_GPIO_Port GPIOB
#define ADC_NTC_BRIDGE_Pin GPIO_PIN_2
#define ADC_NTC_BRIDGE_GPIO_Port GPIOB
#define EEPROM_SCL_Pin GPIO_PIN_13
#define EEPROM_SCL_GPIO_Port GPIOB
#define EEPROM_SDA_Pin GPIO_PIN_14
#define EEPROM_SDA_GPIO_Port GPIOB
#define M_SLEEPn_Pin GPIO_PIN_15
#define M_SLEEPn_GPIO_Port GPIOB
#define M_IN1_Pin GPIO_PIN_8
#define M_IN1_GPIO_Port GPIOA
#define M_IN2_Pin GPIO_PIN_9
#define M_IN2_GPIO_Port GPIOA
#define M_OCLn_Pin GPIO_PIN_6
#define M_OCLn_GPIO_Port GPIOC
#define AUX_Pin GPIO_PIN_10
#define AUX_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
