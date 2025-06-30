/*
 * enc.c
 *
 *  Created on: Jun 5, 2025
 *      Author: phsx
 */

#include "enc.h"
#include "main.h"
#include "config.h"

volatile int64_t encoderAbsolutePosition = 0;
volatile int16_t encoderRPS = 0;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;


void ENCODER_Init(){

  htim3.Instance->CNT = ENCODER_MIDPOINT;
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

/*
 *
 * 10ms interrupt that will calculate speed of encoder and absolute position
 *
 */
void ENCODER_ISR(){
	static int64_t prevEncoderAbsolutePosition = 0;
	int64_t delta = encoderAbsolutePosition - prevEncoderAbsolutePosition;
	prevEncoderAbsolutePosition = encoderAbsolutePosition;

	encoderRPS = ((float)delta / (ENCODER_PR * ENCODER_MODE)) / 0.01f ;
}

void ENCODER_MainLoop(){
	int16_t delta = (int16_t)(TIM3->CNT - ENCODER_MIDPOINT) ;
	TIM3->CNT = ENCODER_MIDPOINT;
	encoderAbsolutePosition += delta;
}

int64_t ENCODER_GetAbsolutePosition()
{
	return encoderAbsolutePosition;
}
int16_t ENCODER_GetRPS()
{
	return encoderRPS;
}

void ENCODER_ResetAbsolutePosition()
{
	encoderAbsolutePosition = 0;
}
