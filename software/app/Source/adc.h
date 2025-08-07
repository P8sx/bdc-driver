/*
 * adc.h
 *
 *  Created on: Jun 20, 2025
 *      Author: phsx
 */

#ifndef ADC_H_
#define ADC_H_
#include "main.h"
#include "config.h"

#define ADC_NTC_MOTOR   0
#define ADC_MOTOR_IOUT  1
#define ADC_VMOTOR      2
#define ADC_NTC_BRIDGE  3



void ADC_Init();
void ADC_MainLoop();

float ADC_GetBridgeTemp();
float ADC_GetMotorTemp();
float ADC_GetMotorCurrent();
float ADC_GetVoltage();

void ADC_UpdateBridgeCurrentLimit();

#endif /* ADC_H_ */
