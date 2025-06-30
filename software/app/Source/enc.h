/*
 * enc.h
 *
 *  Created on: Jun 5, 2025
 *      Author: phsx
 */

#ifndef INC_ENC_H_
#define INC_ENC_H_


/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define ENCODER_MIDPOINT 			0x8000 	// Midpoint for 16bit Timer
#define ENCODER_INTERRUPT_FREQ 		100 	// Hz
#define ENCODER_MODE 				2 		// 2 - if only counting T1 or T2, 4 - if T1 and T2
#define ENCODER_PR					360		// Pulses per revolution

void ENCODER_Init();
void ENCODER_ISR();
void ENCODER_MainLoop();
void ENCODER_ResetAbsolutePosition();
// API Required for MOTOR module
int64_t ENCODER_GetAbsolutePosition();
int16_t ENCODER_GetRPS();



void Encoder_Init();

#endif /* INC_ENC_H_ */
