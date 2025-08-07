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

void ENCODER_Init();
void ENCODER_ISR();
void ENCODER_MainLoop();
void ENCODER_ResetAbsolutePosition();
// API Required for MOTOR module
int64_t ENCODER_GetAbsolutePosition();
int16_t ENCODER_GetRPS();



void Encoder_Init();

#endif /* INC_ENC_H_ */
