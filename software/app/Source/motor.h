/*
 * motor.h
 *
 *  Created on: Jun 8, 2025
 *      Author: phsx
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"
#include "config.h"

typedef enum {
	MODE_DUMB = 0,
    MODE_POSITION = 1,    // Position control mode
    MODE_VELOCITY = 2     // Velocity control mode
} MotorMode;

typedef enum {
	STATE_BRIDGE_THERMAL_SHUTDOWN,
	STATE_MOTOR_THERMAL_SHUTDOWN,
	STATE_RUNNING_CW,
	STATE_RUNNING_CCW,
    STATE_STOP,
	STATE_STOP_POSITION_REACHED,
	STATE_STOP_ENDSTOP_CW,
	STATE_STOP_ENDSTOP_CCW,
	STATE_ESTOP,
} MotorState;


void MOTOR_EndstopISR(uint16_t GPIO_Pin);
void MOTOR_Init();
void MOTOR_MainLoop();

void MOTOR_SetPosition(int64_t pos);
void MOTOR_SetVelocity(int16_t RPS);
void MOTOR_SetDirection(uint16_t pwmA, uint16_t pwmB);
void MOTOR_PIDUpdateTunings();


#endif /* MOTOR_H_ */
