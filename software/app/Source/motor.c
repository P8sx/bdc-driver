/*
 * motor.c
 *
 *  Created on: Jun 8, 2025
 *      Author: phsx
 */


#include "motor.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "adc.h"
#include <stdbool.h>

#include "enc.h"
#include "pid.h"

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart3;

PID_TypeDef mPID;

MotorState currentState = STATE_STOP;

double pidInput, pidOutput, pidTarget;

#define IS_MODE(x) (cfg->motorControlMode == (x))
#define IS_STATE(x) (currentState == (x))
#define SET_STATE(x) (currentState = (x))


void MOTOR_SetPWM(uint16_t pwmA, uint16_t pwmB);
void MOTOR_Stop();


void MOTOR_PIDUpdateTunings()
{
	PID_SetTunings(&mPID, cfg->pidKp, cfg->pidKi, cfg->pidKd);
}

void MOTOR_SetPosition(int64_t pos)
{
	if(IS_STATE(STATE_STOP_POSITION_REACHED))
		SET_STATE(STATE_STOP);

	pidTarget = pos;
}

void MOTOR_SetVelocity(int16_t RPS)
{
	pidTarget = RPS;
}

void MOTOR_SetDirection(uint16_t pwmA, uint16_t pwmB)
{
	if(pwmA == 0 && pwmB == 0){
		MOTOR_Stop();
		SET_STATE(STATE_STOP);
		return;
	}
	MOTOR_SetPWM(pwmA, pwmB);
}

void MOTOR_EndstopISR(uint16_t GPIO_Pin)
{
	if(ENDSTOP_CW_Pin == GPIO_Pin && cfg->endstopCWEnabled && IS_STATE(STATE_RUNNING_CW))
	{
	    SET_STATE(STATE_STOP_ENDSTOP_CW);
	    MOTOR_Stop();
	    return;
	}
	else if(ENDSTOP_CCW_Pin == GPIO_Pin && cfg->endstopCCWEnabled && IS_STATE(STATE_RUNNING_CCW))
	{
	    SET_STATE(STATE_STOP_ENDSTOP_CCW);
	    MOTOR_Stop();
	    return;
	}
}

void MOTOR_Sleep(uint8_t state)
{
	if(state)
	  HAL_GPIO_WritePin(M_SLEEPn_GPIO_Port, M_SLEEPn_Pin, GPIO_PIN_RESET);
	else
	  HAL_GPIO_WritePin(M_SLEEPn_GPIO_Port, M_SLEEPn_Pin, GPIO_PIN_SET);
}

void MOTOR_Init()
{
  PID(&mPID, &pidInput, &pidOutput, &pidTarget, cfg->pidKp, cfg->pidKi, cfg->pidKd, _PID_P_ON_M, _PID_CD_DIRECT);

  PID_SetMode(&mPID, _PID_MODE_AUTOMATIC);

  PID_SetSampleTime(&mPID, MOTOR_MAIN_LOOP_PERIOD);

  PID_SetOutputLimits(&mPID, -MOTOR_PID_PWM_LIMIT, MOTOR_PID_PWM_LIMIT);

  MOTOR_Sleep(true);
}


void MOTOR_MainLoop()
{
	uint32_t now = HAL_GetTick();
	static uint32_t lastConversionTime = 0;


	/* ESTOP Handling */
	if( IS_STATE(STATE_ESTOP) )
	{
		SET_STATE(STATE_ESTOP);
	    MOTOR_Stop();
		return; // Do not execute any action if emergency stop has been set
	}


	/* Bridge over-temp handling */
	float bridgeTemp = ADC_GetBridgeTemp();
	if (bridgeTemp > MOTOR_BRIDGE_TEMP_LIMIT)
	{
	    SET_STATE(STATE_BRIDGE_THERMAL_SHUTDOWN);
	    MOTOR_Stop();
	    return;
	}
	else if (IS_STATE(STATE_BRIDGE_THERMAL_SHUTDOWN))
	{
	    if (bridgeTemp > MOTOR_BRIDGE_TEMP_LIMIT - MOTOR_BRIDGE_TEMP_HYS)
	        return;

	    SET_STATE(STATE_STOP);
	}


	/* Motor over-temp handling */
	if(cfg->motorNTCEnabled)
	{
		float motorTemp = ADC_GetMotorTemp();
		if (motorTemp > cfg->motorOverTempLimit)
		{
		    SET_STATE(STATE_MOTOR_THERMAL_SHUTDOWN);
		    MOTOR_Stop();
		    return;
		}
		else if (IS_STATE(STATE_MOTOR_THERMAL_SHUTDOWN))
		{
		    if (motorTemp > cfg->motorOverTempLimit - cfg->motorTempHys)
		        return;

		    SET_STATE(STATE_STOP);
		}
	}

	// Control loop
    if ((now - lastConversionTime) >= MOTOR_MAIN_LOOP_PERIOD)
    {
    	lastConversionTime = now;
    	if( IS_MODE(MODE_POSITION) && !IS_STATE(STATE_STOP_POSITION_REACHED))
    	{
    		pidInput = ENCODER_GetAbsolutePosition();

			PID_Compute(&mPID);

			uint8_t shouldStop = cfg->positionStopOnTarget &&
			                  (pidInput > pidTarget - cfg->positionTolerance) &&
			                  (pidInput < pidTarget + cfg->positionTolerance);

			int32_t pwmA = 0;
			int32_t pwmB = 0;

			if (!shouldStop)
			{
			    pwmA = (pidOutput > 0) ? pidOutput : 0;
			    pwmB = (pidOutput < 0) ? -pidOutput : 0;
			    MOTOR_SetPWM(pwmA, pwmB);
			}
			else
			{
				SET_STATE(STATE_STOP_POSITION_REACHED);
				MOTOR_Stop();
			}




    	}
    	else if( IS_MODE(MODE_VELOCITY) )
		{
			pidInput = ENCODER_GetRPS();

			PID_Compute(&mPID);

			int32_t pwmA = (pidOutput > 0) ? pidOutput : 0;
			int32_t pwmB = (pidOutput < 0) ? -pidOutput : 0;

			MOTOR_SetPWM(pwmA, pwmB);

		}
    	else if(IS_MODE(MODE_DUMB))
		{
    		// Main loop not needed, control set in function call
		}



    }
}

void MOTOR_Stop()
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

void MOTOR_SetPWM(uint16_t pwmA, uint16_t pwmB)
{
	if(pwmA != 0 && pwmB != 0)	// Invalid input
		return;

	if (cfg->motorInvertControl)
	{
		if(pwmA == 0) SET_STATE(STATE_RUNNING_CW);
		else SET_STATE(STATE_RUNNING_CCW);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmA);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwmB);
	}
	else
	{
		if(pwmB == 0) SET_STATE(STATE_RUNNING_CW);
		else SET_STATE(STATE_RUNNING_CCW);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmB);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwmA);

	}
}

