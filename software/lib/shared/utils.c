/*
 * utils.c
 *
 *  Created on: Jun 18, 2025
 *      Author: phsx
 */

#include "utils.h"
#include <stdbool.h>
#include "shared.h"

static uint8_t beepActive = 0;
static uint8_t beepRemaining = 0;
static uint32_t beepInterval = 0;
static uint32_t beepLastTime = 0;
static uint8_t beepState = 0;
static uint8_t delayedResetFlag = false;

extern uint32_t *bootloaderFlag;


#ifdef BOOTLOADER

#elif defined(APP)

#endif



void BEEP_IT(uint32_t interval_ms, uint8_t count)
{
    if (!beepActive && count > 0) {
        beepActive = 1;
        beepRemaining = count * 2;
        beepInterval = interval_ms;
        beepLastTime = HAL_GetTick();
        beepState = 0;
    }
}
void BEEP(uint32_t interval_ms, uint8_t count)
{
	for(uint8_t i = 0; i < count; i++)
	{
		HAL_GPIO_WritePin(UTILS_BEEP_GPIO_Port, UTILS_BEEP_Pin, GPIO_PIN_SET);
		HAL_Delay(interval_ms);
		HAL_GPIO_WritePin(UTILS_BEEP_GPIO_Port, UTILS_BEEP_Pin, GPIO_PIN_RESET);
		HAL_Delay(interval_ms);
	}
}

void UTILS_UpdateBootFlag(uint32_t bootFlag)
{
	*bootloaderFlag = bootFlag;
}

void UTILS_DelayedReset()
{
	delayedResetFlag = true;
}

void UTILS_MainLoop()
{
    uint32_t now = HAL_GetTick();
	// Delayed reset handling
	if(delayedResetFlag == true)
	{
		static uint8_t delayStart = false;
		static uint32_t delay = 0;

		if(delayStart == false)
		{
			delayStart = true;
			delay = now;
		}
		else if(now - delay > 1000 && delayStart == true)
		{
			NVIC_SystemReset();
		}
	}
	// Non blocking BEEP handling
    if (beepActive) {
        if (HAL_GetTick() - beepLastTime >= beepInterval) {
            beepLastTime = now;

            beepState = !beepState;
            HAL_GPIO_WritePin(UTILS_BEEP_GPIO_Port, UTILS_BEEP_Pin, beepState ? GPIO_PIN_SET : GPIO_PIN_RESET);

            beepRemaining--;
            if (beepRemaining == 0) {
                beepActive = 0;
                HAL_GPIO_WritePin(UTILS_BEEP_GPIO_Port, UTILS_BEEP_Pin, GPIO_PIN_RESET);
            }
        }
    }

#ifdef BOOTLOADER

	/* HEARTBEAT Signal for bootloader */
	static uint32_t lastBeepTime = 0;
	static uint8_t beepState = 0;

    if ((now - lastBeepTime) >= 2000 && beepState == 0)
    {
        HAL_GPIO_WritePin(UTILS_BEEP_GPIO_Port, UTILS_BEEP_Pin, GPIO_PIN_SET);
        lastBeepTime = now;
        beepState = 1;
    }
    else if ((now - lastBeepTime) >= 10 && beepState == 1)
    {
        HAL_GPIO_WritePin(UTILS_BEEP_GPIO_Port, UTILS_BEEP_Pin, GPIO_PIN_RESET);
        beepState = 0;
    }
#elif defined(APP)

	/* If 5 seconds passed since reboot and OTW flag has not been set by OTW master, reset to BOOTLOADER  */
	if(*bootloaderFlag == BOOTFLAG_APP_OTW && HAL_GetTick() > 5000)
	{
	  *bootloaderFlag = BOOTFLAG_BOOTLOADER;
	  NVIC_SystemReset();
	}

	/* HEARTBEAT Signal for APP */
	static uint32_t lastBlinkTime = 0;
	static uint8_t ledState = 0;

    if ((now - lastBlinkTime) >= 1000 && ledState == 0)
    {
        HAL_GPIO_WritePin(UTILS_LED_GPIO_Port, UTILS_LED_Pin, GPIO_PIN_SET);
        lastBlinkTime = now;
        ledState = 1;
    }
    else if ((now - lastBlinkTime) >= 100 && ledState == 1)
    {
        HAL_GPIO_WritePin(UTILS_LED_GPIO_Port, UTILS_LED_Pin, GPIO_PIN_RESET);
        ledState = 0;
    }
#endif
}

