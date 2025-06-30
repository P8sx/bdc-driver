/*
 * utils.h
 *
 *  Created on: Jun 18, 2025
 *      Author: phsx
 */

#ifndef SHARED_UTILS_H_
#define SHARED_UTILS_H_

#include "main.h"

#define UTILS_BEEP_Pin			BUZZER_Pin
#define UTILS_BEEP_GPIO_Port 	BUZZER_GPIO_Port
#define UTILS_LED_Pin			LED_Pin
#define UTILS_LED_GPIO_Port 	LED_GPIO_Port

void BEEP(uint32_t interval_ms, uint8_t count);
void BEEP_IT(uint32_t interval_ms, uint8_t count);

void UTILS_UpdateBootFlag(uint32_t bootFlag);
void UTILS_DelayedReset();
void UTILS_MainLoop();

typedef union {
    double d;
    uint16_t u16[4];
} doubleU;

typedef union {
    float d;
    uint16_t u16[2];
} floatU;


typedef union {
    uint8_t d[2];
    uint16_t u16;
} uint8_tU;

typedef union {
    int8_t d[2];
    uint16_t u16;
} int8_tU;


typedef union {
    uint32_t d;
    uint16_t u16[2];
} uint32_tU;

typedef union {
    int32_t d;
    uint16_t u16[2];
} int32_tU;

typedef union {
    uint64_t d;
    uint16_t u16[4];
} uint64_tU;

typedef union {
	int64_t d;
    uint16_t u16[4];
} int64_tU;



#endif /* SHARED_UTILS_H_ */
