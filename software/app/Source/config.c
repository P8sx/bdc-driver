/*
 * config.c
 *
 *  Created on: Jun 8, 2025
 *      Author: phsx
 */

#include "config.h"
#include <stdbool.h>

#define ENTER_CRITICAL()    __disable_irq()
#define EXIT_CRITICAL()     __enable_irq()

driverConfig_t driverConfig = {
		.motorControlMode = 0,
		.pidKp = 5,
		.pidKi = 4,
		.pidKd = 0.04,
		.positionTolerance = 30,
		.positionStopOnTarget = true,
		.motorNTCEnabled = false,
		.bridgeCurrentLimit = 1.5,
};
const driverConfig_t* cfg = &driverConfig;


void CONFIG_SetMotorInvertControl(uint8_t enabled) {
    ENTER_CRITICAL();
    driverConfig.motorInvertControl = enabled;
    EXIT_CRITICAL();
}

void CONFIG_SetMotorControlMode(uint8_t mode) {
    ENTER_CRITICAL();
    driverConfig.motorControlMode = mode;
    EXIT_CRITICAL();
}

void CONFIG_SetMotorTempHys(float hys) {
    ENTER_CRITICAL();
    driverConfig.motorTempHys = hys;
    EXIT_CRITICAL();
}

void CONFIG_SetMotorOverTempLimit(float limit) {
    ENTER_CRITICAL();
    driverConfig.motorOverTempLimit = limit;
    EXIT_CRITICAL();
}

void CONFIG_SetMotorNTCEnabled(uint8_t enabled) {
    ENTER_CRITICAL();
    driverConfig.motorNTCEnabled = enabled;
    EXIT_CRITICAL();
}

void CONFIG_SetBridgeCurrentLimit(float limit) {
    ENTER_CRITICAL();
    driverConfig.bridgeCurrentLimit = limit;
    EXIT_CRITICAL();
}

void CONFIG_SetBridgeOverCurrentLimit(float limit) {
    ENTER_CRITICAL();
    driverConfig.bridgeOverCurrentLimit = limit;
    EXIT_CRITICAL();
}

void CONFIG_SetBridgeOverCurrentLimitTime(uint16_t timeMs) {
    ENTER_CRITICAL();
    driverConfig.bridgeOverCurrentLimitTime = timeMs;
    EXIT_CRITICAL();
}

void CONFIG_SetPidKp(float kp) {
    ENTER_CRITICAL();
    driverConfig.pidKp = kp;
    EXIT_CRITICAL();
}

void CONFIG_SetPidKd(float kd) {
    ENTER_CRITICAL();
    driverConfig.pidKd = kd;
    EXIT_CRITICAL();
}

void CONFIG_SetPidKi(float ki) {
    ENTER_CRITICAL();
    driverConfig.pidKi = ki;
    EXIT_CRITICAL();
}

void CONFIG_SetPositionTolerance(uint16_t tolerance) {
    ENTER_CRITICAL();
    driverConfig.positionTolerance = tolerance;
    EXIT_CRITICAL();
}

void CONFIG_SetPositionStopOnTarget(uint8_t stop) {
    ENTER_CRITICAL();
    driverConfig.positionStopOnTarget = stop;
    EXIT_CRITICAL();
}

void CONFIG_SetEndstopCWEnabled(uint8_t enabled) {
    ENTER_CRITICAL();
    driverConfig.endstopCWEnabled = enabled;
    EXIT_CRITICAL();
}

void CONFIG_SetEndstopCCWEnabled(uint8_t enabled) {
    ENTER_CRITICAL();
    driverConfig.endstopCCWEnabled = enabled;
    EXIT_CRITICAL();
}
