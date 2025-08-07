/*
 * config.c
 *
 *  Created on: Jun 8, 2025
 *      Author: phsx
 */

#include "config.h"
#include <stdbool.h>
#include "adc.h"
#include "motor.h"
#include "string.h"

#define ENTER_CRITICAL()    __disable_irq()
#define EXIT_CRITICAL()     __enable_irq()

driverConfig_t driverConfig;
//driverConfig_t driverConfig = {
//		.pidKp = 5,
//		.pidKi = 4,
//		.pidKd = 0.04,
//		.positionTolerance = 30,
//		.positionStopOnTarget = true,
//		.motorNTCEnabled = false,
//		.bridgeCurrentLimit = 1.5,
//};
const driverConfig_t* cfg = &driverConfig;
extern I2C_HandleTypeDef hi2c2;


void CONFIG_Save()
{
	uint8_t *pData = (uint8_t*)&driverConfig;
	uint16_t configSize = sizeof(driverConfig);

	for (uint16_t i = 0; i < configSize; i += EEPROM_PAGE_SIZE)
	{
		uint16_t chunkSize = (configSize - i > EEPROM_PAGE_SIZE) ? EEPROM_PAGE_SIZE : (configSize - i);
		HAL_I2C_Mem_Write(&hi2c2, EEPROM_I2C_ADDRESS, EEPROM_CONFIG_ADDRESS + i, I2C_MEMADD_SIZE_8BIT, &pData[i], chunkSize, HAL_MAX_DELAY);
		HAL_Delay(5);
	}
}

void CONFIG_Load()
{
	uint8_t receiveBuffer[sizeof(driverConfig)];
	HAL_I2C_Mem_Read(&hi2c2, EEPROM_I2C_ADDRESS, EEPROM_CONFIG_ADDRESS, I2C_MEMADD_SIZE_8BIT, receiveBuffer, sizeof(driverConfig), HAL_MAX_DELAY);
	memcpy(&driverConfig, receiveBuffer, sizeof(driverConfig));
}

void CONFIG_SetMotorInvertControl(uint8_t enabled) {
    ENTER_CRITICAL();
    driverConfig.motorInvertControl = enabled;
    EXIT_CRITICAL();
    CONFIG_Save();
}

void CONFIG_SetMotorTempHys(float hys) {
    ENTER_CRITICAL();
    driverConfig.motorTempHys = hys;
    EXIT_CRITICAL();
    CONFIG_Save();
}

void CONFIG_SetMotorOverTempLimit(float limit) {
    ENTER_CRITICAL();
    driverConfig.motorOverTempLimit = limit;
    EXIT_CRITICAL();
    CONFIG_Save();
}

void CONFIG_SetMotorNTCEnabled(uint8_t enabled) {
    ENTER_CRITICAL();
    driverConfig.motorNTCEnabled = enabled;
    EXIT_CRITICAL();
    CONFIG_Save();
}

void CONFIG_SetBridgeCurrentLimit(float limit) {
    ENTER_CRITICAL();
    driverConfig.bridgeCurrentLimit = limit;
    EXIT_CRITICAL();
    ADC_UpdateBridgeCurrentLimit();
    CONFIG_Save();
}

void CONFIG_SetBridgeSWCurrentLimit(float limit) {
    ENTER_CRITICAL();
    driverConfig.bridgeSWCurrentLimit = limit;
    EXIT_CRITICAL();
    CONFIG_Save();
}

void CONFIG_SetBridgeSWCurrentLimitTime(uint16_t timeMs) {
    ENTER_CRITICAL();
    driverConfig.bridgeSWCurrentLimitTime = timeMs;
    EXIT_CRITICAL();
    CONFIG_Save();
}

void CONFIG_SetPIDKp(float kp) {
    ENTER_CRITICAL();
    driverConfig.pidKp = kp;
    EXIT_CRITICAL();
    MOTOR_PIDUpdateTunings();
    CONFIG_Save();
}

void CONFIG_SetPIDKd(float kd) {
    ENTER_CRITICAL();
    driverConfig.pidKd = kd;
    EXIT_CRITICAL();
    MOTOR_PIDUpdateTunings();
    CONFIG_Save();
}

void CONFIG_SetPIDKi(float ki) {
    ENTER_CRITICAL();
    driverConfig.pidKi = ki;
    EXIT_CRITICAL();
    MOTOR_PIDUpdateTunings();
    CONFIG_Save();
}

void CONFIG_SetPositionTolerance(uint16_t tolerance) {
    ENTER_CRITICAL();
    driverConfig.positionTolerance = tolerance;
    EXIT_CRITICAL();
    CONFIG_Save();
}

void CONFIG_SetPositionStopOnTarget(uint8_t stop) {
    ENTER_CRITICAL();
    driverConfig.positionStopOnTarget = stop;
    EXIT_CRITICAL();
    CONFIG_Save();
}

void CONFIG_SetEndstopCWEnabled(uint8_t enabled) {
    ENTER_CRITICAL();
    driverConfig.endstopCWEnabled = enabled;
    EXIT_CRITICAL();
    CONFIG_Save();
}

void CONFIG_SetEndstopCCWEnabled(uint8_t enabled) {
    ENTER_CRITICAL();
    driverConfig.endstopCCWEnabled = enabled;
    EXIT_CRITICAL();
    CONFIG_Save();
}
