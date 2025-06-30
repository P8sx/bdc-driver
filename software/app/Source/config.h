/*
 * config.h
 *
 *  Created on: Jun 8, 2025
 *      Author: phsx
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/* -------------------------------------- Includes --------------------------------------*/
#include "main.h"

/* ---------------------------------- ADC CONFIG START ----------------------------------*/
#define ADC_RESOLUTION     			4095.0
#define ADC_VREF        			3.3
#define ADC_DAC_VERF				3.3
#define ADC_DAC_RESOLUTION     		4095.0

#define ADC_TH_R_FIXED     			10000.0
#define ADC_TH_BETA        			3950.0
#define ADC_TH_T0          			298.15
#define ADC_TH_R0          			10000.0

#define ADC_SHUNT_VALUE				0.020
#define ADC_SHUNT_AOUT_GAIN 		10.0

#define ADC_VIN_R1 					200000.0
#define ADC_VIN_R2					15000.0

/* ----------------------------------- MOTOR CONFIG  ------------------------------------*/
#define MOTOR_PID_PWM_LIMIT			2047
#define MOTOR_MAIN_LOOP_PERIOD		10		// ms

#define MOTOR_BRIDGE_TEMP_LIMIT		70.0	// C
#define MOTOR_BRIDGE_TEMP_HYS		10.0	// C

/* -------------------------------------- CONFIG  ---------------------------------------*/
#define EEPROM_I2C_ADDRESS						0x50<<1
/* EEPROM Data addresses */
#define EEPROM_MODBUS_ADDRESS					0x00
#define EEPROM_MODBUS_BAUD						0x01
#define EEPROM_MODBUS_DATA_BITS					0x02
#define EEPROM_MODBUS_STOP_BITS					0x03
#define EEPROM_MODBUS_PARITY					0x04



typedef struct {
	uint8_t  motorInvertControl;
	uint8_t  motorControlMode;				// 0 - dumb mode(SPEED/DIR controlled by COM) , 1 - SERVO mode, 2 - VELOCITY mode

	float	 motorTempHys;					// C - over temperature hysteresis
	float 	 motorOverTempLimit;			// C - over temperature limit
	uint8_t  motorNTCEnabled;				// 0 - disabled, 1 - enabled

	float	 bridgeCurrentLimit;			// A - Current limit set by DAC on ILIM pin
	float 	 bridgeOverCurrentLimit;		// A - OVERCURRENT limit monitored by MCU
	uint16_t bridgeOverCurrentLimitTime;	// ms - OVERCURRENT timeout

	float 	 pidKp;
	float 	 pidKd;
	float 	 pidKi;

	uint16_t positionTolerance;
	uint8_t  positionStopOnTarget;

	uint8_t  endstopCWEnabled;
	uint8_t  endstopCCWEnabled;

} driverConfig_t;

extern const driverConfig_t* cfg;


void CONFIG_SetMotorInvertControl(uint8_t enabled);
void CONFIG_SetMotorControlMode(uint8_t mode);
void CONFIG_SetMotorTempHys(float hys);
void CONFIG_SetMotorOverTempLimit(float limit);
void CONFIG_SetMotorNTCEnabled(uint8_t enabled);
void CONFIG_SetBridgeCurrentLimit(float limit);
void CONFIG_SetBridgeOverCurrentLimit(float limit);
void CONFIG_SetBridgeOverCurrentLimitTime(uint16_t timeMs);
void CONFIG_SetPidKp(float kp);
void CONFIG_SetPidKd(float kd);
void CONFIG_SetPidKi(float ki);
void CONFIG_SetPositionTolerance(uint16_t tolerance);
void CONFIG_SetPositionStopOnTarget(uint8_t stop);
void CONFIG_SetEndstopCWEnabled(uint8_t enabled);
void CONFIG_SetEndstopCCWEnabled(uint8_t enabled);

#endif /* CONFIG_H_ */
