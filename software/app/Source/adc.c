/*
 * adc.c
 *
 *  Created on: Jun 20, 2025
 *      Author: phsx
 */
#include "adc.h"
#include "math.h"
#include "config.h"
#include "motor.h"

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;

static volatile uint16_t adcResult[4] = {0};

float bridgeTemp = 0;
float motorTemp = 0;
float motorCurrent = 0;
float driverVoltage = 0;


float ADC_ToCelsius(uint16_t adc_value) {
    float voltage, rNTC, tempK, tempC;
    if (adc_value >= ADC_RESOLUTION)
        adc_value = ADC_RESOLUTION - 1;
    voltage = (adc_value / ADC_RESOLUTION) * ADC_VREF;
    rNTC = (voltage * ADC_TH_R_FIXED) / (ADC_VREF - voltage);
    tempK = 1.0 / ((1.0 / ADC_TH_T0) + (1.0 / ADC_TH_BETA) * log(rNTC / ADC_TH_R0));
    tempC = tempK - 273.15;
    return tempC;
}
float ADC_ToCurrent(uint16_t value)
{
	float voltage = ((float)value / ADC_RESOLUTION) * ADC_VREF;
	float current = voltage / (ADC_SHUNT_VALUE * ADC_SHUNT_AOUT_GAIN);
	return current;
}

float ADC_ToVoltage(uint16_t value)
{
    float voltage = ((float)value / ADC_RESOLUTION) * ADC_VREF;
    float vin = voltage * (1.0f + (ADC_VIN_R1 / ADC_VIN_R2));
    return vin;
}

int16_t DAC_ToRaw(float current)
{
	float vref = current * 10 * ADC_SHUNT_VALUE;
	uint16_t dac_value = (uint16_t)((vref / ADC_DAC_VERF) * ADC_DAC_RESOLUTION);
	return dac_value;
}

void ADC_Init()
{
	  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	  HAL_ADCEx_Calibration_Start(&hadc1);
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResult, 4);
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_ToRaw(cfg->bridgeCurrentLimit));
}

void ADC_MainLoop()
{
	uint32_t now = HAL_GetTick();
	static uint32_t lastConversionTime = 0;

    if ((now - lastConversionTime) >= 10 )
    {
    	bridgeTemp = ADC_ToCelsius(adcResult[ADC_NTC_BRIDGE]);
    	motorTemp = ADC_ToCelsius(adcResult[ADC_NTC_MOTOR]);
    	motorCurrent = ADC_ToCurrent(adcResult[ADC_MOTOR_IOUT]);
    	driverVoltage = ADC_ToVoltage(adcResult[ADC_VMOTOR]);

        lastConversionTime = now;
    }

}
void ADC_UpdateBridgeCurrentLimit()
{
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_ToRaw(cfg->bridgeCurrentLimit));
}

float ADC_GetMotorCurrent()
{
	if(HAL_GPIO_ReadPin(M_SLEEPn_GPIO_Port, M_SLEEPn_Pin) == GPIO_PIN_RESET)
		return 0;
	return motorCurrent;
}

float ADC_GetBridgeTemp()
{
	return bridgeTemp;
}
float ADC_GetMotorTemp()
{
	if(cfg->motorNTCEnabled)
		return motorTemp;
	return 0;
}
float ADC_GetVoltage()
{
	return driverVoltage;
}
