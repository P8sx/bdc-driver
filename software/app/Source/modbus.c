/*
 * com.c
 *
 *  Created on: Jun 15, 2025
 *      Author: phsx
 */

#include "modbus.h"
#include <string.h>
#include "utils.h"
#include "shared.h"
#include "main.h"
#include <stdbool.h>
#include "utils.h"
#include "config.h"
#include "adc.h"
#include "motor.h"
#include "enc.h"

uint8_t OTWRequestFlag = false;

extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart3;

void COM_Init(void)
{
  modbusTp = TbxMbRtuCreate(DEFAULT_MODBUS_BOOTLOADER_ADDRESS, TBX_MB_UART_PORT3, TBX_MB_UART_19200BPS, TBX_MB_UART_1_STOPBITS, TBX_MB_NO_PARITY);
  modbusServer = TbxMbServerCreate(modbusTp);

  // binary write/read
  TbxMbServerSetCallbackWriteCoil(modbusServer, AppWriteCoil);
  TbxMbServerSetCallbackReadCoil(modbusServer, AppReadCoil);
  // binary read-only
  TbxMbServerSetCallbackReadInput(modbusServer, AppReadCoil);

  // 16bit write/read
  TbxMbServerSetCallbackReadHoldingReg(modbusServer, AppReadReg);
  TbxMbServerSetCallbackWriteHoldingReg(modbusServer, AppWriteReg);
  // 16bit read-only
  TbxMbServerSetCallbackReadInputReg(modbusServer, AppReadReg);

  // TbxMbServerSetCallbackCustomFunction(modbusServer, AppWriteCoil);
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_UART_Receive_IT(&huart3, &rxByte, 1);
}

/*****************************************************************************************************************************
                            MAIN LOOP
 *****************************************************************************************************************************/

void COM_MainLoop(void)
{
  TbxMbEventTask();
}

/*****************************************************************************************************************************
                            HELPER FUNCTIONS
 *****************************************************************************************************************************/

uint8_t COM_GetOTWRequest()
{
  return OTWRequestFlag;
}

/*****************************************************************************************************************************
                            REG CALLBACKS
 *****************************************************************************************************************************/

tTbxMbServerResult AppWriteReg(tTbxMbServer channel, uint16_t addr, uint16_t value)
{
  static floatU motorTempHys;
  static floatU motorOverTempLimit;
  static floatU bridgeCurrentLimit;
  static floatU bridgeOverCurrentLimit;
  static floatU kp, kd, ki;
  static int64_tU motorSetPosition;

  tTbxMbServerResult result = TBX_MB_SERVER_OK;
  switch (addr)
  {
    case HOLDING_REG_SET_MOTOR_RPS:
      MOTOR_SetVelocity((int16_t)value);
      break;

    case HOLDING_REG_SET_POSITION:
      motorSetPosition.u16[0] = value;
      break;
    case HOLDING_REG_SET_POSITION +1:
	  motorSetPosition.u16[1] = value;
      break;
    case HOLDING_REG_SET_POSITION + 2:
	  motorSetPosition.u16[2] = value;
      break;
    case HOLDING_REG_SET_POSITION + 3:
	  motorSetPosition.u16[3] = value;
      MOTOR_SetPosition(motorSetPosition.d);
      motorSetPosition.d = 0;
      break;

    case HOLDING_REG_SET_MOTOR_PWMA:
      MOTOR_SetDirection(value, 0);
      break;

    case HOLDING_REG_SET_MOTOR_PWMB:
      MOTOR_SetDirection(0, value);
      break;

    case HOLDING_REG_MOTOR_TEMP_HYS:
      motorTempHys.u16[0] = value;
      break;
    case HOLDING_REG_MOTOR_TEMP_HYS + 1:
      motorTempHys.u16[1] = value;
      CONFIG_SetMotorTempHys(motorTempHys.d);
      break;

    case HOLDING_REG_MOTOR_OVER_TEMP_LIMIT:
      motorOverTempLimit.u16[0] = value;
      break;
    case HOLDING_REG_MOTOR_OVER_TEMP_LIMIT + 1:
      motorOverTempLimit.u16[1] = value;
      CONFIG_SetMotorOverTempLimit(motorOverTempLimit.d);
      break;

    case HOLDING_REG_BRIDGE_CURRENT_LIMIT:
      bridgeCurrentLimit.u16[0] = value;
      break;
    case HOLDING_REG_BRIDGE_CURRENT_LIMIT + 1:
      bridgeCurrentLimit.u16[1] = value;
      CONFIG_SetBridgeCurrentLimit(bridgeCurrentLimit.d);
      break;

    case HOLDING_REG_BRIDGE_SWCURRENT_LIMIT:
      bridgeOverCurrentLimit.u16[0] = value;
      break;
    case HOLDING_REG_BRIDGE_SWCURRENT_LIMIT + 1:
      bridgeOverCurrentLimit.u16[1] = value;
      CONFIG_SetBridgeSWCurrentLimit(bridgeOverCurrentLimit.d);
      break;
    
    case HOLDING_REG_BRIDGE_OVERCURRENT_TIME:
      CONFIG_SetBridgeSWCurrentLimitTime(value);
      break;
    
    case HOLDING_REG_PID_KP:
      kp.u16[0] = value;
      break; 
    case HOLDING_REG_PID_KP + 1:
      kp.u16[1] = value;
      CONFIG_SetPIDKp(kp.d);
      break;

    case HOLDING_REG_PID_KD:
      kd.u16[0] = value;
      break;  
    case HOLDING_REG_PID_KD + 1:
      kd.u16[1] = value;
      CONFIG_SetPIDKd(kd.d);
      break;

    case HOLDING_REG_PID_KI:
      ki.u16[0] = value;
      break;
    case HOLDING_REG_PID_KI + 1:
      ki.u16[1] = value;
      CONFIG_SetPIDKi(ki.d);
      break;
    
    case HOLDING_REG_POSITION_TOLERANCE:
      CONFIG_SetPositionTolerance(value);
      break;
      
    default:
      result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
      break;
  }

  return result;
}

tTbxMbServerResult AppReadReg(tTbxMbServer channel, uint16_t addr, uint16_t *value)
{
  static floatU motorTemp;
  static floatU bridgeTemp;
  static floatU voltage;
  static floatU kP, kI, kD;
  static int64_tU encoderAbsolutePosition;

  tTbxMbServerResult result = TBX_MB_SERVER_OK;
  switch (addr)
  {
  case INPUT_REG_BUILD_DATE ... INPUT_REG_BUILD_DATE + INPUT_REG_BUILD_DATE_SIZE:
    const uint8_t buildDatetime[INPUT_REG_VERSION_SIZE] = BUILD_DATETIME;
    *value = buildDatetime[addr - INPUT_REG_BUILD_DATE];
    break;

  case INPUT_REG_VERSION ... INPUT_REG_VERSION + INPUT_REG_VERSION_SIZE:
    const uint8_t buildVersion[INPUT_REG_VERSION_SIZE] = BUILD_VERSION;
    *value = buildVersion[addr - INPUT_REG_VERSION];
    break;

  case INPUT_REG_MOTOR_TEMP:
    motorTemp.d = ADC_GetMotorTemp();
    *value = motorTemp.u16[0];
    break;

  case INPUT_REG_MOTOR_TEMP + 1:
    *value = motorTemp.u16[1];
    motorTemp.d = 0;
    break;

  case INPUT_REG_BRIDGE_TEMP:
    bridgeTemp.d = ADC_GetBridgeTemp();
    *value = bridgeTemp.u16[0];
    break;

  case INPUT_REG_BRIDGE_TEMP + 1:
    *value = bridgeTemp.u16[1];
    bridgeTemp.d = 0;
    break;

  case INPUT_REG_VOLTAGE:
    voltage.d = ADC_GetVoltage();
    *value = voltage.u16[0];
    break;

  case INPUT_REG_VOLTAGE + 1:
    *value = voltage.u16[1];
    voltage.d = 0;
    break;

  case INPUT_REG_ENC_RPS:
    *value = ENCODER_GetRPS();
    break;

  case INPUT_REG_ENC_ABSOLUTE_POSITION:
    encoderAbsolutePosition.d = ENCODER_GetAbsolutePosition();
    *value = encoderAbsolutePosition.u16[0];
    break;

  case INPUT_REG_ENC_ABSOLUTE_POSITION + 1:
    *value = encoderAbsolutePosition.u16[1];
    break;

  case INPUT_REG_ENC_ABSOLUTE_POSITION + 2:
    *value = encoderAbsolutePosition.u16[2];
    break;

  case INPUT_REG_ENC_ABSOLUTE_POSITION + 3:
    *value = encoderAbsolutePosition.u16[3];
    encoderAbsolutePosition.d = 0;
    break;

  case HOLDING_REG_PID_KP:
    kP.d = cfg->pidKp;
    *value = kP.u16[0];
    break;
  case HOLDING_REG_PID_KP + 1:
  	*value = kP.u16[1];
    kP.d = 0;
    break;

  case HOLDING_REG_PID_KD:
	kD.d = cfg->pidKd;
	*value = kD.u16[0];
    break;
  case HOLDING_REG_PID_KD + 1:
	*value = kD.u16[1];
	kD.d = 0;
    break;

  case HOLDING_REG_PID_KI:
	kI.d = cfg->pidKi;
	*value = kI.u16[0];
    break;
  case HOLDING_REG_PID_KI + 1:
	*value = kI.u16[1];
	kI.d = 0;
    break;
  default:
    result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
  }
  return result;
}

tTbxMbServerResult AppWriteCoil(tTbxMbServer channel, uint16_t addr, uint8_t value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_OK;
  switch (addr)
  {
  case COIL_REG_JUMP:
    if (value == TBX_OFF)
    {
      UTILS_UpdateBootFlag(BOOTFLAG_BOOTLOADER);
      UTILS_DelayedReset();
    }
    break;

  case COIL_REG_VALIDATE_UPDATE:
    UTILS_UpdateBootFlag(BOOTFLAG_APP);
    break;

  case COIL_REG_MOTOR_INVERT_CONTROL:
    CONFIG_SetMotorInvertControl(value);
    break;

  case COIL_REG_MOTOR_NTC_ENABLED:
    CONFIG_SetMotorNTCEnabled(value);
    break;

  case COIL_REG_POSITION_STOP_ON_TARGET:
    CONFIG_SetPositionStopOnTarget(value);
    break;

  case COIL_REG_ENDSTOP_CW_ENABLED:
    CONFIG_SetEndstopCWEnabled(value);
    break;

  case COIL_REG_ENDSTOP_CCW_ENABLED:
    CONFIG_SetEndstopCCWEnabled(value);
    break;

  case COIL_REG_ESTOP:
    MOTOR_EStop(value);
    break;
  case COIL_REG_ENC_ZERO:
	ENCODER_ResetAbsolutePosition();
	break;
  default:
    result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
  }
  return result;
}

tTbxMbServerResult AppReadCoil(tTbxMbServer channel, uint16_t addr, uint8_t *value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_OK;
  switch (addr)
  {
  case COIL_REG_JUMP:
    *value = TBX_ON;
    break;
  case INPUT_CONTACTS_MOTOR_OCLN:
	  *value = HAL_GPIO_ReadPin(M_OCLn_GPIO_Port, M_OCLn_Pin);
    break;
  default:
    result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
  }
  return result;
}
