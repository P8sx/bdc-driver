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


  //TbxMbServerSetCallbackCustomFunction(modbusServer, AppWriteCoil);
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_UART_Receive_IT(&huart3, &rxByte, 1);
}



/*****************************************************************************************************************************
														MAIN LOOP
 *****************************************************************************************************************************/

void COM_MainLoop(void){
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
  tTbxMbServerResult result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
  return result;
}

tTbxMbServerResult AppReadReg(tTbxMbServer channel, uint16_t addr, uint16_t *value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;

  if(addr >= INPUT_REG_BUILD_DATE && addr < INPUT_REG_BUILD_DATE + INPUT_REG_BUILD_DATE_SIZE)
  {
	  const uint8_t buildDatetime[INPUT_REG_VERSION_SIZE] = BUILD_DATETIME;
	  *value = buildDatetime[addr - INPUT_REG_BUILD_DATE];
	  result = TBX_MB_SERVER_OK;
  }

  if(addr >= INPUT_REG_VERSION && addr < INPUT_REG_VERSION + INPUT_REG_VERSION_SIZE)
  {
	  const uint8_t buildVersion[INPUT_REG_VERSION_SIZE] = BUILD_VERSION;
	  *value = buildVersion[addr - INPUT_REG_VERSION];
	  result = TBX_MB_SERVER_OK;
  }

	return result;
}





tTbxMbServerResult AppWriteCoil(tTbxMbServer channel, uint16_t addr, uint8_t value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
  if(COIL_REG_JUMP == addr)
  {
	  if(value == TBX_OFF)
	  {
		  UTILS_UpdateBootFlag(BOOTFLAG_BOOTLOADER);
		  UTILS_DelayedReset();
		  return TBX_MB_SERVER_OK;
	  }
  }

  else if (COIL_REG_VALIDATE_UPDATE == addr)
  {
	 UTILS_UpdateBootFlag(BOOTFLAG_APP);
     result = TBX_MB_SERVER_OK;
  }
  return result;
}




tTbxMbServerResult AppReadCoil(tTbxMbServer channel, uint16_t addr, uint8_t* value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
  if(COIL_REG_JUMP == addr)
  {
	  *value = TBX_ON;
	  result = TBX_MB_SERVER_OK;
  }
  return result;
}

