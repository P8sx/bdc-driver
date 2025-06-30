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
#include "otw.h"

uint8_t  flashPage[2048];


uint8_t  flashPageNumber = 0;
uint8_t  flashPerformUpdate = false;


uint32_tU flashPageCRC;


extern CRC_HandleTypeDef hcrc;
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
  //TbxMbServerSetCallbackReadInput(modbusServer, AppReadCoil);

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

 if(flashPerformUpdate)											// Check if COM requested OTW update
 {
	 flashPerformUpdate = false;
	 if(OTW_FLASH_Flash() == HAL_OK)							// Flash and reset to APP
	 {
		 UTILS_UpdateBootFlag(BOOTFLAG_APP_OTW);
		 UTILS_DelayedReset();
		 BEEP(100,2);
	 }
	 else{
		 BEEP(1000,3);
	 }
 }

}

/*****************************************************************************************************************************
														HELPER FUNCTIONS
 *****************************************************************************************************************************/


/*****************************************************************************************************************************
														REG CALLBACKS
 *****************************************************************************************************************************/




tTbxMbServerResult AppWriteReg(tTbxMbServer channel, uint16_t addr, uint16_t value)
{
  static uint16_t flashPageBytesCount = 0;
  tTbxMbServerResult result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;


  if (HOLDING_REG_FLASH_PAGE_NUMBER == addr)
  {
	  flashPageNumber = value;
	  result = TBX_MB_SERVER_OK;
  }
  else if (addr >= HOLDING_REG_FLASH_PAGE_START_ADDR && addr < HOLDING_REG_FLASH_PAGE_START_ADDR + HOLDING_REG_FLASH_PAGE_SIZE)
  {
	  uint16_t index = (addr - HOLDING_REG_FLASH_PAGE_START_ADDR) * 2;
	  uint8_tU data = {.u16 = value};

	  flashPage[index + 1]  = data.d[0];
	  flashPage[index] 		= data.d[1];

	  flashPageBytesCount += 2;
	  result = TBX_MB_SERVER_OK;
  }



  if((HOLDING_REG_FLASH_PAGE_START_ADDR + HOLDING_REG_FLASH_PAGE_SIZE) == addr + 1 && flashPageBytesCount == sizeof(flashPage))
  {
	  flashPageBytesCount = 0;
	  flashPageCRC.d = HAL_CRC_Calculate(&hcrc, (uint32_t *)flashPage, sizeof(flashPage));
	  OTW_FLASH_WritePageToRAM(flashPage, flashPageNumber);
  }
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

  else if(addr >= INPUT_REG_VERSION && addr < INPUT_REG_VERSION + INPUT_REG_VERSION_SIZE)
  {
	  const uint8_t buildVersion[INPUT_REG_VERSION_SIZE] = BUILD_VERSION;
	  *value = buildVersion[addr - INPUT_REG_VERSION];
	  result = TBX_MB_SERVER_OK;
  }

  else if (INPUT_REG_FLASH_PAGE_CRC32 == addr)
  {
	  *value = flashPageCRC.u16[1];
	  result = TBX_MB_SERVER_OK;
  }
  else if (INPUT_REG_FLASH_PAGE_CRC32 + 1 == addr)
  {
	  *value = flashPageCRC.u16[0];
	  result = TBX_MB_SERVER_OK;
	  flashPageCRC.d = 0;

  }
	return result;
}





tTbxMbServerResult AppWriteCoil(tTbxMbServer channel, uint16_t addr, uint8_t value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
  if(COIL_REG_JUMP == addr)
  {
	  if(value == TBX_ON)
	  {
		  UTILS_UpdateBootFlag(BOOTFLAG_APP);
		  UTILS_DelayedReset();
		  return TBX_MB_SERVER_OK;
	  }
  }

  if (COIL_REG_OTW_PERFORM_UPDATE == addr)
  {
	 flashPerformUpdate = true;
     result = TBX_MB_SERVER_OK;
  }

  return result;
}




tTbxMbServerResult AppReadCoil(tTbxMbServer channel, uint16_t addr, uint8_t* value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
  if(COIL_REG_JUMP == addr)
  {
	  *value = TBX_OFF;
	  result = TBX_MB_SERVER_OK;
  }
  return result;
}

