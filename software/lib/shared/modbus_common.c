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

uint8_t rxByte;
tTbxMbTp modbusTp = NULL;
tTbxMbServer modbusServer = NULL;


extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart3;


void COM_DeInit(void)
{
	if(modbusServer != NULL)
	{
		TbxMbServerFree(modbusServer);
	}

	if(modbusTp != NULL)
	{
		TbxMbRtuFree(modbusTp);
	}


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t errorCode = HAL_UART_GetError(&huart3);
  if ((errorCode & (HAL_UART_ERROR_NE|HAL_UART_ERROR_PE|HAL_UART_ERROR_FE)) == 0U)
  {
    TbxMbUartDataReceived(TBX_MB_UART_PORT3, &rxByte, 1U);
  }
  HAL_UART_Receive_IT(&huart3, &rxByte, 1U);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * handle)
{
  TbxMbUartTransmitComplete(TBX_MB_UART_PORT3);
}

uint8_t TbxMbPortUartTransmit(tTbxMbUartPort port, uint8_t const * data, uint16_t len)
{
  uint8_t result = TBX_ERROR;
  if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)data, len) == HAL_OK)
  {
    result = TBX_OK;
  }
  return result;
}

uint16_t TbxMbPortTimerCount(void)
{
	return (uint16_t)__HAL_TIM_GET_COUNTER(&htim16);
}
