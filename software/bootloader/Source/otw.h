/*
 * owt.h
 *
 *  Created on: Jun 15, 2025
 *      Author: phsx
 */

#ifndef OTW_H_
#define OTW_H_

#include "main.h"
#include "shared.h"


#define APP_START_PAGE 			(BOOT_APP_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE
#define APP_NUMBER_OF_PAGES		((FLASH_BASE + FLASH_SIZE - BOOT_APP_ADDRESS) / FLASH_PAGE_SIZE)
#define APP_SIZE				(20 * 2048)

void OTW_FLASH_WritePageToRAM(uint8_t* flashPage, uint8_t flashPageNumber);
HAL_StatusTypeDef OTW_FLASH_EraseFlash(void);
HAL_StatusTypeDef OTW_FLASH_Flash();

#endif /* OTW_H_ */
