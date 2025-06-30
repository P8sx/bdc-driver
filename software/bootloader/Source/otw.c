/*
 * owt.c
 *
 *  Created on: Jun 15, 2025
 *      Author: phsx
 */

#include <otw.h>
#include "stm32g0xx_hal.h"
#include <string.h>

__attribute__((aligned(8))) uint8_t newApp[APP_SIZE] = {0};

extern IWDG_HandleTypeDef hiwdg;

HAL_StatusTypeDef OTW_FLASH_EraseFlash(void)
{
	HAL_StatusTypeDef eraseError = HAL_ERROR;

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = APP_START_PAGE;
	EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.NbPages = APP_NUMBER_OF_PAGES;

	eraseError = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	return eraseError;
}

void OTW_FLASH_WritePageToRAM(uint8_t* flashPage, uint8_t flashPageNumber)
{
	if (flashPageNumber >= APP_NUMBER_OF_PAGES || flashPage == NULL) {
	        return;
	}
	memcpy(&newApp[flashPageNumber * 2048], flashPage, 2048);
}

HAL_StatusTypeDef OTW_FLASH_Flash()
{
	HAL_StatusTypeDef flashError = HAL_ERROR;

    HAL_FLASH_Unlock();

    flashError = OTW_FLASH_EraseFlash();
    if(flashError != HAL_OK) {
    	HAL_FLASH_Lock();
        return flashError;
    }

    for (uint8_t flashPageNumber = 0; flashPageNumber < APP_NUMBER_OF_PAGES; flashPageNumber++)
    {
        uint32_t address = BOOT_APP_ADDRESS + (FLASH_PAGE_SIZE * flashPageNumber);


        if((address % 8) != 0) {
            flashError = HAL_ERROR;
            break;
        }

        for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i += 8)
        {
        	uint64_t data;
            memcpy(&data, newApp + (flashPageNumber * FLASH_PAGE_SIZE) + i, sizeof(uint64_t));

            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, data) != HAL_OK)
            {
                flashError = HAL_FLASH_GetError();
                break;
            }
        }

        if(flashError != HAL_OK) break;

        HAL_IWDG_Refresh(&hiwdg);
    }

    HAL_FLASH_Lock();
    return flashError;
}
