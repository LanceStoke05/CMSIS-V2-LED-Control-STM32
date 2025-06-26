/*
 * funcs.h
 *
 *  Created on: Jun 20, 2025
 *      Author: piski
 */

#ifndef INC_FUNCS_H_
#define INC_FUNCS_H_

#include "main.h"

void Task_1_UART(void * arguments)
{
	uint8_t pData;
	for(;;){
		if(HAL_UART_Receive(&huart3, &pData, 1, HAL_MAX_DELAY) == HAL_OK){
			if(osMutexAcquire(countMutex, osWaitForever)==osOK)
			{
				if((pData == 0) || (pData == 1) || (pData == 2)) {mode = (int) pData;}
				osMutexRelease(countMutex);
			}
		}
	}
}

void Task_2_Flash(void * arguments)
{
	for(;;)
	{
		if(osMutexAcquire(modeMutex, osWaitForever)==osOK)
		{
			if(osMutexAcquire(countMutex, osWaitForever)==osOK)
			{
				HAL_FLASH_Unlock();
				HAL_FLASHEx_Erase(pEraseInit, SectorError);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr_start, (uint32_t)count);
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (flash_addr_start + sizeof(uint32_t)), (uint32_t)mode);
				HAL_FLASH_Lock();
				osMutexRelease(countMutex);
				osMutexRelease(modeMutex);
				osDelay(1000);
			}
		}
	}
}

void Task_3_LED(void * arguments)
{
	int count1;
	for(;;)
	{
		if(osMutexAcquire(modeMutex, osWaitForever)== osOK)
		{
			if (mode == 0)
			{
				osMutexRelease(modeMutex);
				HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
			}
			else if (mode == 1) {
				osMutexRelease(modeMutex);
				HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
			}
			else
			{
				osMutexRelease(modeMutex);
				if(osMutexAcquire(countMutex, osWaitForever)== osOK)
				{
					count1 = count;
					osMutexRelease(countMutex);
					osDelay(count1);
					HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
				}
			}
		}
	}
}

#endif /* INC_FUNCS_H_ */
