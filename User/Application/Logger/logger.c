/*
 * logger.c
 *
 *  Created on: 30.10.2019
 *      Author: tobing
 */

#include "logger.h"

uint8_t LOG_ENABLE=1;

uint16_t log_length;
char log_buffer[BUFFER_SIZE] = {0};

uint8_t send2UART(UART_HandleTypeDef *huart, const char* message, ...)
{
	if(!getLogEnable())
	{
		return HAL_OK;
	}

    va_list variables;
    uint8_t fails = 0;

    va_start(variables, message);
    log_length = vsnprintf(log_buffer, sizeof(log_buffer), message, variables);
    //log_length = vsprintf(log_buffer, message, variables);
    va_end(variables);

    fails += HAL_UART_Transmit(huart,log_buffer,log_length, HAL_MAX_DELAY) != HAL_OK;

    return fails;
}

uint8_t getLogEnable()
{
	return LOG_ENABLE;
}

void setLogEnable(uint8_t x)
{
	LOG_ENABLE = x;
}

uint8_t MemoryInit()
{
	HAL_GPIO_WritePin(SD_POWER_GPIO_Port, SD_POWER_Pin, GPIO_PIN_RESET);  // Power ON SD card
	HAL_Delay(500);
	SD_spi_handle = &hspi1;
	sdResult = f_mount(&fs_area, "", 1);
	  if( FR_OK != sdResult )
	  {
		  return 1;
	  }
	  else
	  {
		return 0;
	  }
}

uint8_t openLogFile()
{
	for(int fidx=0; fidx<1000; fidx++)
	{
		sprintf(filenameLog, "log%03d.csv", fidx);
		sdResult = f_open(&log_file, filenameLog, FA_WRITE | FA_CREATE_NEW);
		if( sdResult == FR_OK )
		{
			return fidx;
		}
	}
}

uint8_t send2Memory(const char* message, ...)
{

    va_list variables;
    uint8_t fails = 0;

    va_start(variables, message);


    memset(buffer, 0, sizeof(buffer));

    vsnprintf(buffer, sizeof(buffer), message, variables);

    va_end(variables);

	f_write(&log_file, buffer, sizeof(buffer)/sizeof(buffer[0]), &sd_bw_log);
	f_sync(&log_file);

    return 0; //temporary
}

uint8_t memoryInitialized()
{
	if(FR_OK == sdResult)
		return 1;
	else
		return 0;
}
