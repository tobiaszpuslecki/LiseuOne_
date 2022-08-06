/*
 * logger.h
 *
 *  Created on: 30.10.2019
 *      Author: tobing
 */

#ifndef APPLICATION_LOGGER_LOGGER_H_
#define APPLICATION_LOGGER_LOGGER_H_


#define BUFFER_SIZE 128
#define LOG_UART &huart2

#include "fatfs.h"
#include "spi.h"
#include "usart.h"
#include <stdarg.h>


FATFS fs_area;
FIL log_file;
char buffer[BUFFER_SIZE];
char filenameLog[50];
UINT sd_bw_log;
FRESULT sdResult;


uint8_t send2UART(UART_HandleTypeDef *huart, const char* message, ...);
uint8_t getLogEnable();
void setLogEnable(uint8_t x);

uint8_t MemoryInit();
uint8_t openLogFile();
uint8_t send2Memory(const char* message, ...);
uint8_t memoryInitialized();


#endif /* APPLICATION_LOGGER_LOGGER_H_ */
