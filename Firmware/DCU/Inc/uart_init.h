/**
  ******************************************************************************
  * @file    uart_lib.h
  * @brief   This file contains the headers for the bluetooth methods.
	******************************************************************************
 **/

#ifndef UART_INIT_H
#define UART_INIT_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "UartRingBufferManager.h"

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32h7xx_hal.h"
#include "def.h"

void  MX_UART4_Init(uint32_t baudrate);
void  MX_UART5_Init(uint32_t baudrate);
void  MX_UART7_Init(uint32_t baudrate);
void  MX_UART8_Init(uint32_t baudrate);
void  MX_USART1_UART_Init(uint32_t baudrate);
void  MX_USART2_UART_Init(uint32_t baudrate);
void  MX_USART3_UART_Init(uint32_t baudrate);
void  MX_USART6_UART_Init(uint32_t baudrate);


#ifdef __cplusplus
}
#endif

#endif
/**********************************END OF FILE**********************************/
