/**
  ******************************************************************************
  * @file    uart_lib.h
  * @brief   This file contains the headers for the bluetooth methods.
	******************************************************************************
 **/

#ifndef UART_LIB_H
#define UART_LIB_H

#include <stdint.h>
#include <stdio.h>

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


void serPrint(UART_HandleTypeDef *haurt, const char* str);
void serPrintln(UART_HandleTypeDef *haurt, const char* str);
void serPrintHex(UART_HandleTypeDef *haurt, uint8_t *buf, uint8_t len);



#ifdef __cplusplus
}
#endif

#endif
/**********************************END OF FILE**********************************/
