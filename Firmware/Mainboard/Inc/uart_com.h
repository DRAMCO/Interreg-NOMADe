/*  ____  ____      _    __  __  ____ ___
 * |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
 * | | | | |_) |  / _ \ | |\/| | |  | | | |
 * | |_| |  _ <  / ___ \| |  | | |__| |_| |
 * |____/|_| \_\/_/   \_\_|  |_|\____\___/
 *                           research group
 *                             dramco.be/
 *
 *  KU Leuven - Technology Campus Gent,
 *  Gebroeders De Smetstraat 1,
 *  B-9000 Gent, Belgium
 *
 *         File: UartRingBufferManager.h
 *      Created: 2020-02-27
 *       Author: Jarne Van Mulders
 *      Version: V1.0
 *
 *  Description: Firmware IMU sensor module for the NOMADe project
 *
 *  Interreg France-Wallonie-Vlaanderen NOMADe
 *
 */

#ifndef UART_COM_H_
#define UART_COM_H_

#include "stm32h7xx_hal.h"
#include "usb_com.h"
#include <string.h>
#include <stdio.h>
#include "UartRingbufferManager.h"



void UART_COM_write(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len);
/*-------------------------------------------------------------------------------------------------*/


uint8_t UART_COM_read(UART_HandleTypeDef *huart);
/*-------------------------------------------------------------------------------------------------*/


void UART_COM_print(UART_HandleTypeDef *huart, const char* str);
/*-------------------------------------------------------------------------------------------------*/


void UART_COM_print_ln(UART_HandleTypeDef *huart, const char* str);
/*-------------------------------------------------------------------------------------------------*/

//!!! probleem met deze functie // -- Results in a HardFault Interrupt !!
void UART_COM_print_buffer_hex(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len);
/*-------------------------------------------------------------------------------------------------*/





#endif



