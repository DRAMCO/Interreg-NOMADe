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

#ifndef UARTRINGBUFFERMANAGER_H_
#define UARTRINGBUFFERMANAGER_H_

#include "stm32h7xx_hal.h"
#include "UartRingbuffer.h"


void UART_Ringbuffer_Init(UART_HandleTypeDef *huart);
/*-------------------------------------------------------------------------------------------------*/


uint8_t UART_Read(UART_HandleTypeDef *huart);
/*-------------------------------------------------------------------------------------------------*/


void UART_Write(UART_HandleTypeDef *huart, uint8_t byte);
/*-------------------------------------------------------------------------------------------------*/


uint16_t UART_IsDataAvailable(UART_HandleTypeDef *huart);
/*-------------------------------------------------------------------------------------------------*/


void UART_ISR_Manager(UART_HandleTypeDef *huart);
/*-------------------------------------------------------------------------------------------------*/

























#endif



