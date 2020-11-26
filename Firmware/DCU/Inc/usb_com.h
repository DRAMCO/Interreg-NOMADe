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
 *         File: uart_com.h
 *      Created: 2020-08-12
 *       Author: Jarne Van Mulders
 *      Version: V1.0
 *
 *  Description: Firmware IMU sensor module for the NOMADe project
 *
 *  Interreg France-Wallonie-Vlaanderen NOMADe
 *
 */

#ifndef USB_COM_H_
#define USB_COM_H_

#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "imu_com.h"
#include "sd_card_com.h"

extern UART_HandleTypeDef huart5;

void USB_COM_show_menu(void);
/*-------------------------------------------------------------------------------------------------*/


void USB_COM_print(const char* str);
/*-------------------------------------------------------------------------------------------------*/


void USB_COM_print_ln(const char* str);
/*-------------------------------------------------------------------------------------------------*/


void USB_COM_print_value_ln(const char* str, uint32_t value);
/*-------------------------------------------------------------------------------------------------*/


void USB_COM_print_info(const char* str, const char* str2);
/*-------------------------------------------------------------------------------------------------*/


void USB_COM_print_buffer_hex(uint8_t *buffer, uint8_t len);
/*-------------------------------------------------------------------------------------------------*/


void USB_COM_print_buffer_mac_address(const char* str, uint8_t *mac_address);
/*-------------------------------------------------------------------------------------------------*/


void USB_COM_check_rx_buffer(void);
/*-------------------------------------------------------------------------------------------------*/


void USB_COM_change_frequency_menu(void);
/*-------------------------------------------------------------------------------------------------*/


#endif



