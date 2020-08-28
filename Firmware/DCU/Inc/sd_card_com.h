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

#ifndef SD_CARD_COM_H_
#define SD_CARD_COM_H_

#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "fatfs.h"
#include "usb_com.h"


FRESULT SD_CARD_COM_mount(void);
/*-------------------------------------------------------------------------------------------------*/


void SD_CARD_COM_create_new_file(void);
/*-------------------------------------------------------------------------------------------------*/


uint16_t SD_CARD_COM_check_existing_files(void);
/*-------------------------------------------------------------------------------------------------*/


void SD_CARD_COM_open_file(void);
/*-------------------------------------------------------------------------------------------------*/


void SD_CARD_COM_close_file(void);
/*-------------------------------------------------------------------------------------------------*/


void SD_CARD_COM_save_data(uint32_t systemtick, uint8_t sensor_number, uint8_t *sd_card_buffer);
/*-------------------------------------------------------------------------------------------------*/


FRESULT SD_CARD_COM_open_close(FIL* fp, const TCHAR* p, BYTE mode);
/*-------------------------------------------------------------------------------------------------*/


#endif



