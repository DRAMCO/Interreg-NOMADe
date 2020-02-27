/**
  ******************************************************************************
  * @file    ble_lib.h
  * @brief   This file contains the headers for the bluetooth methods.
	******************************************************************************
 **/

#ifndef BLE_LIB_H
#define BLE_LIB_H

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32h7xx_hal.h"
#include "ble.h"

uint8_t SENS_connect(uint8_t number);
uint8_t SENS_disconnect(uint8_t number);


#ifdef __cplusplus
}
#endif

#endif
/**********************************END OF FILE**********************************/
