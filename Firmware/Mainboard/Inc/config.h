/**
  ******************************************************************************
  * @file    config.h
  * @brief   This file contains the headers for the bluetooth methods.
	******************************************************************************
 **/

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32h7xx_hal.h"
#include "main.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);


#ifdef __cplusplus
}
#endif

#endif
/**********************************END OF FILE**********************************/
