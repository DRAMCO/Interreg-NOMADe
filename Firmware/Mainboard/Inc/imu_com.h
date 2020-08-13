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

#ifndef IMU_COM_H_
#define IMU_COM_H_

#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "proteusII_driver.h"


/* IMU module status variables -----------------------------------------------*/
typedef struct __imu_module{
	UART_HandleTypeDef *uart;
	uint8_t mac_address [6];
	char *name;
	volatile uint8_t connected;
	volatile uint8_t is_calibrated;
	volatile uint16_t battery_voltage;
	/* ... */
} imu_module;

// ================================================================
// ===              Define Communication Commands               ===
// ================================================================

#define IMU_SENSOR_MODULE_REQ_STATUS                0x30
#define IMU_SENSOR_MODULE_IND_STATUS                0x31

#define IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE       0x40
#define IMU_SENSOR_MODULE_IND_BATTERY_VOLTAGE       0x41
#define IMU_SENSOR_MODULE_IND_BATTERY_LOW_ERROR     0x42

#define IMU_SENSOR_MODULE_REQ_START_SYNC            0x60
#define IMU_SENSOR_MODULE_IND_SYNC_DONE             0x61
#define IMU_SENSOR_MODULE_REQ_STOP_MEASUREMENTS     0x62
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_STOPPED  0x63
#define IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS    0x64
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_STARTED  0x65

#define IMU_SENSOR_MODULE_REQ_START_CALIBRATION     0x70
#define IMU_SENSOR_MODULE_IND_CALIBRATION_STARTED   0x74    //  Nog toevoegen
#define IMU_SENSOR_MODULE_IND_CANNOT_CALIBRATE      0x71
#define IMU_SENSOR_MODULE_IND_CALIBRATION_DONE      0x72
#define IMU_SENSOR_MODULE_IND_NEED_TO_CALIBRATE     0x73

#define IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED 0x80
#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_10HZ    0x81
#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_20HZ    0x82
#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_25HZ    0x83
#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_50HZ    0x84
#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_100HZ   0x85

#define IMU_SENSOR_MODULE_REQ_CHANGE_BAT_LOW_VOLT   0xA0    // Hier moet extra data mee worden doorgestuurd
#define IMU_SENSOR_MODULE_IND_BAT_LOW_VOLT_CHANGED  0xA1

#define IMU_SENSOR_MODULE_REQ_GO_TO_SLEEP           0x90
#define IMU_SENSOR_MODULE_IND_SLEEP_MODE            0x91

#define ADV_MSG                                     0xF0


void IMU_connect(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_disconnect(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_get_battery_voltage(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_start_calibration(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_start_synchronisation(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_send_adv_msg(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_go_to_sleep(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/



#endif



