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
	volatile uint8_t number;
	UART_HandleTypeDef *uart;
	uint8_t mac_address [6];
	char *name;
	volatile uint8_t connected;
	volatile uint8_t is_calibrated;
	volatile uint8_t measuring;
	volatile uint16_t battery_voltage;
	volatile uint32_t sync_time;
} imu_module;

// ================================================================
// ===              Define Communication Commands               ===
// ================================================================

#define IMU_SENSOR_MODULE_RSP_SEND_DATA_F1                              0x20    //  QUAT
#define IMU_SENSOR_MODULE_RSP_SEND_DATA_F2                              0x21    //  GYRO
#define IMU_SENSOR_MODULE_RSP_SEND_DATA_F3                              0x22    //  ACC
#define IMU_SENSOR_MODULE_RSP_SEND_DATA_F4                              0x23    //  GYRO + ACC
#define IMU_SENSOR_MODULE_RSP_SEND_DATA_F5                              0x24    //  QUAT + GYRO + ACC
#define IMU_SENSOR_MODULE_REQ_CHANGE_DF                                 0x25
#define IMU_SENSOR_MODULE_IND_DF_CHANGED                                0x26

#define DATA_FORMAT_1                                                   0x01
#define DATA_FORMAT_2                                                   0x02
#define DATA_FORMAT_3                                                   0x03
#define DATA_FORMAT_4                                                   0x04
#define DATA_FORMAT_5                                                   0x05

#define IMU_SENSOR_MODULE_REQ_STATUS                                    0x30
#define IMU_SENSOR_MODULE_IND_STATUS                                    0x31

#define IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE                           0x40
#define IMU_SENSOR_MODULE_IND_BATTERY_VOLTAGE                           0x41
#define IMU_SENSOR_MODULE_IND_BATTERY_LOW_ERROR                         0x42

#define IMU_SENSOR_MODULE_REQ_START_SYNC                                0x50
#define IMU_SENSOR_MODULE_IND_SYNC_STARTED                              0x51
#define IMU_SENSOR_MODULE_IND_SYNC_DONE                                 0x52
#define IMU_SENSOR_MODULE_IND_CANNOT_SYNC                               0x53
#define IMU_SENSOR_MODULE_IND_NEED_TO_SYNCHRONISE                       0x54
#define IMU_SENSOR_MODULE_REQ_GET_SYNC_TIME                             0x55
#define IMU_SENSOR_MODULE_IND_SYNC_TIME                                 0x56
#define IMU_SENSOR_MODULE_REQ_CHANGE_SYNC_TIME                          0x57
#define IMU_SENSOR_MODULE_IND_SYNC_TIME_CHANGED                         0x58

#define IMU_SENSOR_MODULE_REQ_STOP_MEASUREMENTS                         0x60
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_STOPPED                      0x61
#define IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS                        0x62
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_STARTED                      0x63
#define IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS_WITHOUT_SYNC           0x64
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_STARTED_WITHOUT_SYNC         0x65

#define IMU_SENSOR_MODULE_REQ_START_CALIBRATION                         0x70
#define IMU_SENSOR_MODULE_IND_CALIBRATION_STARTED                       0x74
#define IMU_SENSOR_MODULE_IND_CANNOT_CALIBRATE                          0x71
#define IMU_SENSOR_MODULE_IND_CALIBRATION_DONE                          0x72
#define IMU_SENSOR_MODULE_IND_NEED_TO_CALIBRATE                         0x73

#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_CHANGED                     0x80
#define IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED                     0x81
#define SAMPLING_FREQ_10HZ                                              0x01
#define SAMPLING_FREQ_20HZ                                              0x02
#define SAMPLING_FREQ_25HZ                                              0x03
#define SAMPLING_FREQ_50HZ                                              0x04
#define SAMPLING_FREQ_100HZ                                             0x05

#define IMU_SENSOR_MODULE_REQ_GO_TO_SLEEP                               0x90
#define IMU_SENSOR_MODULE_IND_SLEEP_MODE                                0x91

#define IMU_SENSOR_MODULE_REQ_CHANGE_BAT_LOW_VOLT                       0xA0    // Hier moet extra data mee worden doorgestuurd
#define IMU_SENSOR_MODULE_IND_BAT_LOW_VOLT_CHANGED                      0xA1

#define IMU_SENSOR_MODULE_REQ_MILLIS                                    0xB0
#define IMU_SENSOR_MODULE_RSP_MILLIS                                    0xB1

#define IMU_SENSOR_MODULE_REQ_SW_VERSION                                0xC0
#define IMU_SENSOR_MODULE_RSP_SW_VERSION                                0xC1

#define ADV_MSG                                                         0xF0


extern uint8_t previous_connected_modules [6];
extern uint8_t sync_enable;

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


void IMU_send_adv_msg_wrong(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_send_adv_msg(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_go_to_sleep(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_start_measurements(imu_module *imu);
void IMU_start_measurements_without_sync(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_stop_measurements(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_change_sampling_frequency(imu_module *imu, uint8_t command);
/*-------------------------------------------------------------------------------------------------*/


void IMU_sync_reset(void);
/*-------------------------------------------------------------------------------------------------*/


void IMU_change_dataformat(imu_module *imu, uint8_t df);
/*-------------------------------------------------------------------------------------------------*/


void IMU_sync_handler(imu_module *imu, imu_module *imu_array);
/*-------------------------------------------------------------------------------------------------*/


void IMU_get_systicks(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_get_sync_time(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_get_change_sync_time(imu_module *imu, uint16_t value);
/*-------------------------------------------------------------------------------------------------*/


void IMU_reset_previous_connected_modules_array(void);
/*-------------------------------------------------------------------------------------------------*/


void IMU_synchronisation_adaptation(imu_module *imu_array);
/*-------------------------------------------------------------------------------------------------*/


void IMU_get_status(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_get_software_version(imu_module *imu);
/*-------------------------------------------------------------------------------------------------*/


void IMU_adjust_mac_address(imu_module *imu, uint8_t *buffer);
/*-------------------------------------------------------------------------------------------------*/

#endif



