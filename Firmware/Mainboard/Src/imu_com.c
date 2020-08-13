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
 *         File: imu_com.c
 *      Created: 2020-08-13
 *       Author: Jarne Van Mulders
 *      Version: V1.0
 *
 *  Description: Firmware IMU sensor module for the NOMADe project
 *
 *  Interreg France-Wallonie-Vlaanderen NOMADe
 *
 */
#include "imu_com.h"

void IMU_connect(imu_module *imu){
	BT_connect(imu->uart, imu->mac_address);
}

void IMU_disconnect(imu_module *imu){
	BT_disconnect(imu->uart);
}

void IMU_get_battery_voltage(imu_module *imu){
	BT_transmitMsg_CMD(imu->uart, IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE);
}

void IMU_start_calibration(imu_module *imu){
	BT_transmitMsg_CMD(imu->uart, IMU_SENSOR_MODULE_REQ_START_CALIBRATION);	
}

void IMU_start_synchronisation(imu_module *imu){
	BT_transmitMsg_CMD(imu->uart, IMU_SENSOR_MODULE_REQ_START_SYNC);
	
}

void IMU_send_adv_msg(imu_module *imu){
	uint8_t buf [4]	= {ADV_MSG};
	BT_transmit_CMD_Bytes(imu->uart, 0x0C, 0x01, buf);
}

void IMU_go_to_sleep(imu_module *imu){
	BT_transmitMsg_CMD(imu->uart, IMU_SENSOR_MODULE_REQ_GO_TO_SLEEP);
}








