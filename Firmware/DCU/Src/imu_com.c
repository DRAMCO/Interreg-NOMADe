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

enum sync_state{STATE00, STATE0, STATE1, STATE2, STATE3, STATE4, STATE5, STATE6, STATE7, STATE8};
uint8_t state = STATE0;
uint8_t wait_time = 0;

uint8_t sync_counter = 0;
extern uint32_t last_sync_started_time;

static void IMU_set_sync_time_zero(imu_module *imu_array);

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

void IMU_send_adv_msg_wrong(imu_module *imu){
	BT_transmit_CMD_Byte(imu->uart, CMD_SETBEACON_REQ, 0x00);
}

void IMU_send_adv_msg(imu_module *imu){
	BT_transmit_CMD_Byte(imu->uart, CMD_SETBEACON_REQ, ADV_MSG);
}

void IMU_go_to_sleep(imu_module *imu){
	BT_transmitMsg_CMD(imu->uart, IMU_SENSOR_MODULE_REQ_GO_TO_SLEEP);
}

void IMU_start_measurements(imu_module *imu){
	uint16_t value = (HAL_GetTick() - last_sync_started_time)/1000 + 2;
	uint8_t buf [] = {(uint8_t) value, (uint8_t) value >> 8}; 
	BT_transmitMsg_CMD_Data(imu->uart, IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS, 2, buf);
	//BT_transmitMsg_CMD(imu->uart, IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS);
}

void IMU_start_measurements_without_sync(imu_module *imu){
	BT_transmitMsg_CMD(imu->uart, IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS_WITHOUT_SYNC);
}

void IMU_stop_measurements(imu_module *imu){
	BT_transmitMsg_CMD(imu->uart, IMU_SENSOR_MODULE_REQ_STOP_MEASUREMENTS);
}

void IMU_change_sampling_frequency(imu_module *imu, uint8_t command){
	BT_transmitMsg_CMD_Data(imu->uart, IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_CHANGED, 1, &command);
}


void IMU_sync_reset(void){
	state = STATE00;
}

void IMU_change_dataformat(imu_module *imu, uint8_t df){
	BT_transmitMsg_CMD_Data(imu->uart, IMU_SENSOR_MODULE_REQ_CHANGE_DF, 1, &df);
}
	
void IMU_sync_handler(imu_module *imu, imu_module *imu_array){
	
	if(sync_enable){
	
		
			//	Do not sync during measurements
		for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
			if((imu_array + i)->measuring)	{	
				return;
			}
		}
		/*
		for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
			if((imu_array + i)->connected){	
				USB_COM_print_info((imu_array + i)->name, "con"); 
			}
		}
		*/
		
		
		switch(state){
			case STATE00:{
				state = STATE0;
			} break;
			case STATE0:{
				//USB_COM_print("STATE1");
				IMU_send_adv_msg(imu);
				state = STATE1;
			} break;
			
			case STATE1:{
				//USB_COM_print("STATE1");
				//IMU_send_adv_msg(imu);
				state = STATE2;
			} break;
			
				//	Reconnect
			case STATE2:{
				//USB_COM_print("STATE2");
				for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
					if(previous_connected_modules[i])	{	
						IMU_connect((imu_array + i)); 
					}
				}
				state = STATE3;
			} break;
			
				//	Wait a few seconds
			case STATE3:{
				//USB_COM_print("STATE3");
				if(wait_time < 4)	wait_time++;
				else {
					state = STATE4;
					wait_time = 0;
				}
			} break;
			
				//	Check connections
			case STATE4:{
				//USB_COM_print("STATE4");
				state = STATE5;
				for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
					//if(previous_connected_modules[i]) USB_COM_print_info((imu_array + i)->name, "prev con");
					//if((imu_array + i)->connected) USB_COM_print_info((imu_array + i)->name, "con");
					if(previous_connected_modules[i] != (imu_array + i)->connected)	{	
						IMU_connect((imu_array + i)); 
						//USB_COM_print_info((imu_array + i)->name, "");
						//if((imu_array + i)->connected) USB_COM_print("Connection");
						//if(previous_connected_modules[i]) USB_COM_print("Was connected");
						state = STATE3;
					}
				}
			} break;
			
			case STATE5:{
				IMU_set_sync_time_zero(imu_array);
				for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
					if((imu_array + i)->connected)	IMU_get_sync_time((imu_array + i));
				}
				state = STATE6;
			} break;
			
			case STATE6:{
				for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
					if((imu_array + i)->sync_time > 0){
						state = STATE7;
						break;
					}
				}
			} break;
			
			case STATE7:{
				IMU_synchronisation_adaptation(imu_array);
				state = STATE8;
			} break;
			
				//	Clear previous connected modules array
			case STATE8:{
				//USB_COM_print("STATE5");
				IMU_reset_previous_connected_modules_array();
				USB_COM_print_ln("Synchronisation done");
				sync_enable = 0;
				wait_time = 0;
				state = STATE00;
			} break;	
			default:
				break;
		}
	}
	
	
	/*
	
	
	if(sync_enable){
		if(sync_counter == 2){	
			sync_counter++;
			//BT_stopScanning(imu->uart);
		}
		if(sync_counter == 3){	
			sync_counter++;
			IMU_send_adv_msg(imu);			
		}
		if(sync_counter == 4){
			sync_counter++;
			//IMU_send_adv_msg_wrong(imu);
			for(uint8_t i = 0; i < 6; i++){
				if(previous_connected_modules[i])	{	
					IMU_connect((imu_array + i)); 
					previous_connected_modules[i] = 0;
				}
			}
		}
		if(sync_counter == 4){
			
			for(uint8_t i = 0; i < 6; i++){
				if(previous_connected_modules[i])	{	
					IMU_connect((imu_array + i)); 
					previous_connected_modules[i] = 0;
				}
			}
			
			sync_enable = 0;
			sync_counter = 0;
		
		
		
		
		
	}
		*/
}


void IMU_reset_previous_connected_modules_array(void){
	for(uint8_t i = 0; i < 6; i++){
		if(previous_connected_modules[i])	{	
			previous_connected_modules[i] = 0;
		}
	}
}


void IMU_get_systicks(imu_module *imu){
	BT_transmitMsg_CMD(imu->uart, IMU_SENSOR_MODULE_REQ_MILLIS);
}

void IMU_get_sync_time(imu_module *imu){
	uint16_t value = (HAL_GetTick() - last_sync_started_time)/1000 + 1;
	uint8_t buf [] = {(uint8_t) value, (uint8_t) value >> 8}; 
	BT_transmitMsg_CMD_Data(imu->uart, IMU_SENSOR_MODULE_REQ_GET_SYNC_TIME, 2, buf);
}

void IMU_get_change_sync_time(imu_module *imu, uint16_t value){
	uint8_t buf [] = {(uint8_t) value, (uint8_t) value >> 8}; 
	BT_transmitMsg_CMD_Data(imu->uart, IMU_SENSOR_MODULE_REQ_CHANGE_SYNC_TIME, 2, buf);
}

void IMU_synchronisation_adaptation(imu_module *imu_array){
	uint32_t highest_value = 0;
	for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
		if((imu_array + i)->connected){
			if((imu_array + i)->sync_time > highest_value)
				highest_value = (imu_array + i)->sync_time;
		}
	}
	
	for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
		if((imu_array + i)->connected){
			uint32_t substact = highest_value - (imu_array + i)->sync_time;
			uint8_t dev = substact / 250;
			uint8_t rest = substact % 250;
			if(rest > 150) 	IMU_get_change_sync_time((imu_array + i), dev + 1);
			else{
				if(dev != 0) 	IMU_get_change_sync_time((imu_array + i), dev);
			}
		}
	}
}


void IMU_set_sync_time_zero(imu_module *imu_array){
	for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
		(imu_array + i)->sync_time = 0;
	}
}	

void IMU_get_status(imu_module *imu){
	BT_transmitMsg_CMD(imu->uart, IMU_SENSOR_MODULE_REQ_STATUS);
}

void IMU_get_software_version(imu_module *imu){
	BT_transmitMsg_CMD(imu->uart, IMU_SENSOR_MODULE_REQ_SW_VERSION);
}

void IMU_adjust_mac_address(imu_module *imu, uint8_t *buffer){
	for(uint8_t i = 0; i < 6; i++){
		imu->mac_address [i] = *(buffer + i);
	}
}
