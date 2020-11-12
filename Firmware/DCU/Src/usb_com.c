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
 *         File: usb_com.c
 *      Created: 2020-02-27
 *       Author: Jarne Van Mulders
 *      Version: V1.0
 *
 *  Description: Firmware IMU sensor module for the NOMADe project
 *
 *  Interreg France-Wallonie-Vlaanderen NOMADe
 *
 */
#include "usb_com.h"

extern imu_module imu_1;
extern imu_module imu_2;
extern imu_module imu_3;
extern imu_module imu_4;
extern imu_module imu_5;
extern imu_module imu_6;

extern imu_module *imu_array [];

extern uint8_t state;

uint32_t last_sync_started_time = 0;

uint8_t previous_command = 0;
imu_module *imu = NULL;
uint8_t previous_connected_modules [6];
uint8_t sync_enable;


void USB_COM_show_menu(void){
	USB_COM_print_ln("\n\n########################################################################");
	USB_COM_print_ln("					Command list");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("Command \"0\": Print MENU");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("			General functions");
	USB_COM_print_ln("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
	USB_COM_print_ln("Command \"1\": Connect");
	USB_COM_print_ln("Command \"2\": Disconnect");
	USB_COM_print_ln("Command \"3\": Go to sleep");
	USB_COM_print_ln("Command \"4\": Print the battery voltages off all connected modules");
	USB_COM_print_ln("Command \"5\": Print system ticks of each module");
	USB_COM_print_ln("Command \"6\": Print MAC addresses to which the BLE slot should connect");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("			SD Card functions");
	USB_COM_print_ln("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
	USB_COM_print_ln("Command \"m\": Mount SD card");
	USB_COM_print_ln("Command \"u\": Unmount SD card");
	USB_COM_print_ln("Command \"n\": Create new file");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("			IMU communication");
	USB_COM_print_ln("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
	USB_COM_print_ln("Command \"c\": Calibrate all connected modules");
	USB_COM_print_ln("Command \"s\": Start synchronization");
	USB_COM_print_ln("Command \"r\": Start the measurement");
	USB_COM_print_ln("Command \"t\": Start the measurement without synchronisation");
	USB_COM_print_ln("Command \"e\": End the measurement");
	USB_COM_print_ln("Command \"f\": Change sampling frequency");
	USB_COM_print_ln("Command \"k\": Change output data format to only Quaternions data");
	USB_COM_print_ln("Command \"l\": Change output data format to Quaternions + Gyroscope + Accelerometer data");
	USB_COM_print_ln("########################################################################\n\n");
}


void USB_COM_print(const char* str){
	UART_COM_print(&huart5, str);
}


void USB_COM_print_ln(const char* str){
	UART_COM_print_ln(&huart5, str);
}

void USB_COM_print_value_ln(const char* str, uint32_t value){
	UART_COM_print(&huart5, str);
	char char_array [20];
	sprintf(char_array, " %d\n", value); 
  UART_COM_write(&huart5, (uint8_t *)char_array, strlen(char_array));
}

void USB_COM_print_info(const char* str, const char* str2){
	UART_COM_print(&huart5, str);
	UART_COM_print_ln(&huart5, str2);
}

void USB_COM_print_buffer_hex(uint8_t *buffer, uint8_t len){
	UART_COM_print_buffer_hex(&huart5, buffer, len);
}

void USB_COM_print_buffer_mac_address(const char* str, uint8_t *mac_address){
	USB_COM_print(str);
	USB_COM_print("MAC Address: ");
	USB_COM_print_buffer_hex(mac_address, 6);
}


void USB_COM_check_rx_buffer(void){
	if(UART_IsDataAvailable(&huart5)){
		
		uint8_t command = UART_COM_read(&huart5);
		
		if(command == 0x0A) return;
		
		if(previous_command != 0){
			switch(command){
				case 0x31:{
					imu = &imu_1;
					if(0x66 == previous_command) USB_COM_change_sampling_frequency_all(IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_10HZ);
				} break;
				case 0x32:{
					imu = &imu_2;
					if(0x66 == previous_command) USB_COM_change_sampling_frequency_all(IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_20HZ);
				} break;
				case 0x33:{
					imu = &imu_3;
					if(0x66 == previous_command) USB_COM_change_sampling_frequency_all(IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_25HZ);
				} break;
				case 0x34:{
					imu = &imu_4;
					if(0x66 == previous_command) USB_COM_change_sampling_frequency_all(IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_50HZ);
				} break;
				case 0x35:{
					imu = &imu_5;
					if(0x66 == previous_command) USB_COM_change_sampling_frequency_all(IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_100HZ);
				} break;
				case 0x36:{
					imu = &imu_6;
				} break;
				default:
					imu = NULL;
					USB_COM_print_ln("Please enter a number between 1 and 6!");
			}
			command = previous_command;
			previous_command = 0;
		}
		switch(command){
			case 0x30:{
				USB_COM_show_menu();
			} break;
			case 0x31:{
				if(imu != NULL) {	IMU_connect(imu);	imu = NULL; }
				else { previous_command = 0x31; USB_COM_print_ln("IMU number: "); }
			} break;
			case 0x32:{
				if(imu != NULL) {	IMU_disconnect(imu);	imu = NULL; }
				else { previous_command = 0x32; USB_COM_print_ln("IMU number: "); }
			} break;
			case 0x33:{
				//if(imu != NULL) {	IMU_go_to_sleep(imu);	imu = NULL; }
				//else { previous_command = 0x33; USB_COM_print_ln("IMU number: "); }
				for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
					if(imu_array[i]->connected)	IMU_go_to_sleep(imu_array[i]);
				}
			} break;
			case 0x34:{
				for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
					if(imu_array[i]->connected)	IMU_get_battery_voltage(imu_array[i]);
				}
			} break;
			case 0x35:{
				for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
					if(imu_array[i]->connected)	IMU_get_systicks(imu_array[i]);
				}
			} break;
			case 0x36:{
				for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
					USB_COM_print_buffer_mac_address(imu_array[i]->name, imu_array[i]->mac_address);
				}
			} break;
			
			case 0x37:{
				for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
					if(imu_array[i]->connected)	IMU_get_sync_time(imu_array[i]);
				}
			} break;
			
			case 0x38:{
				IMU_synchronisation_adaptation(*imu_array);
			} break;
			
			
			
			
			
			case 0x63:{
				for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
					if(imu_array[i]->connected)	IMU_start_calibration(imu_array[i]);
				}
			} break;
			case 0x73:{
				//memset(previous_connected_modules, 0, 6);
				
				last_sync_started_time = HAL_GetTick();
				
				for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
					if(imu_array[i]->connected)	{	
						IMU_start_synchronisation(imu_array[i]); 
						previous_connected_modules [i] = 1; 
						//USB_COM_print_info(imu_array[i]->name, "con"); 
					}
				}
				
				IMU_sync_reset();
				
				//BT_disconnect(imu_1.uart);
				HAL_Delay(10);
				//BT_startScanning(imu_1.uart);
				
				sync_enable = 1;
				
				IMU_send_adv_msg_wrong(&imu_1);
			
				//HAL_Delay(500);
				
				//IMU_send_adv_msg(&imu_1);
				/*
				//HAL_Delay(1000);
				
				for(uint8_t i = 0; i < 6; i++){
					if(previous_connected_modules[i] == 1)	IMU_connect(imu_array[i]);
				}
				memset(previous_connected_modules, 0, 6);
				*/
			} break;
			
			
			case 0x72:{
				if(SD_CARD_COM_get_status()){
					USB_COM_print_ln("Start measurement");
					SD_CARD_COM_open_file();
					for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
						if(imu_array[i]->connected){
							IMU_start_measurements(imu_array[i]);
						}
					}
				}
				else	USB_COM_print_ln("No available sd card found");
			}	break;
			
			case 0x65:{
				USB_COM_print_ln("Stop measurement");
				SD_CARD_COM_close_file();
				for(uint8_t i = 0; i < 6; i++){
					if(imu_array[i]->connected){	
						IMU_stop_measurements(imu_array[i]);
					}
				}
			}	break;
			
			case 0x6E:{
				SD_CARD_COM_create_new_file();
			}	break;
			
			case 0x6D:{
				SD_CARD_COM_mount();
			}	break;	
			
			case 0x75:{
				SD_CARD_COM_unmount();
			}	break;	
			

			case 0x66:{
				if(imu != NULL){	imu = NULL; }
				else { 
					previous_command = 0x66; 
					USB_COM_change_frequency_menu();
					USB_COM_print_ln("Give number: "); 
				}
			} break;
			
			
			case 0x6B:{
				for(uint8_t i = 0; i < 6; i++){
					if(imu_array[i]->connected){	
						IMU_change_dataformat(imu_array[i], DATA_FORMAT_1);
					}
				}				
			} break;
				
			case 0x6C:{
				for(uint8_t i = 0; i < 6; i++){
					if(imu_array[i]->connected){	
						IMU_change_dataformat(imu_array[i], DATA_FORMAT_2);
					}
				}
			} break;
			
			case 0x74:{
				if(SD_CARD_COM_get_status()){
					USB_COM_print_ln("Start measurement");
					SD_CARD_COM_open_file();
					for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
						if(imu_array[i]->connected){
							IMU_start_measurements_without_sync(imu_array[i]);
						}
					}
				}
				else	USB_COM_print_ln("No available sd card found");
			} break;
			
			
			
			case 0x0A:{
				// It's just an enter (captered)
			} break;
			
			default:
				USB_COM_print_ln("Undefined command!");
		}
	}

}	


void USB_COM_change_frequency_menu(void){
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("			MENU -- Sample frequency options");
	USB_COM_print_ln("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
	USB_COM_print_ln("Command \"1\": Sampling frequency 10 Hz");
	USB_COM_print_ln("Command \"2\": Sampling frequency 20 Hz");
	USB_COM_print_ln("Command \"3\": Sampling frequency 25 Hz");
	USB_COM_print_ln("Command \"4\": Sampling frequency 50 Hz");
	USB_COM_print_ln("Command \"5\": Sampling frequency 100 Hz");
	USB_COM_print_ln("________________________________________________________________________");
}




void USB_COM_change_sampling_frequency_all(uint8_t command){
	for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
		if(imu_array[i]->connected)	IMU_change_sampling_frequency(imu_array[i], command);
	}
}




