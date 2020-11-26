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

// ================================================================
// ===                    Static functions                      ===
// ================================================================

static uint8_t is_measuring(void);
static void send_to_all_sensors(void (*function)(imu_module*), imu_module *imu_array);
static void send_to_all_sensors_with_param(void (*function)(imu_module*, uint8_t), imu_module *imu_array, uint8_t df);
static uint8_t convert_ASCII_to_HEX(uint8_t msn, uint8_t lsn);
static void read_mac_address(imu_module *imu);

// ================================================================
// ===                    		Variables                      		===
// ================================================================

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

// ================================================================
// ===                     		Functions                     		===
// ================================================================

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
	USB_COM_print_ln("Command \"6\": Print MAC addresses to which the BLE slots should connect");
	USB_COM_print_ln("Command \"9\": Change MAC address of a BLE slot");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("			IMU communication (sended to all connected modules)");
	USB_COM_print_ln("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
	USB_COM_print_ln("Command \"c\": Start calibration");
	USB_COM_print_ln("Command \"s\": Start synchronization");
	USB_COM_print_ln("Command \"r\": Start the measurement with synchronisation");
	USB_COM_print_ln("Command \"t\": Start the measurement without synchronisation");
	USB_COM_print_ln("Command \"e\": End the measurement");
	USB_COM_print_ln("Command \"f\": Change sampling frequency");
	USB_COM_print_ln("Command \"4\": Get the battery voltage");
	USB_COM_print_ln("Command \"5\": Get the system tick");
	USB_COM_print_ln("Command \"a\": Get the IMU module status");
	USB_COM_print_ln("Command \"z\": Get the IMU module software version");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("			SD Card functions");
	USB_COM_print_ln("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
	USB_COM_print_ln("Command \"m\": Mount SD card");
	USB_COM_print_ln("Command \"u\": Unmount SD card");
	USB_COM_print_ln("Command \"n\": Create new file");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("			Change dataformat");
	USB_COM_print_ln("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
	USB_COM_print_ln("Command \"g\": Only Quaternion data");
	USB_COM_print_ln("Command \"h\": Only Gyroscope data");
	USB_COM_print_ln("Command \"j\": Only Accelerometer data");
	USB_COM_print_ln("Command \"k\": Gyroscope + Accelerometer data");
	USB_COM_print_ln("Command \"l\": Quaternion + Gyroscope + Accelerometer data");
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
					if(0x66 == previous_command) send_to_all_sensors_with_param(IMU_change_sampling_frequency, *imu_array, SAMPLING_FREQ_10HZ);
				} break;
				case 0x32:{
					imu = &imu_2;
					if(0x66 == previous_command) send_to_all_sensors_with_param(IMU_change_sampling_frequency, *imu_array, SAMPLING_FREQ_20HZ);
				} break;
				case 0x33:{
					imu = &imu_3;
					if(0x66 == previous_command) send_to_all_sensors_with_param(IMU_change_sampling_frequency, *imu_array, SAMPLING_FREQ_25HZ);
				} break;
				case 0x34:{
					imu = &imu_4;
					if(0x66 == previous_command) send_to_all_sensors_with_param(IMU_change_sampling_frequency, *imu_array, SAMPLING_FREQ_50HZ);
				} break;
				case 0x35:{
					imu = &imu_5;
					if(0x66 == previous_command) send_to_all_sensors_with_param(IMU_change_sampling_frequency, *imu_array, SAMPLING_FREQ_100HZ);
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
			
			case 0x39:{
				if(imu != NULL) {	read_mac_address(imu);	imu = NULL; }
				else { previous_command = 0x39; USB_COM_print_ln("Slot number: "); }
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
			
			
			//----------------------------------//
			//			***		CALIBRATION		***			//
			case 0x63:{		//	c-KEY
				send_to_all_sensors(IMU_start_calibration, *imu_array);
			} break;
			//----------------------------------//

			
			//----------------------------------------//
			//			***		SD CARD FUNCTIONS		***			//
			case 0x6E:{		//	n-KEY
				if(!is_measuring()) SD_CARD_COM_create_new_file();
				else USB_COM_print_ln("End measurement before generating a new file!"); 
			}	break;
			
			case 0x6D:{		//	m-KEY
				SD_CARD_COM_mount();
			}	break;	
			
			case 0x75:{		//	u-KEY
				if(!is_measuring()) SD_CARD_COM_unmount();
				else USB_COM_print_ln("End measurement before umounting the SD Card!");
			}	break;
			
			case 0x66:{		//	f-KEY
				if(imu != NULL){	imu = NULL; }
				else { 
					previous_command = 0x66; 
					USB_COM_change_frequency_menu();
					USB_COM_print_ln("Give number: "); 
				}
			} break;
			//----------------------------------------//
			
			
			//--------------------------------------------//
			//			***		START STOP MESUREMENT		***			//
			case 0x72:{		//	r-KEY
				if(SD_CARD_COM_get_status()){
					USB_COM_print_ln("Start measurement");
					SD_CARD_COM_open_file();
					send_to_all_sensors(IMU_start_measurements, *imu_array);
				}
				else	USB_COM_print_ln("No available sd card found");
			}	break;
			
			case 0x74:{		//	t-KEY
				if(SD_CARD_COM_get_status()){
					USB_COM_print_ln("Start measurement");
					SD_CARD_COM_open_file();
					send_to_all_sensors(IMU_start_measurements_without_sync, *imu_array);
				}
				else	USB_COM_print_ln("No available sd card found");
			} break;
			
			case 0x65:{		//	e-KEY
				if(is_measuring()){
					USB_COM_print_ln("Stop measurement");
					SD_CARD_COM_close_file();
					send_to_all_sensors(IMU_stop_measurements, *imu_array);
				}
				else USB_COM_print_ln("You cannot end the measurement because you have not started a measurement");
			}	break;
			//--------------------------------------------//
			
			
			
			//------------------------------------------//
			//			***		CHANGE DATA FORMAT		***			//
			case 0x67:{		//	g-KEY
				send_to_all_sensors_with_param(IMU_change_dataformat, *imu_array, DATA_FORMAT_1);				
			} break;
				
			case 0x68:{		//	h-KEY
				send_to_all_sensors_with_param(IMU_change_dataformat, *imu_array, DATA_FORMAT_2);
			} break;
			
			case 0x6A:{		//	j-KEY
				send_to_all_sensors_with_param(IMU_change_dataformat, *imu_array, DATA_FORMAT_3);
			} break;
			
			case 0x6B:{		//	k-KEY
				send_to_all_sensors_with_param(IMU_change_dataformat, *imu_array, DATA_FORMAT_4);
			} break;
			
			case 0x6C:{		//	l-KEY
				send_to_all_sensors_with_param(IMU_change_dataformat, *imu_array, DATA_FORMAT_5);
			} break;
			//------------------------------------------//
			
			
			//--------------------------------//
			//			***		IMU INFO		***			//
			case 0x61:{		//	a-KEY
				send_to_all_sensors(IMU_get_status, *imu_array);
			} break;
			
			case 0x7A:{		//	z-KEY
				send_to_all_sensors(IMU_get_software_version, *imu_array);
			} break;
			//--------------------------------//
			
			
			case 0x0A:{		// 	Capture enter-KEY
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




// ================================================================
// ===                    Static functions                      ===
// ================================================================

uint8_t is_measuring(void){
	uint8_t rt_val = 0;
	for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
		if(imu_array[i]->measuring) rt_val = 1;
	}
	return rt_val;
}

void send_to_all_sensors(void (*function)(imu_module*), imu_module *imu_array){
	for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
		if(imu_array[i].connected) function(&imu_array[i]);
	}
}

void send_to_all_sensors_with_param(void (*function)(imu_module*, uint8_t), imu_module *imu_array, uint8_t df){
	for(uint8_t i = 0; i < NUMBER_OF_SENSOR_SLOTS; i++){
		if(imu_array[i].connected) function(&imu_array[i], df);
	}
}

void read_mac_address(imu_module *imu){
	HAL_Delay(1);
	
	uint8_t new_mac_adress [6];
	
	for(uint8_t i = 0; i < 6; i++){
		uint8_t a, b;
		if(UART_IsDataAvailable(&huart5))	a = UART_COM_read(&huart5);
		if(UART_IsDataAvailable(&huart5))	b = UART_COM_read(&huart5);
		new_mac_adress [i] = convert_ASCII_to_HEX(a, b);
	}
	
	IMU_adjust_mac_address(imu, new_mac_adress);
	USB_COM_print_buffer_mac_address(imu->name, imu->mac_address);
	
}


uint8_t convert_ASCII_to_HEX(uint8_t msn, uint8_t lsn){
	uint8_t a = 0;
	
	if(msn < 0x40) a = (msn - 0x30) << 4;
	else{
		switch(msn){
			case 0x41:	a = 0xA << 4; break;
			case 0x42:	a = 0xB << 4; break;
			case 0x43:	a = 0xC << 4; break;
			case 0x44:	a = 0xD << 4; break;
			case 0x45:	a = 0xE << 4; break;
			case 0x46:	a = 0xF << 4; break;
		}
	}
	if(lsn < 0x40) a = a | (lsn - 0x30);
	else{
		switch(lsn){
			case 0x41:	a = a | 0xA; break;
			case 0x42:	a = a | 0xB; break;
			case 0x43:	a = a | 0xC; break;
			case 0x44:	a = a | 0xD; break;
			case 0x45:	a = a | 0xE; break;
			case 0x46:	a = a | 0xF; break;
		}
	}
	return a;
}




