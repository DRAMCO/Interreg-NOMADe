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

uint8_t previous_command = 0;
imu_module *imu = NULL;
uint8_t previous_connected_modules [6];


void USB_COM_show_menu(void){
	USB_COM_print_ln("\n\n########################################################################");
	USB_COM_print_ln("					MENU");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("Command \"0\": Print MENU");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("			General functions");
	USB_COM_print_ln("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
	USB_COM_print_ln("Command \"1\": Connect");
	USB_COM_print_ln("Command \"2\": Disconnect");
	USB_COM_print_ln("Command \"3\": Go to sleep");
	USB_COM_print_ln("Command \"4\": Print the battery voltages off all connected modules");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("			SD Card functions");
	USB_COM_print_ln("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
	USB_COM_print_ln("Command \"n\": Create new file");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("			IMU communication");
	USB_COM_print_ln("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -");
	USB_COM_print_ln("Command \"c\": Calibrate all connected modules");
	USB_COM_print_ln("Command \"s\": Start synchronisation");
	USB_COM_print_ln("Command \"r\": Run the measurment");
	USB_COM_print_ln("Command \"e\": End the measurment");
	USB_COM_print_ln("########################################################################\n\n");
}


void USB_COM_print(const char* str){
	UART_COM_print(&huart5, str);
}


void USB_COM_print_ln(const char* str){
	UART_COM_print_ln(&huart5, str);
}

void USB_COM_print_value_ln(const char* str, uint32_t value){
	char char_array [100];
	sprintf(char_array, str, value); 
  UART_COM_write(&huart5, (uint8_t *)char_array, strlen(char_array));
}

void USB_COM_print_info(const char* str, const char* str2){
	UART_COM_print(&huart5, str);
	UART_COM_print_ln(&huart5, str2);
}


void USB_COM_print_buffer_hex(uint8_t *buffer, uint8_t len){
	UART_COM_print_buffer_hex(&huart5, buffer, len);
}


void USB_COM_check_rx_buffer(void){
	if(UART_IsDataAvailable(&huart5)){
		
		uint8_t command = UART_COM_read(&huart5);
		
		if(command == 0x0A) return;
		
		if(previous_command != 0){
			switch(command){
				case 0x31:{
					imu = &imu_1;
				} break;
				case 0x32:{
					imu = &imu_2;
				} break;
				case 0x33:{
					imu = &imu_3;
				} break;
				case 0x34:{
					imu = &imu_4;
				} break;
				case 0x35:{
					imu = &imu_5;
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
				if(imu != NULL) {	IMU_go_to_sleep(imu);	imu = NULL; }
				else { previous_command = 0x33; USB_COM_print_ln("IMU number: "); }
			} break;
			case 0x34:{
				//if(imu != NULL) {	IMU_get_battery_voltage(imu);	imu = NULL; }
				//else { previous_command = 0x34; USB_COM_print_ln("IMU number: "); }
				for(uint8_t i = 0; i < 6; i++){
					if(imu_array[i]->connected)	IMU_get_battery_voltage(imu_array[i]);
				}
			} break;
			
			
			
			
			case 0x63:{
				for(uint8_t i = 0; i < 6; i++){
					if(imu_array[i]->connected)	IMU_start_calibration(imu_array[i]);
				}
			} break;
			case 0x73:{
				for(uint8_t i = 0; i < 6; i++){
					if(imu_array[i]->connected)	{	
						IMU_start_synchronisation(imu_array[i]); 
						previous_connected_modules [i] = 1; 
					}
				}
				
				IMU_send_adv_msg_wrong(&imu_1);
			
				HAL_Delay(500);
				
				IMU_send_adv_msg(&imu_1);
				/*
				//HAL_Delay(1000);
				
				for(uint8_t i = 0; i < 6; i++){
					if(previous_connected_modules[i] == 1)	IMU_connect(imu_array[i]);
				}
				memset(previous_connected_modules, 0, 6);
				*/
			} break;
			
			
			case 0x72:{
				USB_COM_print_ln("Start measurement");
				SD_CARD_COM_open_file();
				for(uint8_t i = 0; i < 6; i++){
					if(imu_array[i]->connected)	IMU_start_measurements(imu_array[i]);
				}
			}	break;
			
			case 0x65:{
				USB_COM_print_ln("Stop measurement");
				SD_CARD_COM_close_file();
				for(uint8_t i = 0; i < 6; i++){
					if(imu_array[i]->connected)	IMU_stop_measurements(imu_array[i]);
				}
			}	break;
			
			case 0x6E:{
				SD_CARD_COM_create_new_file();
			}	break;


			
			case 0x0A:{
				// It's just an enter (captered)
			} break;
			
			default:
				USB_COM_print_ln("Undefined command!");
		}
	}

}	








