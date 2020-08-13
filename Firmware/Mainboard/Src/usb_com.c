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


void USB_COM_show_menu(void){
	USB_COM_print_ln("\n\n########################################################################");
	USB_COM_print_ln("					MENU");
	USB_COM_print_ln("________________________________________________________________________");
	USB_COM_print_ln("Command \"0\": Print MENU");
	USB_COM_print_ln("Command \"1\": Make connection with module 1");
	USB_COM_print_ln("Command \"2\": Disconnect with module 1");
	USB_COM_print_ln("Command \"3\": Go to sleep with module 1");
	USB_COM_print_ln("Command \"4\": Print Battery voltage");
	USB_COM_print_ln("########################################################################\n\n");
}


void USB_COM_print(const char* str){
	UART_COM_print(&huart5, str);
}


void USB_COM_print_ln(const char* str){
	UART_COM_print_ln(&huart5, str);
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
		
		switch(command){
			case 0x30:{
				USB_COM_show_menu();
			} break;
			case 0x31:{
				IMU_connect(&imu_1);
			} break;
			case 0x32:{
				IMU_disconnect(&imu_1);
			} break;
			case 0x33:{
				IMU_go_to_sleep(&imu_1);
			} break;
			case 0x34:{
				IMU_get_battery_voltage(&imu_1);
			} break;
			
			case 0x0A:{
				// It's just an enter (captered)
			} break;
			
			default:
				USB_COM_print_ln("Undefined command!");
		}
	}

}	








