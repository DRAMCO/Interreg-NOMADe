/**
  ******************************************************************************
  * @file    ble_lib.c
  * @brief   STM32H7432ZI communicate with Proteus-II BT module
	******************************************************************************
 **/


#include "ble_lib.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

uint8_t mac_buf_sens_1 [] = {0xB0, 0x0A, 0x20, 0xDA, 0x18, 0x00};
//uint8_t mac_buf_sens_1 [] = {0xAF, 0x0A, 0x20, 0xDA, 0x18, 0x00};
//uint8_t mac_buf_sens_1 [] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//uint8_t mac_buf_sens_2 [] = {0xE3, 0x04, 0x20, 0xDA, 0x18, 0x00}; 	// BT Module PIC
uint8_t mac_buf_sens_2 [] = {0xB0, 0x0A, 0x20, 0xDA, 0x18, 0x00}; 		// BT Module ATMEGA328P
uint8_t mac_buf_sens_3 [] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t mac_buf_sens_4 [] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t mac_buf_sens_5 [] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t mac_buf_sens_6 [] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


uint8_t SENS_connect(uint8_t number){
	uint8_t status;
	switch(number){
		case 1:		status = BT_connect(&huart4, mac_buf_sens_1);		break;
		case 2:		status = BT_connect(&huart6, mac_buf_sens_2);		break;
		case 3:		status = BT_connect(&huart7, mac_buf_sens_3);		break;		
		case 4:		status = BT_connect(&huart8, mac_buf_sens_4);		break;		
		case 5:		status = BT_connect(&huart1, mac_buf_sens_5);		break;
		case 6:		status = BT_connect(&huart2, mac_buf_sens_6);		break;
		default:  status = 0;
	}
	return status;
}

uint8_t SENS_disconnect(uint8_t number){
	uint8_t status;
	switch(number){
		case 1:		status = BT_disconnect(&huart4);		break;
		case 2:		status = BT_disconnect(&huart6);		break;
		case 3:		status = BT_disconnect(&huart7);		break;		
		case 4:		status = BT_disconnect(&huart8);		break;		
		case 5:		status = BT_disconnect(&huart1);		break;
		case 6:		status = BT_disconnect(&huart2);		break;
		default:  status = 0;
	}
	return status;
}


