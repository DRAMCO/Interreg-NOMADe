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
 *         File: uart_com.c
 *      Created: 2020-08-12
 *       Author: Jarne Van Mulders
 *      Version: V1.0
 *
 *  Description: Firmware IMU sensor module for the NOMADe project
 *
 *  Interreg France-Wallonie-Vlaanderen NOMADe
 *
 */
#include "uart_com.h"

void UART_COM_write(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len){	
	for(int i = 0; i < len; i++){
		UART_Write(huart, *(buf + i));
	}
}


uint8_t UART_COM_read(UART_HandleTypeDef *huart){
	return UART_Read(huart);
}


void UART_COM_print(UART_HandleTypeDef *huart, const char* str){
	int i = 0;
	for(i = 0; str[i]!='\0' ; i++) { }
	uint8_t buffer [i];
	for(int j = 0; j < i; j++){
		buffer [j] = (uint8_t)*(str + j);
	}
	UART_COM_write(huart, buffer, i);
}


void UART_COM_print_ln(UART_HandleTypeDef *huart, const char* str){
	int i = 0;
	for(i = 0; str[i]!='\0' ; i++) {}
	uint8_t buffer [i+1];
	for(int j = 0; j < i; j++){
		buffer [j] = (uint8_t)*(str + j);
	}
	buffer[i] = 0x0A;
	UART_COM_write(huart, buffer, i+1);
}


void UART_COM_print_buffer_hex(UART_HandleTypeDef *huart, uint8_t *array, uint8_t len){
	uint8_t buffer_len = 3*len + 1;
	char buffer [buffer_len];
	for (uint8_t i = 0; i < len; i++){
		uint8_t nib1 = (array[i] >> 4) & 0x0F;
		uint8_t nib2 = (array[i] >> 0) & 0x0F;
		buffer[i*3+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
		buffer[i*3+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
		buffer[i*3+2] = 0x20;
	}
	buffer[buffer_len - 1] = 0x0A;
	UART_COM_write(huart, (uint8_t *)buffer, buffer_len);
}











