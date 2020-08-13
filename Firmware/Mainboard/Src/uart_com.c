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

static void convert_to_hex_str(char* str, uint8_t* val, size_t val_count);


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


//!!! probleem met deze functie // -- Results in a HardFault Interrupt !!
void UART_COM_print_buffer_hex(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len){
	UART_COM_print(huart, "Buffer: ");
	
	char string [200];
	/*for(int i = 0; i < len; i++){
		sprintf(&string[3*i], "%02X ", (uint8_t)*(buf + i));
	}
	sprintf(&string[3*len], "\n");
	*/
	
	convert_to_hex_str(string, buf, len);
	UART_COM_write(huart, (uint8_t *)string, 3*len + 1);

}

static char hex[] = "0123456789ABCDEF"; 
void convert_to_hex_str(char* str, uint8_t* val, size_t val_count)
{
	for (size_t i = 0; i < val_count; i++)
	{
		str[(i * 3) + 0] = hex[((val[i] & 0xF0) >> 4)];
		str[(i * 3) + 1] = hex[((val[i] & 0x0F) >> 0)];
		str[(i * 3) + 2] = 0x20;
	}
}





/*
void printBuf(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len){
	
	char string [100];
	
	for(int i = 0; i < len; i++){
		sprintf(&(string[5*i]), "0x%X  ", (uint8_t)*(buf + i));
	}
	sprintf(&(string[5*len]), "\n");
	HAL_UART_Transmit(huart, (uint8_t *)string, 5*len + 1, 100);
	
}
void serPrintHex(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t len){

	char output[(len * 2) + 1];
	char *ptr = &output[0];
	int i;
	for (i = 0; i < len; i++)
	{
			ptr += sprintf(ptr, "%02X", buf[i]);
	}
	//serPrintln(huart, output);
}*/













