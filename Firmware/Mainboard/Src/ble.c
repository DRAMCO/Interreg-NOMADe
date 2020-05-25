/**
  ******************************************************************************
  * @file    ble.c
  * @brief   STM32H7432ZI communicates with Proteus-II BT module
	******************************************************************************
 **/


#include "ble.h"

extern UART_HandleTypeDef huart5; 

extern uint8_t val;

//	BT - MCU Communication

void BT_Initialize(void){
}

void BT_getMAC(UART_HandleTypeDef *haurt){
    BT_getReq(haurt, FS_BTMAC);
    //EUSART1_Read_Block(12);
		//for(uint8_t i = 0; i < 6; i++)
    //    bt_mac [i] = rx_buffer [5 + i];  
}

uint8_t BT_connect(UART_HandleTypeDef *huart, uint8_t *mac){
	
	
		// Empty receive buffer!!!!
	
	
		// Eerst nakijken of het al niet verbonden is!!!!
	
		//serPrintln(&huart5, "Try to connect");
		
		BT_transmitFrame(huart, CMD_CONNECT_REQ, 6, mac);
	
		uint8_t rx_buf [20];
		if(HAL_UART_Receive(huart, rx_buf, 6, 100) == HAL_OK){
			//serPrintln(&huart5, "OK");
		}
		//printBuf(&huart5, rx_buf, 6);
		if(rx_buf[4] == 0){
			serPrintln(&huart5, "Request understood, try to connect now");
		}
		else {
			serPrintln(&huart5, "ERROR connection");
			return 0;
		}
			
		
		if(HAL_UART_Receive(huart, rx_buf, 5, 2000) == HAL_OK){
			//serPrintln(&huart5, "OK");
		}
		//printBuf(&huart5, rx_buf, 5);
		if(rx_buf[4] == 0){
			serPrintln(&huart5, "Bonding successful, encrypted link established");
			//if(HAL_UART_Receive(haurt, rx_buf, 1, 2000) == HAL_OK){
				//serPrintln(&huart5, "OK");
			//}
			HAL_UART_Abort(huart);
			//printBuf(&huart5, rx_buf, 1);
			if(HAL_UART_Receive(huart, rx_buf, 13, 5000) == HAL_OK){
				//serPrintln(&huart5, "OK");
			}
			printBuf(&huart5, rx_buf, 13);
			serPrintln(&huart5, "Channel opened successfully to module");
		}
		else {
			serPrintln(&huart5, "Connection failed, e.g. due to a timeout!");
			if(HAL_UART_Receive(huart, rx_buf, 1, 2000) == HAL_OK){
			//serPrintln(&huart5, "OK");
		}
			//printBuf(&huart5, rx_buf, 1);
			return 0;
		}
		
		
		
		
		/*
		
		HAL_UART_Receive(haurt, rx_buf, 4, 500);
		len = rx_buf[2] | rx_buf[3] << 8;
		if(len != 0){
			HAL_UART_Receive(haurt, &(rx_buf[4]), len, 100);
			HAL_UART_Transmit(&huart5, rx_buf, 4 + len, 100);
			if(rx_buf[4]){
				serPrintln(&huart5, "Bonding successful, encrypted link established");
			}
			else{
				serPrintln(&huart5, "ERROR connection");
				return 0;
			}
		}
		
		HAL_UART_Receive(haurt, rx_buf, 4, 1000);
		len = rx_buf[2] | rx_buf[3] << 8;
		if(len != 0){
			HAL_UART_Receive(haurt, &(rx_buf[4]), len - 4, 100);
			HAL_UART_Transmit(&huart5, rx_buf, len, 100);
			if(rx_buf[4]){
				serPrintln(&huart5, "Channel opened successfully to module");
			}
			else{
				serPrintln(&huart5, "ERROR connection");
				return 0;
			}
		}
		*/
		
		
	
	
	
	/*
		if(BT_getConnectReq(haurt)){
			BT_getConnectInd(haurt);
			BT_getChannelopenRSP(haurt);
		}
		*/
		// Moet nog aangepast worden als ConnectReq false is HW check connection anders is er zeker en vast geen connectie
		return 1;
		
	
		// HIER pas returnen als led is uitgelezen!!!!
	
	
	
	
	/*
		EUSART1_Read_Block(6);
		if(rx_buffer [4] != 0x00){	HAL_Delay(1000);	return 0;		} 
		else {
			if(EUSART1_Read_Block_TimeOut(12, 500) == HAL_TIMEOUT) {	return 0;	} 
			else {	EUSART1_Read_Block(13);	maxPayloadSize = rx_buffer [11];	}
		}
	*/
}




uint8_t BT_disconnect(UART_HandleTypeDef *haurt){
		return BT_transmitFrame(haurt, CMD_DISCONNECT_REQ, 0, NULL);
		//EUSART1_Read_Block(6);
}

void BT_changeBaudrate(uint8_t baud){
		//BT_setReqParam(UART_BaudrateIndex, baud);
		//BT_UART5_Init(921600);
		//EUSART1_Read_Block(6);
}




// 	Communication with the watch


#define time_out_receiving_data		20		//	[ms]

/*
Probleem met Timeouts nu
-> constante timeouts zorgt voor verlies van data packetten waarin een timeout valt
-> gesynchroniseerd doorsturen zal waarschijnlijk beter zijn

*/


uint8_t BT_getSensorData(uint8_t * buffer, uint8_t len){
	//uint8_t getInfoBuf [] = {0x67, 0x65, 0x74}; // get message (ASCII for "get")
	//BT_transmitData(3, getInfoBuf);
	
	/*
	if(EUSART1_Read_Block_TimeOut(len, time_out_receiving_data) == HAL_TIMEOUT){
		return 0;
	}
	else {
		for(int i = 0; i < len; i++){
			*(buffer + i) = rx_buffer[i];
		}
		return 1;
	}*/
	
	return 1;
}


//  Requests

void BT_getReq(UART_HandleTypeDef *haurt, uint8_t index){
    BT_transmitFrame(haurt, CMD_GET_REQ, 1, &index);
}

void BT_getReqParam(UART_HandleTypeDef *haurt, uint8_t index, uint8_t param){
		uint8_t data [] = {index, param};
		BT_transmitFrame(haurt, CMD_GET_REQ, 2, data);
}

void BT_setReq(UART_HandleTypeDef *haurt, uint8_t index){
		BT_transmitFrame(haurt, CMD_SET_REQ, 1, &index);
}

void BT_setReqParam(UART_HandleTypeDef *haurt, uint8_t index, uint8_t param){
		uint8_t data [] = {index, param};
		BT_transmitFrame(haurt, CMD_SET_REQ, 2, data);
}

uint8_t BT_getConnectReq(UART_HandleTypeDef *haurt){
		uint8_t rx_buf [6];
		HAL_UART_Receive(haurt, rx_buf, 6, 100);
		if(0 == rx_buf[4]) 	return 1;
		else								return 0;
}
	


//	Indications

uint8_t BT_getConnectInd(UART_HandleTypeDef *haurt){
		uint8_t rx_buf [6];
		uint8_t status;
		if(HAL_UART_Receive(haurt, rx_buf, 12, 2000) == HAL_OK){
			if(0 == rx_buf[4]) 	return 1;
			else								return 0;
		}
		else status = 0;
		return status;
}

//	RSP

uint8_t BT_getChannelopenRSP(UART_HandleTypeDef *haurt){
		uint8_t rx_buf [6];
		uint8_t status;
		if(HAL_UART_Receive(haurt, rx_buf, 13, 2000) == HAL_OK){
			if(0 == rx_buf[4]) 	return 1;
			else								return 0;
		}
		else status = 0;
		return status;
}


//  Frames

uint8_t BT_transmitData(UART_HandleTypeDef *haurt, uint8_t len, uint8_t *data){
    return BT_transmitFrame(haurt, CMD_DATA_REQ, len, data);
}

uint8_t BT_transmitFrame(UART_HandleTypeDef *haurt, uint8_t cmd, uint16_t len, uint8_t *data){
		uint8_t tx_buf [5 + len];
    tx_buf [0] = 0x02;
    tx_buf [1] = cmd;
    tx_buf [2] = (uint8_t)len;
    tx_buf [3] = (uint8_t)(len >> 8);
    for(int i = 0; i < len; i++){
        tx_buf [i + 4] = *(data + i);
    }
		tx_buf [len + 4] = calculateCS(tx_buf, len + 4);
		return HAL_UART_Transmit(haurt, tx_buf, (len + 5), 1000);
}

uint8_t calculateCS(uint8_t *data, uint8_t len){
    uint8_t checksum = *(data);
    for(uint8_t i = 1; i < len; i++){
        checksum = checksum ^ *(data + i);
    }
    return checksum;
}



