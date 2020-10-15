/**
  ******************************************************************************
  * @file    ble.c
  * @brief   STM32H7432ZI communicates with Proteus-II BT module
	******************************************************************************
 **/


#include "proteusII_driver.h"

extern UART_HandleTypeDef huart5; 

// ================================================================
// ===            		Init and other functions               		===
// ================================================================

void BT_Initialize(void){
	
}

void BT_getMAC(UART_HandleTypeDef *huart){
    //BT_getReq(huart, FS_BTMAC);
    //EUSART1_Read_Block(12);
		//for(uint8_t i = 0; i < 6; i++)
    //    bt_mac [i] = rx_buffer [5 + i];  
}


// ================================================================
// ===            Connection management functions               ===
// ================================================================

void BT_connect(UART_HandleTypeDef *huart, uint8_t *mac){
	uint8_t mac_reverse_buffer [6];
	for(uint8_t i = 0; i < 6; i++)
		mac_reverse_buffer [5 - i] = *(mac + i);
	BT_transmit_CMD_Bytes(huart, CMD_CONNECT_REQ, 6, mac_reverse_buffer);
}




void BT_disconnect(UART_HandleTypeDef *huart){
	BT_transmit_CMD(huart, CMD_DISCONNECT_REQ);
}


// ================================================================
// ===               Scan management functions                  ===
// ================================================================

void BT_changeScanTiming(UART_HandleTypeDef *huart){
	uint8_t buf [] = {RF_ScanTiming, 0x00};
	BT_transmit_CMD_Bytes(huart, CMD_SET_REQ, 2, buf);
	HAL_Delay(200);
}

void BT_changeScanFactor(UART_HandleTypeDef *huart){
	uint8_t buf [] = {RF_ScanFactor, 0x01};
	BT_transmit_CMD_Bytes(huart, CMD_SET_REQ, 2, buf);
	HAL_Delay(200);
}

void BT_startScanning(UART_HandleTypeDef *huart){
	/*
	uint8_t buf [] = {RF_BeaconFlags, 0x01};
	BT_transmit_CMD_Bytes(huart, CMD_SET_REQ, 2, buf);
	HAL_Delay(200);
	*/
	BT_transmit_CMD(huart, CMD_SCANSTART_REQ);
}

void BT_stopScanning(UART_HandleTypeDef *huart){
	BT_transmit_CMD(huart, CMD_SCANSTOP_REQ);
}


// ================================================================
// ===              Baudrate management functions               ===
// ================================================================

void BT_updateBaudrate(UART_HandleTypeDef *huart, uint8_t baudrate_index){
    
}


uint8_t BT_getUARTBaudrate(UART_HandleTypeDef *huart){
  BT_transmit_CMD_Byte(huart, CMD_GET_REQ, UART_BaudrateIndex);
	return 0;
}

uint8_t BT_setUARTBaudrate(UART_HandleTypeDef *huart, uint8_t baudrate_index){
	uint8_t data [] = {UART_BaudrateIndex, baudrate_index};
  BT_transmit_CMD_Bytes(huart, CMD_SET_REQ, 2, data);
	return 0;
}

// ================================================================
// ===            Send message through Bluetooth                ===
// ================================================================

void BT_transmitMsg_CMD(UART_HandleTypeDef *huart, uint8_t communication_cmd){
    BT_transmitData(huart, 1, &communication_cmd);
}

void BT_transmitMsg_CMD_Data(UART_HandleTypeDef *huart, uint8_t communication_cmd, uint8_t len, uint8_t * sndbuf){
    uint8_t data [1 + len];
    data [0] = communication_cmd;
    for(uint8_t i = 0; i < len; i++){
        data [1 + i] = *(sndbuf + i);
    }
    BT_transmitData(huart, len + 1, data);
}

// ================================================================
// ===            Transmit messages to BT modules               ===
// ================================================================

void BT_transmitData(UART_HandleTypeDef *huart, uint8_t len, uint8_t *data){
    BT_transmit_CMD_Bytes(huart, CMD_DATA_REQ, len, data);
}

void BT_transmit_CMD_Bytes(UART_HandleTypeDef *huart, uint8_t cmd, uint16_t len, uint8_t *data){
		uint8_t tx_buffer [5 + len];
    tx_buffer [0] = 0x02;
    tx_buffer [1] = cmd;
    tx_buffer [2] = len;
    tx_buffer [3] = (uint8_t)(len >> 8);
    for(int i = 0; i < len; i++){
        tx_buffer [i + 4] = *(data + i);
    }
		tx_buffer [len + 4] = BT_calculateCS(tx_buffer, len + 4);
		
		UART_COM_write(huart, tx_buffer, (len + 5));
}

void BT_transmit_CMD_Byte(UART_HandleTypeDef *huart, uint8_t cmd, uint8_t data){
    uint8_t tx_buffer [6];
    tx_buffer [0] = 0x02;
    tx_buffer [1] = cmd;
    tx_buffer [2] = 0x01;
    tx_buffer [3] = 0x00;
    tx_buffer [4] = data;
		tx_buffer [5] = BT_calculateCS(tx_buffer, 5);
		UART_COM_write(huart, tx_buffer, 6);
}

void BT_transmit_CMD(UART_HandleTypeDef *huart, uint8_t cmd){
    uint8_t tx_buffer [5];
    tx_buffer [0] = 0x02;
    tx_buffer [1] = cmd;
    tx_buffer [2] = 0x00;
    tx_buffer [3] = 0x00;
		tx_buffer [4] = BT_calculateCS(tx_buffer, 4);
		UART_COM_write(huart, tx_buffer, 5);
}


// ================================================================
// ===              Receive BT module messages                  ===
// ================================================================
/*
uint8_t BT_receiveFrame_TimeOut(uint8_t * rsvbuf, uint16_t timeout){
    uint32_t begin_time = HAL_GetTick();
    while(!BT_receiveFrame(rsvbuf)){
        if((HAL_GetTick() - begin_time) > timeout){
            return 0;
            break;
        }
    }
    return 1;
}

uint8_t BT_receiveFrame(uint8_t * rsvbuf){
	
	
    if(Serial.available() > 0){
        if(Serial.read() == 0x02){
            *(rsvbuf + 0) = 0x02;
            *(rsvbuf + 1) = Serial.read();
            *(rsvbuf + 2) = Serial.read();
            *(rsvbuf + 3) = Serial.read();
            uint16_t len = *(rsvbuf + 2) | (*(rsvbuf + 3) << 8);

            //Serial.write(0x99);

            //  The incoming data
            for(uint8_t i = 0; i < len; i++){
                *(rsvbuf + 4 + i) = Serial.read();
            }

            uint8_t total_len = len + 4;

            //for(uint8_t i = 0; i < total_len; i++){
            //    Serial.write(*(rsvbuf + i));
            //}
           
            if(calculateCS(rsvbuf, total_len) == Serial.read()){
                return 1;
            }
        }
    }   
    return 0;
		
}
*/

// ================================================================
// ===                      UART functions                      ===
// ================================================================


uint8_t BT_calculateCS(uint8_t *data, uint8_t len){
    uint8_t checksum = *(data);
    for(uint8_t i = 1; i < len; i++){
        checksum = checksum ^ *(data + i);
    }
    return checksum;
}



