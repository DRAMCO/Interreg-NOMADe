#include "Arduino.h"
#include "Bluetooth.h"

BLUETOOTH::BLUETOOTH(){

}

void BLUETOOTH::init(){
    pinMode(BT_CTS, INPUT);
    pinMode(BT_RTS, INPUT);
    pinMode(BT_STATUS, INPUT);
    pinMode(BT_RES, OUTPUT);
    pinMode(WAKE_UP_PIN, OUTPUT);
    digitalWrite(WAKE_UP_PIN, HIGH);
}


void BLUETOOTH::reset(){
    digitalWrite(BT_RES, LOW);
    delay(100);
    digitalWrite(BT_RES, HIGH);
}

uint8_t BLUETOOTH::isConnected(){
    return digitalRead(BT_STATUS);
}

void BLUETOOTH::disconnect(){
    transmitFrame(CMD_DISCONNECT_REQ, 0, NULL);
    // NIET de beste manier, beter effectief uitlezen wat de BT module zegt
    while(isConnected()){
        transmitFrame(CMD_DISCONNECT_REQ, 0, NULL);
        delay(200);
    }
}


void BLUETOOTH::transmitData(uint8_t len, uint8_t *data){
    transmitFrame(CMD_DATA_REQ, len, data);
}

void BLUETOOTH::transmitFrame(uint8_t cmd, uint16_t len, uint8_t * data){
    uint8_t tx_buffer [len + 4];
    tx_buffer [0] = 0x02;
    tx_buffer [1] = cmd;
    tx_buffer [2] = (uint8_t)len;
    tx_buffer [3] = (uint8_t)(len >> 8);
    for(uint8_t i = 0; i < len; i++){
        tx_buffer [i + 4] = *(data + i);
    }
    
    UART_Write_Block(tx_buffer, (4 + len));
}

void BLUETOOTH::transmitFrame(uint8_t cmd, uint8_t data){
    uint8_t tx_buffer [5];
    tx_buffer [0] = 0x02;
    tx_buffer [1] = cmd;
    tx_buffer [2] = 0x01;
    tx_buffer [3] = 0x00;
    tx_buffer [4] = data;
    UART_Write_Block(tx_buffer, 5);
}

void BLUETOOTH::UART_Write_Block(uint8_t * data, uint8_t len){
    uint8_t cs = calculateCS(data, len);
    Serial.write(data, len);
    Serial.write(cs);
}

uint8_t BLUETOOTH::calculateCS(uint8_t * data, uint8_t len){
    uint8_t checksum = *(data);
    for(uint8_t i = 1; i < len; i++){
        checksum = checksum ^ *(data + i);
    }
    return checksum;
}


void BLUETOOTH::sleep_mode(void){
  transmitFrame(CMD_SLEEP_REQ, NULL, NULL);
}

void BLUETOOTH::wakeup(void){
  digitalWrite(WAKE_UP_PIN, LOW);
  delay(10);
  digitalWrite(WAKE_UP_PIN, HIGH);
}