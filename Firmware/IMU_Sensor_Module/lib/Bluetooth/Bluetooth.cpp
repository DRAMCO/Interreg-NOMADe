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
    wakeup();
}

void BLUETOOTH::reset(){
    digitalWrite(BT_RES, LOW);
    delay(100);
    digitalWrite(BT_RES, HIGH);
}

void BLUETOOTH::softReset(){
    transmitFrame(CMD_RESET_REQ);
}

void BLUETOOTH::sleep_mode(void){
  transmitFrame(CMD_SLEEP_REQ);
}

void BLUETOOTH::wakeup(void){
  digitalWrite(WAKE_UP_PIN, LOW);
  delay(10);
  digitalWrite(WAKE_UP_PIN, HIGH);
}


// ================================================================
// ===            Connection management functions               ===
// ================================================================

uint8_t BLUETOOTH::isConnected(){
    return digitalRead(BT_STATUS);
}

void BLUETOOTH::disconnect(){
    transmitFrame(CMD_DISCONNECT_REQ);
    // NIET de beste manier, beter effectief uitlezen wat de BT module zegt
    delay(50); //200
    while(isConnected()){
        transmitFrame(CMD_DISCONNECT_REQ);
        delay(50); //200
    }
}

// ================================================================
// ===               Scan management functions                  ===
// ================================================================

void BLUETOOTH::changeScanTiming(){
    uint8_t buf [] = {RF_ScanTiming, 0x00};
    transmitFrame(CMD_SET_REQ, 2, buf);
    delay(200);
}

void BLUETOOTH::changeScanFactor(){
    uint8_t buf [] = {RF_ScanFactor, 0x01};
    transmitFrame(CMD_SET_REQ, 2, buf);
    delay(200);
}

void BLUETOOTH::startScanning(){
    
    uint8_t buf [] = {RF_BeaconFlags, 0x01};
    transmitFrame(CMD_SET_REQ, 2, buf);
    delay(100);

    transmitFrame(CMD_SCANSTART_REQ);
}

void BLUETOOTH::stopScanning(){
    transmitFrame(CMD_SCANSTOP_REQ);
}


// ================================================================
// ===              Baudrate management functions               ===
// ================================================================

void BLUETOOTH::getUARTBaudrate(){
    transmitFrame(CMD_GET_REQ, UART_BaudrateIndex);
}

void BLUETOOTH::setUARTBaudrate(uint8_t baudrate_index){
    uint8_t data [] = {UART_BaudrateIndex, baudrate_index};
    transmitFrame(CMD_SET_REQ, 2, data);
}

// ================================================================
// ===            Send message through Bluetooth                ===
// ================================================================

void BLUETOOTH::transmitFrameMsg(uint8_t communication_cmd){
    transmitData(1, &communication_cmd);
}

void BLUETOOTH::transmitFrameMsg(uint8_t communication_cmd, uint8_t len, uint8_t * sndbuf){
    uint8_t data [1 + len];
    data [0] = communication_cmd;
    for(uint8_t i = 0; i < len; i++){
        data [1 + i] = *(sndbuf + i);
    }
    transmitData(len + 1, data);
}

// ================================================================
// ===            Transmit messages to BT modules               ===
// ================================================================

void BLUETOOTH::transmitData(uint8_t len, uint8_t *data){
    transmitFrame(CMD_DATA_REQ, len, data);
}

void BLUETOOTH::transmitFrame(uint8_t cmd, uint16_t len, uint8_t *data){
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

void BLUETOOTH::transmitFrame(uint8_t cmd){
    uint8_t tx_buffer [5];
    tx_buffer [0] = 0x02;
    tx_buffer [1] = cmd;
    tx_buffer [2] = 0x00;
    tx_buffer [3] = 0x00;
    UART_Write_Block(tx_buffer, 4);
}

// ================================================================
// ===              Receive BT module messages                  ===
// ================================================================

bool BLUETOOTH::receiveFrame(uint8_t * rsvbuf, uint16_t timeout){
    uint32_t begin_time = millis();
    while(!receiveFrame(rsvbuf)){
        if((millis() - begin_time) > timeout){
            return 0;
            break;
        }
    }
    return 1;
}

bool BLUETOOTH::receiveFrame(uint8_t * rsvbuf){
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


// ================================================================
// ===                      UART functions                      ===
// ================================================================

    //  Change whem baudrate > 230400 -->> Implement Flow Control HERE
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