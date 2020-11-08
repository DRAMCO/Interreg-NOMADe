
#include "Arduino.h"
#include "BTCom.h"

BTCOM::BTCOM(BLUETOOTH *bt, BMS *bms, MPU6050 *mpu, Variables *counters){
    this->bt = bt;
    this->bms = bms;
    this->mpu = mpu;
    this->counters = counters;
}


void BTCOM::incoming_data_handler(){
  if(Serial.available() > 0){
    uint8_t rsvbuf [100];
    if(Serial.read() == 0x02){
      rsvbuf [0] = 0x02;
      delay(1);
      rsvbuf [1] = Serial.read();
      rsvbuf [2] = Serial.read();
      rsvbuf [3] = Serial.read();
      uint16_t len = rsvbuf [2] | (rsvbuf [3] << 8);

      delay(1);

      //  The incoming data
      if(len < 255){
        uint8_t i;
        for(i = 0; i < len; i++){
          if(Serial.available() > 0){
            rsvbuf [4 + i] = Serial.read();
          }
          else break;
        }
        if(i == len){
          uint16_t total_len = len + 4;
          if(bt->calculateCS(rsvbuf, total_len) == Serial.read()){
            communication_management(rsvbuf);
          }
        }
      }
    }
  } 
}

void BTCOM::communication_management(uint8_t *rsv_buffer){
    //uint8_t rsv_buffer [50];
    //if(bt->receiveFrame(rsv_buffer)){
      switch(*(rsv_buffer + 1)){
        case CMD_RESET_CNF:{
            //  Reset request received
        }
        break;
        case CMD_DATA_CNF:{
            //  Data transmission request received
        }
        break;

        case CMD_TXCOMPLETE_RSP:{
            //  Data has been sent
        }
        break;

        case CMD_GETSTATE_CNF:{
            //  Current module state
        }
        break;

        case CMD_CONNECT_IND:{
            //  Connection established
        }
        break;

        case CMD_CHANNELOPEN_RSP:{
            //  Channel open, data transmission possible
        }
        break;

        case CMD_DISCONNECT_CNF:{
            //  Disconnection request received
        }
        break;
        
        case CMD_DISCONNECT_IND:{
            //  Disconnected
            startup_time = millis();
            //state = WAIT_FOR_CONNECTION;
        }
        break;

        case CMD_SLEEP_CNF:{
            //  Sleep request received
            if(*(rsv_buffer + 4) == 0xFF){
              //bt->softReset();
              delay(100);
            }
        }
        break;

        case CMD_SET_CNF:{
            //  Module flash settings have been modified
        }
        break;

        case CMD_DATA_IND:{
            //  Data has been received
            rsv_msg_handler(rsv_buffer);
        }
        break;

        case CMD_SCANSTART_CNF:{
            //  Scanning started
        }
        break;

        case CMD_SCANSTOP_CNF:{
            //  Scanning stopped
        }
        break;

        case CMD_BEACON_IND:{
          if(*(rsv_buffer + 11) == ADV_MSG){
            if(!sync_executed) sync_time = millis();
            sync_executed = 1;
            bt->stopScanning();
            state = IDLE;
          }
        }
        break;

        default:
            break;
        }


        //if(rsv_buffer [1] != 0)
        //  Serial.write(rsv_buffer [1]);
    //}
}

void BTCOM::rsv_msg_handler(uint8_t *rsv_buffer){
  //switch(*command_msg){
  switch(*(rsv_buffer + 11)){

    case IMU_SENSOR_MODULE_REQ_STATUS:{
        //  Send module status
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_STATUS, 1, &state);
    } break;

    case IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE:{
        //  Send battery voltage
      float voltage = bms->getBatVoltage();
      uint16_t value = (uint16_t)(voltage * 100);
      uint8_t data [] = {IMU_SENSOR_MODULE_IND_BATTERY_VOLTAGE, (uint8_t)(value), (uint8_t)(value >> 8)};
      bt->transmitData(3, data);
    } break;

    case IMU_SENSOR_MODULE_REQ_START_CALIBRATION:{
      if(state == IDLE){
        state = CALIBRATION;      /***    Start calibration     ***/
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_CALIBRATION_STARTED);
      }
      else{
          //  Send msg, cannot calibrate!!
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_CANNOT_CALIBRATE);
      }
    } break;

    case IMU_SENSOR_MODULE_REQ_START_SYNC:{
      sync_executed = 0;
      if(state == IDLE){
          //  Send msg, SYNC started
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SYNC_STARTED);
        delay(50); 
        sync_now = 1;
        state = SYNC;
      }  
      else{
        //  Send msg, cannot synchronise!!
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_CANNOT_SYNC);
      }
    } break;

    case IMU_SENSOR_MODULE_REQ_GET_SYNC_TIME:{
      //uint8_t value = (millis() - sync_time + 1000)/1000;
      //uint32_t starttime = sync_time + (1000 * value);
      uint16_t value = (*(rsv_buffer + 12) | *(rsv_buffer + 13) << 8);
      uint32_t starttime = sync_time + (1000 * value);
       if(starttime - millis() < 5000) while(millis() < starttime);

      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SYNC_TIME);
    } break;

    case IMU_SENSOR_MODULE_REQ_CHANGE_SYNC_TIME:{
      int8_t adaptation_value = *(rsv_buffer + 12);
      sync_time = sync_time + (adaptation_value * 250);
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SYNC_TIME_CHANGED);
    } break;

    case IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS:{

      if(calibration && synchronisation){
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_MEASUREMENTS_STARTED);

        digitalWrite(IND_LED, LOW);

        counters->packet_send_counter = 0;

        uint8_t value = (millis() - sync_time + 1000)/1000;
        uint32_t starttime = sync_time + (1000 * value);
        while(millis() < starttime);

        //uint16_t value = (*(rsv_buffer + 12) | *(rsv_buffer + 13) << 8);
        //uint32_t starttime = sync_time + (1000 * value);
        //if(starttime - millis() < 5000) while(millis() < starttime);
        
        mpu->setDMPEnabled(true);    //    Start IMU DMP
        state = RUNNING;
      }
      else{
        if(!synchronisation){
            //  Send msg, first need to synchronise.
          bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_NEED_TO_SYNCHRONISE);
        }
        if(!calibration){
            //  Send msg, first need to callibrate.
          bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_NEED_TO_CALIBRATE);
        }
      }

      /*
      // Start measurements
      if(calibration){
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_MEASUREMENTS_STARTED);

        digitalWrite(IND_LED, LOW);

        packet_send_counter = 0;

        uint8_t value = (millis() - sync_time + 1000)/1000;
        uint32_t starttime = sync_time + (1000 * value);

        while(millis() < starttime);
        
        mpu->setDMPEnabled(true);    //    Start IMU DMP
        state = RUNNING;
      }
      else{
          //  Send msg, first need to callibrate.
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_NEED_TO_CALIBRATE);
      }*/
    } break;

    case IMU_SENSOR_MODULE_REQ_STOP_MEASUREMENTS:{
      mpu->setDMPEnabled(0);
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_MEASUREMENTS_STOPPED);
      digitalWrite(IND_LED, LOW);

      sync_executed = 0;

      counters->mpu_read_counter = 0;
      counters->buffer_counter = 0;
      counters->packet_send_counter = 0;

      state = IDLE;
    } break;

    case IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_10HZ:{
      counters->pack_nr_before_send = IMU_SAMPLING_FREQ / 10;
        //  Send msg, sampling frequency changed
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED);
    } break;

    case IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_20HZ:{
      counters->pack_nr_before_send = IMU_SAMPLING_FREQ / 20;
        //  Send msg, sampling frequency changed
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED);
    } break;

    case IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_25HZ:{
      counters->pack_nr_before_send = IMU_SAMPLING_FREQ / 25;
        //  Send msg, sampling frequency changed
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED);
    } break;

    case IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_50HZ:{
      counters->pack_nr_before_send = IMU_SAMPLING_FREQ / 50;
        //  Send msg, sampling frequency changed
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED);
    }
    break;

    case IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_100HZ:{
      counters->pack_nr_before_send = IMU_SAMPLING_FREQ / 100;
        //  Send msg, sampling frequency changed
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED);
    } break;

    case IMU_SENSOR_MODULE_REQ_GO_TO_SLEEP:{
        //  Send msg, Module State is SLEEP
      mpu->setDMPEnabled(0);
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SLEEP_MODE);
      delay(200);
      state = SLEEP;
    } break;

    case IMU_SENSOR_MODULE_REQ_MILLIS:{
        //  Send current system ticks
      uint32_t ticks = millis();
      uint8_t data [] = {IMU_SENSOR_MODULE_RSP_MILLIS, (uint8_t)(ticks), (uint8_t)(ticks >> 8), (uint8_t)(ticks >> 16), (uint8_t)(ticks >> 24)};
      bt->transmitData(5, data);
    } break;

    default:{
    } break;

  }

}