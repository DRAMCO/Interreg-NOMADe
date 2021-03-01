
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
            //if(*(rsv_buffer + 4) == 0x00) tx_packet_succes_counter++;
            //else tx_packet_failed_counter++;
        }
        break;

        case CMD_GETSTATE_CNF:{
            //  Current module state
            baud_correct = true;
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

    case IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS_WITHOUT_SYNC:{
      if(calibration){
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_MEASUREMENTS_STARTED_WITHOUT_SYNC);
        digitalWrite(IND_LED, LOW);
        counters->packet_send_counter = 0;
        mpu->setDMPEnabled(true);    //    Start IMU DMP
        state = INIT_MES;
      }
      else{
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_NEED_TO_CALIBRATE);
      }
      
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
        state = INIT_MES;
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
        state = INIT_MES;
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

    case IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_CHANGED:{
      uint8_t sample_freq = *(rsv_buffer + 12);
      switch(sample_freq){
        case DATA_FORMAT_1: { counters->pack_nr_before_send = IMU_SAMPLING_FREQ / 10;   } break;
        case DATA_FORMAT_2: { counters->pack_nr_before_send = IMU_SAMPLING_FREQ / 20;   } break;
        case DATA_FORMAT_3: { counters->pack_nr_before_send = IMU_SAMPLING_FREQ / 25;   } break;
        case DATA_FORMAT_4: { counters->pack_nr_before_send = IMU_SAMPLING_FREQ / 50;   } break;
        case DATA_FORMAT_5: { counters->pack_nr_before_send = IMU_SAMPLING_FREQ / 100;  } break;
      }
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED, 1, &sample_freq);
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

    case IMU_SENSOR_MODULE_REQ_CHANGE_DF:{
      uint8_t format_nr = *(rsv_buffer + 12);
      switch(format_nr){
        case DATA_FORMAT_1: { dataformat = QUAT;            } break;
        case DATA_FORMAT_2: { dataformat = GYRO;            } break;
        case DATA_FORMAT_3: { dataformat = ACC;             } break;
        case DATA_FORMAT_4: { dataformat = GYRO_ACC;        } break;
        case DATA_FORMAT_5: { dataformat = QUART_GYRO_ACC;  } break;
      }
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_DF_CHANGED, 1, &format_nr);
    } break;

    case IMU_SENSOR_MODULE_REQ_SW_VERSION:{
      uint8_t software_version = SW_VERSION;
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_RSP_SW_VERSION, 1, &software_version);
    } break;


    default:{
    } break;

  }

}

void BTCOM::update_baudrate(){
  if(baud_pointer < 4)  baud_pointer++;
  else                  baud_pointer = 3;
  Serial.flush();
  Serial.begin(baudrate_array[baud_pointer]);
  bt->reset();
}


void BTCOM::controle_check_baudrate(){
    //  Check baudrate is matching
  if(!baud_correct){
    delay(10);
    if(check_baud){
      check_baud = false;
      update_baudrate();
    }
    else{
      if(counter_baud_check > 50){
        counter_baud_check = 0;
        check_baud = true;
      }
      counter_baud_check++;
    }
  } 

    //  Update baudrate
  if(baud_correct && !baud_changed){
    if(baud_pointer != BT_UART_BAUDRATE){
      bt->setUARTBaudrate(BT_UART_BAUDRATE);
      delay(500);
      baud_correct = false;
    }
    baud_changed = true;
    
      //  Eénmalig uitvoeren
    bt->changeScanTiming();    // Not Low Power !!!
    bt->changeScanFactor();    // Not Low Power !!!
  }
}

bool BTCOM::baudrate_ok(){
  return (baud_correct && baud_changed);
}