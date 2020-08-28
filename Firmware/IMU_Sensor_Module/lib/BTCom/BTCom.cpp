
#include "Arduino.h"
#include "BTCom.h"

BTCOM::BTCOM(BLUETOOTH * bt, BMS * bms, MPU6050 * mpu){
    this->bt = bt;
    this->bms = bms;
    this->mpu = mpu;
}

void BTCOM::communication_management(){
    uint8_t rsv_buffer [50];
    if(bt->receiveFrame(rsv_buffer)){
      switch(rsv_buffer [1]){
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
        }
        break;

        case CMD_SLEEP_CNF:{
            //  Sleep request received
            if(rsv_buffer[4] == 0xFF){
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
            rsv_msg_handler(&rsv_buffer[11]);
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
            //  Check if the "SYNC" command is send
          //if(rsv_buffer[11] == 0x83 && rsv_buffer[12] == 0x89 && rsv_buffer[13] == 0x78 && rsv_buffer[14] == 0x67){
          //  if(!sync_executed) sync_time = millis();
          //  sync_executed = 1;
          //  bt->stopScanning();
          //}
        }
        break;

        default:
            break;
        }


        //if(rsv_buffer [1] != 0)
        //  Serial.write(rsv_buffer [1]);
    }
}

void BTCOM::rsv_msg_handler(uint8_t * command_msg){
  switch(*command_msg){

    case IMU_SENSOR_MODULE_REQ_STATUS:{
        //  Send module status
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_STATUS, 1, &state);
    }
    break;

    case IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE:{
        //  Send battery voltage
      float voltage = bms->getBatVoltage();
      uint16_t value = (uint16_t)(voltage * 100);
      uint8_t data [] = {IMU_SENSOR_MODULE_IND_BATTERY_VOLTAGE, (uint8_t)(value), (uint8_t)(value >> 8)};
      bt->transmitData(3, data);
    }
    break;

    case IMU_SENSOR_MODULE_REQ_START_CALIBRATION:{
      if(state == IDLE){
        state = CALIBRATION;      /***    Start calibration     ***/
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_CALIBRATION_STARTED);
      }
      else{
          //  Send msg, cannot calibrate!!
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_CANNOT_CALIBRATE);
      }
    }
    break;

    case IMU_SENSOR_MODULE_REQ_START_SYNC:{
      // Start measurements
      if(calibration){
          //  Send msg, SYNC started
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SYNC_DONE);
        delay(50); 
        sync_now = 1;
        state = SYNC;
      }
      else{
          //  Send msg, first need to callibrate.
        bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_NEED_TO_CALIBRATE);
      }
    }
    break;

    case IMU_SENSOR_MODULE_REQ_STOP_MEASUREMENTS:{
      mpu->setDMPEnabled(0);
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_MEASUREMENTS_STOPPED);
      digitalWrite(IND_LED, LOW);
      mpu_read_counter = 0;
      buffer_counter = 0;
      sync_executed = 0;
      packet_send_number = 0;
      state = IDLE;
    }
    break;

    case IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_10HZ:{
      pack_nr_before_send = IMU_SAMPLING_FREQ / 10;
        //  Send msg, sampling frequency changed
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED);
    }
    break;

    case IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_20HZ:{
      pack_nr_before_send = IMU_SAMPLING_FREQ / 20;
        //  Send msg, sampling frequency changed
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED);
    }
    break;

    case IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_25HZ:{
      pack_nr_before_send = IMU_SAMPLING_FREQ / 25;
        //  Send msg, sampling frequency changed
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED);
    }
    break;

    case IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_50HZ:{
      pack_nr_before_send = IMU_SAMPLING_FREQ / 50;
        //  Send msg, sampling frequency changed
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED);
    }
    break;

    case IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_100HZ:{
      pack_nr_before_send = IMU_SAMPLING_FREQ / 100;
        //  Send msg, sampling frequency changed
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED);
    }
    break;

    case IMU_SENSOR_MODULE_REQ_GO_TO_SLEEP:{
        //  Send msg, Module State is SLEEP
      mpu->setDMPEnabled(0);
      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_SLEEP_MODE);
      delay(200);
      state = SLEEP;
    }
    break;

    case IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS:{

      bt->transmitFrameMsg(IMU_SENSOR_MODULE_IND_MEASUREMENTS_STARTED);

      digitalWrite(IND_LED, LOW);

      packet_send_number = 0;

      uint8_t value = (millis() - sync_time + 1000)/1000;
      uint32_t starttime = sync_time + (1000 * value);

      while(millis() < starttime);
      
      mpu->setDMPEnabled(true);    //    Start IMU DMP
      state = RUNNING;

    }
    break;

    case ADV_MSG:{
      if(!sync_executed) sync_time = millis();
      sync_executed = 1;
      bt->stopScanning();
      state = IDLE;
    }
    break;

    default:{
      Serial.write(0xFF);
    }
    break;

  }

}