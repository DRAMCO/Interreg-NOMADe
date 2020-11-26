#include "Arduino.h"
#include "IMU.h"

static void Error_Handler(void);

IMU::IMU(DMP *dmp, MPU6050 *mpu, Variables *counters, LED *led){
    this->mpu = mpu;
    this->dmp = dmp;
    this->counters = counters;
    this->led = led;
}

void IMU::powerdown(void){
  digitalWrite(MPU_ON, LOW);
}

void IMU::powerup(void){
  digitalWrite(MPU_ON, HIGH);
}

void IMU::initMeasurement(){
  switch(dataformat){
    case QUAT:{               pstruc_start = 0;   pstruc_stop = SAMPLE_DATA_LEN_QUATERNIONS;  } break;  //  Only QUAT sended
    case GYRO:{               pstruc_start = 8;   pstruc_stop = 14;                           } break;  //  Only GYRO sended
    case ACC:{                pstruc_start = 14;  pstruc_stop = SAMPLE_DATA_LEN_ALL;          } break;  //  Only ACC sended
    case GYRO_ACC:{           pstruc_start = 8;   pstruc_stop = SAMPLE_DATA_LEN_ALL;          } break;  //  Only GYRO + ACC sended
    case QUART_GYRO_ACC:{     pstruc_start = 0;   pstruc_stop = SAMPLE_DATA_LEN_ALL;          } break;  //  All data is sended QUAT + GYRO + ACC
  }
}

uint8_t IMU::getIMUData(uint8_t *len, uint8_t *send_buffer){
  uint8_t return_val = 0;

    //  Reset the FIFO Buffer if we don't want to save this data
  if(counters->packet_number_counter < counters->pack_nr_before_send){
    mpu->resetFIFO();
    counters->packet_number_counter++;
    return 0;
  }

  uint16_t fifoCount;                 //  Count of all bytes currently in FIFO
  uint8_t fifoBuffer[64];             //  FIFO storage buffer
  
  fifoCount = mpu->getFIFOCount();
  uint8_t mpuIntStatus = mpu->getIntStatus();
	
    //  Check the number of bytes in the FIFO buffer 
  if (fifoCount > PACKETSIZE) Error_Handler();
  
    //  Check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu->resetFIFO();
    #ifdef DEBUG
      Serial.println(F("FIFO overflow!"));
    #endif
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }


  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)){
      //  Read DMP packet from FIFO buffer
    while(fifoCount >= PACKETSIZE){
      mpu->getFIFOBytes(fifoBuffer, PACKETSIZE);
      fifoCount -= PACKETSIZE;
    }

    if(counters->buffer_counter >= MAX_BUFFER_SIZE){
      counters->buffer_counter = 0;
      uint32_t time_at_the_moment = millis();
      data_buffer [1] = (uint8_t)counters->packet_send_counter;
      data_buffer [2] = (uint8_t)(counters->packet_send_counter >> 8);
      data_buffer [3] = (uint8_t)time_at_the_moment;
      data_buffer [4] = (uint8_t)(time_at_the_moment >> 8);
      data_buffer [5] = (uint8_t)(time_at_the_moment >> 16);
      data_buffer [6] = (uint8_t)(time_at_the_moment >> 24);

      *len = (pstruc_stop - pstruc_start) * MAX_BUFFER_SIZE + 7;

      memcpy(send_buffer, data_buffer, *len);

      counters->packet_send_counter++;
      return_val = 1;
    }

    /* ================================================================================================ *
    | MPU6050 42-byte FIFO packet structure:                                                           |
    |                                                                                                  |
    | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
    |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
    |                                                                                                  |
    | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
    |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
    * ================================================================================================ */

    uint8_t offset_buf [20] = {0, 1, 4, 5, 8, 9, 12, 13, 16, 17, 20, 21, 24, 25, 28, 29, 32, 33, 36, 37};
    for(uint8_t i = pstruc_start; i < pstruc_stop; i++){
      data_buffer [7 + (i - pstruc_start) + (pstruc_stop-pstruc_start)*counters->buffer_counter] = fifoBuffer[offset_buf[i]]; 
    }

    counters->packet_number_counter = 1;
    counters->buffer_counter++;
  }
  return return_val;
}

uint8_t* IMU::getDataBuffer(){
  return data_buffer;
}

void IMU::reset_counters(void){
    counters->buffer_counter = 0; 
    counters->packet_send_counter = 0;
    counters->mpu_read_counter = 0;
}

void IMU::calibrate(void){
    //  Return statusbyte [0 = success]
  uint8_t devStatus;

    //  Init MPU6050
  mpu->initialize();

    //  Init DMP
  devStatus = dmp->initialize();

    //  Set external CLK Oscillator 19.2MHz
  #ifdef MPU_EXTERNAL_CLOCK
    delay(10);
    mpu->setClockSource(MPU6050_CLOCK_PLL_EXT19M);
  #endif

    //  Set gyro offsets (scaled for minimal sensitivity)
  mpu->setXGyroOffset(220);
  mpu->setYGyroOffset(76);
  mpu->setZGyroOffset(-85);
  mpu->setZAccelOffset(1788); // 1688 factory default setting


  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    led->blink(1, 100);
    mpu->CalibrateAccel(6);
    led->blink(1, 100);
    mpu->CalibrateGyro(6);
    led->blink(1, 100);
  }
}


void Error_Handler(void){
  state = SLEEP;
}



