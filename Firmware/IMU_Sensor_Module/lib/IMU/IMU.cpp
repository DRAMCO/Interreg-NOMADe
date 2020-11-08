#include "Arduino.h"
#include "IMU.h"

static void Error_Handler(void);

IMU::IMU(DMP *dmp, MPU6050 *mpu, Variables *counters){
    this->mpu = mpu;
    this->dmp = dmp;
    this->counters = counters;
}

void IMU::powerdown(void){
  digitalWrite(MPU_ON, LOW);
}

void IMU::powerup(void){
  digitalWrite(MPU_ON, HIGH);
}

uint8_t IMU::getIMUData(uint8_t *len){
  uint8_t return_val = 0;


  // MOET DYNAMISCH AANPASBAAR WORDEN !!!
  uint8_t sample_data_len = SAMPLE_DATA_LEN;





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
      //  Read a packet from FIFO
    while(fifoCount >= PACKETSIZE){ // Lets catch up to NOW, someone is using the dreaded delay()!
      mpu->getFIFOBytes(fifoBuffer, PACKETSIZE);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
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

      *len = sample_data_len * MAX_BUFFER_SIZE + 7;

      counters->packet_send_counter++;
      return_val = 1;
    }

    //sample_data_len = 8 // Only quaternions sended
    //sample_data_len = 8 // all data sended QUAT + GYRO + ACC

    uint8_t offset_buf [20] = {0, 1, 4, 5, 8, 9, 12, 13, 16, 17, 20, 21, 24, 25, 28, 29, 32, 33, 36, 37};
    for(uint8_t i = 0; i < sample_data_len; i++){
      data_buffer [7 + i + sample_data_len*counters->buffer_counter] = fifoBuffer[offset_buf[i]]; 
    }
/*
    data_buffer [7 + 8*counters->buffer_counter] = fifoBuffer[0];  
    data_buffer [8 + 8*counters->buffer_counter] = fifoBuffer[1];
    data_buffer [9 + 8*counters->buffer_counter] = fifoBuffer[4];
    data_buffer [10 + 8*counters->buffer_counter] = fifoBuffer[5];
    data_buffer [11 + 8*counters->buffer_counter] = fifoBuffer[8];
    data_buffer [12 + 8*counters->buffer_counter] = fifoBuffer[9];
    data_buffer [13 + 8*counters->buffer_counter] = fifoBuffer[12];
    data_buffer [14 + 8*counters->buffer_counter] = fifoBuffer[13];
*/  
    counters->packet_number_counter = 1;
    counters->buffer_counter++;
  }
  return return_val;
}

/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */
/*
    
    uint8_t offset_buf [20] = {0, 1, 4, 5, 8, 9, 12, 13, 16, 17, 20, 21, 24, 25, 28, 29, 32, 33, 36, 37};

    for(uint8_t i = 0; i < 20; i++){
      data_buffer [7 + i + sample_data_len*counters->buffer_counter] = fifoBuffer[offset_buffer[i]]; 
    }

    data_buffer [7 + 8*counters->buffer_counter] = fifoBuffer[0];  
    data_buffer [8 + 8*counters->buffer_counter] = fifoBuffer[1];
    data_buffer [9 + 8*counters->buffer_counter] = fifoBuffer[4];
    data_buffer [10 + 8*counters->buffer_counter] = fifoBuffer[5];
    data_buffer [11 + 8*counters->buffer_counter] = fifoBuffer[8];
    data_buffer [12 + 8*counters->buffer_counter] = fifoBuffer[9];
    data_buffer [13 + 8*counters->buffer_counter] = fifoBuffer[12];
    data_buffer [14 + 8*counters->buffer_counter] = fifoBuffer[13];

*/


uint8_t* IMU::getDataBuffer(){
  return data_buffer;
}

void IMU::reset_counters(void){
    counters->buffer_counter = 0; 
    counters->packet_send_counter = 0;
    counters->mpu_read_counter = 0;
}

void IMU::calibrate(void){
  #ifdef DEBUG
    DEBUG Serial.println(F("Initializing I2C devices...")); 
  #endif

  mpu->initialize();

  //  Verify connection
  #ifdef DEBUG 
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.eavailable());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again
  #endif

  //  Load and configure the DMP
  #ifdef DEBUG
    Serial.println(F("Initializing DMP..."));
  #endif

  uint8_t devStatus;                  //  Return status after each device operation (0 = success, !0 = error)
  devStatus = dmp->initialize();

  #ifdef MPU_EXTERNAL_CLOCK
    delay(10);
    mpu->setClockSource(MPU6050_CLOCK_PLL_EXT19M);
  #endif

  //  Supply your own gyro offsets here, scaled for min sensitivity
  mpu->setXGyroOffset(220);
  mpu->setYGyroOffset(76);
  mpu->setZGyroOffset(-85);
  mpu->setZAccelOffset(1788); // 1688 factory default for my test chip


  //  Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      //LED_blink(1, 100);
      mpu->CalibrateAccel(6);
      //LED_blink(1, 100);
      mpu->CalibrateGyro(6);
      //LED_blink(1, 100);
      //mpu.PrintActiveOffsets();

      #ifdef DEBUG 
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
      #endif

      //mpu.setDMPEnabled(true);

      #ifdef DEBUG
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(MPU_INT));
        Serial.println(F(")..."));
      #endif

      //attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
      //mpuIntStatus = mpu.getIntStatus();

      #ifdef DEBUG
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
      #endif

      //dmpReady = true;

      // get expected DMP packet size for later comparison
      //packetSize = mpu.dmpGetFIFOPacketSize();
  }
  #ifdef DEBUG 
    else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  #endif

  // if programming failed, don't try to do anything
  #ifdef DEBUG 
    if (!dmpReady) Error_Handler();
  #endif
}




void Error_Handler(void){
  state = SLEEP;
}



