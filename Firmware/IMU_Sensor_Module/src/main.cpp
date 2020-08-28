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
 *         File: main.cpp
 *      Created: 2020-08-28
 *       Author: Jarne Van Mulders
 *      Version: V2.1
 *
 *  Description: Firmware IMU sensor module for the NOMADe project
 *
 *  Interreg France-Wallonie-Vlaanderen NOMADe
 *
 */


#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "LowPower.h"
#include "Config.h"
#include "BMS.h"
#include "Bluetooth.h"
#include "BTCom.h"
#include <avr/wdt.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define IMU

// ================================================================
// ===                      Create objects                      ===
// ================================================================

MPU6050 mpu;
BMS bms;
BLUETOOTH bt;
BTCOM btcom(&bt, &bms, &mpu);

// ================================================================
// ===                      State Machine                       ===
// ================================================================

uint8_t state = SLEEP;

// ================================================================
// ===                    Global booleans                       ===
// ================================================================

bool calibration = 0;
bool status_periferals = 1;
bool sync_now = 0;
bool sync_executed = 0;
bool battery_low_state = 0;
bool interruptIMU = false;

// ================================================================
// ===              Global buffers and variables                ===
// ================================================================

uint8_t buffer_counter = 0;         //  Global variable (number of sensor read outs before sending)
uint8_t packet_number_counter = 1;  //  Send packet after counter is same as PACK_NRS_BEFORE_SEND
uint8_t mpu_read_counter = 0;
uint8_t pack_nr_before_send = DEFAULT_PACK_NRS_BEFORE_SEND;
uint16_t packet_send_number = 0;
uint8_t on_counter = 0;
uint8_t off_counter = 0;

uint32_t sync_time = 0;

uint8_t data [8* MAX_BUFFER_SIZE + 7];  //  Data buffer
uint8_t rsv_buffer [50];

// ================================================================
// ===                  STATIC VOID FUNCTIONS                   ===
// ================================================================

static void LED_blink(uint8_t num, uint16_t wait_time);
static void LED_blink(uint8_t num, uint16_t on_time, uint16_t off_time);
static void LED_blink_Running();
static void IND_LED_On();
static void IND_LED_Off();
static void processIMUData(void);
static void shutdown_periferals();
static void start_periferals();

static void MPU_powerdown(void);
static void MPU_powerup(void);
static void MPU_calibrate(void);

static void Error_Handler(void);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

  //  Interrupt MPU6050
void dmpData(void){
  interruptIMU = true;
}

  //  Wake up ISR
void wakeUpISR(void){
}

  //  Sleep ISR
void sleepISR(){
  if(state != SLEEP) state = SLEEP;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

      //  Configure IN- and OUTPUTS
    pinMode(MPU_INT, INPUT);
    pinMode(IND_LED, OUTPUT);
    pinMode(MPU_ON, OUTPUT);
    
      //  Default pin states
    digitalWrite(MPU_ON, HIGH);
    digitalWrite(IND_LED, LOW);

      //  Initialise
    bms.init();
    bt.init();

      //  Set analog reference to 1.1V
    analogReference(INTERNAL);

      // Reset BT module
    bt.reset();
    delay(100);

    bt.changeScanTiming();
    bt.changeScanFactor(); 

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

uint16_t num = 0;

void loop(){

    //  Catch the incoming data from the DCU and BLE module
  if(status_periferals){
    if(state != RUNNING && state != SYNC && state != IDLE){
      if(Serial.available()){
        btcom.communication_management();
      }
    }
    else{
      if(Serial.available() > 0){
        uint8_t byte = Serial.read();
        if(byte == CMD_DATA_IND){
          uint8_t command;
          for(int i = 0; i < 10; i++){
            while(!(Serial.available() > 0));
            command = Serial.read();
          }
          btcom.rsv_msg_handler(&command);
        }
        if(byte == CMD_BEACON_IND){
          uint8_t command;
          for(int i = 0; i < 10; i++){
            while(!(Serial.available() > 0));
            command = Serial.read();
          }
          btcom.rsv_msg_handler(&command);            
        }
      }
    }
  }


  switch(state){
    case SLEEP:{
      detachInterrupt(0);
      if(status_periferals) shutdown_periferals();

      attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, FALLING);
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      detachInterrupt(0);

      if(!digitalRead(BUTTON_INT)){
        if(battery_low_state)   { state = SLEEP;  LED_blink(3, 100);  }
        else                    { state = STARTUP;                       }
      }

    }
    break;

    case STARTUP:{
      status_periferals = 1;
      wdt_enable(WDTO_8S);
      LED_blink(3, 100);
      delay(10);
      MPU_powerup();
      delay(10);
      Serial.flush();
      bt.wakeup();
      delay(10);
      attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, FALLING);
      attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpData, FALLING);
      state = IDLE;
    }
    break;
    
    case IDLE:{
      if(bt.isConnected())  {   
        delay(10);
        LED_blink(1, 50, 100);
      }
      else{
        LED_blink(1, 200);
      }
    }
    break;

    case CALIBRATION:{
      if(calibration){
        //  Already callibrated
        state = IDLE;
      }
      else {
        //  Start callibration
        MPU_calibrate();
        calibration = 1;
        bt.transmitFrameMsg(IMU_SENSOR_MODULE_IND_CALIBRATION_DONE);
      }
    }
    break;

    case SYNC:{ 
      if(sync_now){
        sync_now = 0;
        bt.disconnect();
        delay(10);
        bt.startScanning();
      }

    }
    break;

    case RUNNING:{
      if(bt.isConnected()){
        if(interruptIMU){
          interruptIMU = false;
          processIMUData();
          LED_blink_Running();
        }
      }
      else state = SLEEP;
    }
    break;

    case CHARGING:{
      IND_LED_On();
      if(status_periferals) shutdown_periferals();
      
      if(!bms.batCharging()){
        state = SLEEP;
        IND_LED_Off();
      }
    }
    break;

    case BATTERY_LOW:{
      LED_blink(6, 100);
      if(bt.isConnected()){
          //  Send battery low message
        bt.transmitFrameMsg(IMU_SENSOR_MODULE_IND_BATTERY_LOW_ERROR);
      }
      state = SLEEP;
    }
    break;

    default:
      state = IDLE;
  }

    // Reset the WDT
  wdt_reset();

    //  Check the battery voltage
  if(state != SLEEP && !bms.batStatus())    { state = BATTERY_LOW;    battery_low_state = 1;  }
    //  Check if the battery is charging
  if(bms.batCharging())   { state = CHARGING;       battery_low_state = 0;  }
}


// ================================================================
// ===                  STATIC VOID FUNCTIONS                   ===
// ================================================================

void LED_blink(uint8_t num, uint16_t wait_time){
  for(int i = 0; i < num; i++){
    digitalWrite(IND_LED, HIGH);
    delay(wait_time);
    digitalWrite(IND_LED, LOW);
    delay(wait_time);
  }
}


void LED_blink(uint8_t num, uint16_t on_time, uint16_t off_time){
  if(!digitalRead(IND_LED)){
    off_counter++;
    if(off_counter == off_time/10){
      digitalWrite(IND_LED, HIGH);
      off_counter = 0;
    }
  }
  else{
    on_counter++;
    if(on_counter == on_time/10){
      digitalWrite(IND_LED, LOW);
      on_counter = 0;
    }
  }
}

void LED_blink_Running(){
  mpu_read_counter++;
  if(mpu_read_counter%50 == 0){
    digitalWrite(IND_LED, !digitalRead(IND_LED));
    mpu_read_counter = 0;
  }
}

void IND_LED_On(void){
  digitalWrite(IND_LED, HIGH);
}

void IND_LED_Off(void){
  digitalWrite(IND_LED, LOW);
}

void shutdown_periferals(void){
  calibration = 0;
  sync_executed = 0;
  status_periferals = 0;
  mpu_read_counter = 0;
  buffer_counter = 0;
  packet_send_number = 0;
  bt.stopScanning();      // If there goes something wrong with the synchronisation or a reset of the bt module could also help
  MPU_powerdown();
  if(bt.isConnected()){
    bt.disconnect();
  }
  delay(100);
  bt.sleep_mode();
  delay(10);
  for(uint8_t i = 0; i < 5; i++)  btcom.communication_management();
  delay(10);
  Serial.flush();
  LED_blink(2, 200);
  wdt_disable();
}

// ================================================================
// ===                        MPU6050                           ===
// ================================================================


void MPU_calibrate(void){

  mpu.initialize();

  uint8_t devStatus;                  //  Return status after each device operation (0 = success, !0 = error)
  devStatus = mpu.dmpInitialize();

  //  Supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if(devStatus == 0) {
      // Calibration Time: generate offsets and calibrate the MPU6050
      LED_blink(1, 100);
      mpu.CalibrateAccel(6);
      LED_blink(1, 100);
      mpu.CalibrateGyro(6);
      LED_blink(1, 100);
  }
}

void MPU_powerdown(void){
  digitalWrite(MPU_ON, LOW);
}

void MPU_powerup(void){
  digitalWrite(MPU_ON, HIGH);
  #ifdef MPU_EXTERNAL_CLOCK
    delay(10);
    mpu.setClockSource(MPU6050_CLOCK_PLL_EXT19M);
  #endif
}

void processIMUData(void){
    //  Reset the FIFO Buffer when saving the data is not desired
    //  This happens with sampling frequencies below 100HZ
  if(packet_number_counter < pack_nr_before_send){
    mpu.resetFIFO();
    packet_number_counter++;
    return;
  }

  uint16_t fifoCount;                 //  Number of bytes currently in the FIFO buffer
  uint8_t fifoBuffer[64];             //  FIFO storage buffer
  
  fifoCount = mpu.getFIFOCount();
  uint8_t mpuIntStatus = mpu.getIntStatus();
	
    //  Check the number of bytes in the FIFO buffer 
  if (fifoCount > PACKETSIZE) Error_Handler();
  
    //  Check for a MPU6050 FIFO overflow
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();  //  Reset FIFO buffer
  }


  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)){
      //  Read a packet from the MPU6050 FIFO buffer
    while(fifoCount >= PACKETSIZE){ 
      mpu.getFIFOBytes(fifoBuffer, PACKETSIZE);
      fifoCount -= PACKETSIZE;
    }

    if(buffer_counter >= MAX_BUFFER_SIZE){
      buffer_counter = 0;
      uint32_t time_at_the_moment = millis();
      data [0] = IMU_SENSOR_MODULE_REQ_SEND_DATA;
      data [1] = (uint8_t)packet_send_number;
      data [2] = (uint8_t)(packet_send_number >> 8);
      data [3] = (uint8_t)time_at_the_moment;
      data [4] = (uint8_t)(time_at_the_moment >> 8);
      data [5] = (uint8_t)(time_at_the_moment >> 16);
      data [6] = (uint8_t)(time_at_the_moment >> 24);
      bt.transmitData((8 * MAX_BUFFER_SIZE + 1), data);
      packet_send_number++;
    }
    
    data [7 + 8*buffer_counter] = fifoBuffer[0];  
    data [8 + 8*buffer_counter] = fifoBuffer[1];
    data [9 + 8*buffer_counter] = fifoBuffer[4];
    data [10 + 8*buffer_counter] = fifoBuffer[5];
    data [11 + 8*buffer_counter] = fifoBuffer[8];
    data [12 + 8*buffer_counter] = fifoBuffer[9];
    data [13 + 8*buffer_counter] = fifoBuffer[12];
    data [14 + 8*buffer_counter] = fifoBuffer[13];
    
    packet_number_counter = 1;
    buffer_counter++;
  }
}


// ================================================================
// ===                      ERROR HANDLER                       ===
// ================================================================

void Error_Handler(void){
  state = SLEEP;
}