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
 *      Created: 2020-02-27
 *       Author: Jarne Van Mulders
 *      Version: V3.1
 *
 *  Description: Firmware IMU sensor module for the NOMADe project
 *
 *  Interreg France-Wallonie-Vlaanderen NOMADe
 *
 */

#include "Arduino.h"
#include "LowPower.h"
#include "BTCom.h"
#include <avr/wdt.h>
#include "IMU.h"

// ================================================================
// ===                          TO DO                           ===
// ================================================================
/*        Date: 07/04/2020      */
/*
 * OK -- Led currently off when charging
 * OK--- Check received message from ble module when go low power (voor de zekerheid)
 * OK -- When sending data check first if there is a connection
 * OK--- Send Battery voltage
 * ----- Writing Module Test software (for new modules)
 * OK--- Add WatchDog Timer
 * ----- Do synchronisation based on beacons and advertisements
 * ----- Check CMD_DISCONNECT_CNF and only if operation failed, redo a disconnection
 */


// ================================================================
// ===                      Create objects                      ===
// ================================================================

Variables counters;
LED led(&counters);
MPU6050 mpu;
DMP dmp(&mpu);
BMS bms;
BLUETOOTH bt;
IMU imu(&dmp, &mpu, &counters, &led);
BTCOM btcom(&bt, &bms, &mpu, &counters);

// ================================================================
// ===                      State Machine                       ===
// ================================================================

uint8_t state = SLEEP;
uint8_t dataformat = QUAT;

// ================================================================
// ===              Global buffers and variables                ===
// ================================================================

uint32_t startup_time = 0;
uint32_t sync_time = 0;
uint32_t baudrate_array [] = {9600, 19200, 38400, 115200, 230400, 460800, 921600};
uint32_t baud_pointer = 3;

// ================================================================
// ===                    Global booleans                       ===
// ================================================================

bool calibration = 0;
bool status_periferals = 1;
bool sync_now = 0;
bool sync_executed = 0;
bool synchronisation = 0;
bool battery_low_state = 0;
bool interruptIMU = false;
bool baud_correct = false;

// ================================================================
// ===                  STATIC VOID FUNCTIONS                   ===
// ================================================================

static void state_machine();
static void processIMUData(void);
static void shutdown_periferals();

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
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

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
    
      //  Set analog reference to external voltage AREF pin
    analogReference(EXTERNAL);

    Serial.flush();

      // Reset BT module
    bt.reset();
    delay(100);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop(){

    //  Check the incoming BLE data
  btcom.incoming_data_handler();

    //  Check baudrate is matched with the Proteus II modem
  btcom.controle_check_baudrate();

    //  Check state machine
  if(btcom.baudrate_ok()) state_machine();

    //  Reset the WDT
  wdt_reset();

    //  Check battery voltage
  if(state != SLEEP && !bms.batStatus())    { state = BATTERY_LOW;    battery_low_state = 1;  }

    //  Check if battery is charging
  if(bms.batCharging())                     { state = CHARGING;       battery_low_state = 0;  }
}


// ================================================================
// ===                  STATIC VOID FUNCTIONS                   ===
// ================================================================

void state_machine(){
  switch(state){
    
    /***    Sleep State                                         ****
     * Shutdown everthing: IMU, BLE modules
     * Go to sleep
     * Wake up after 8s and check if the battery is charging
    ****                                                        ***/
    case SLEEP:{
      detachInterrupt(0);
      if(status_periferals) shutdown_periferals();

      attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, FALLING);
      LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
      detachInterrupt(0);

      if(!digitalRead(BUTTON_INT)){
        if(battery_low_state){ 
          state = SLEEP;  
          led.blink(3, 100);  
        }
        else{
          startup_time = millis();
          state = STARTUP;
        }
      }
    }
    break;

    /***    Startup State   ****
     * Start up all periferals
    ****                        ***/
    case STARTUP:{
      status_periferals = 1;
      wdt_enable(WDTO_8S);
      led.blink(3, 100);
      delay(10);
      imu.powerup();
      delay(10);
      Serial.flush();
      bt.wakeup();
      delay(10);
      attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, FALLING);
      attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpData, FALLING);
      state = WAIT_FOR_CONNECTION;
    } break;


    /***    Wait_for_connection State   ****
     * Wait for BLE connection
     * Blink the indication LED
    ****                        ***/
    case WAIT_FOR_CONNECTION:{
      if(bt.isConnected()){
        state = IDLE;
      }
      else{
        led.blink(1, 200);
        if(millis() > startup_time + STARTUP_TIMEOUT) state = SLEEP;
      }
    } break;
    

    /***    Idle State   ****
     * Do nothing
    ****                        ***/
    case IDLE:{
      if(bt.isConnected()){   
        if(sync_executed){
          //uint8_t value = (millis() - sync_time + 1000)/1000;
          //uint32_t starttime = sync_time + (1000 * value);
          //while(millis() < starttime);

          bt.transmitFrameMsg(IMU_SENSOR_MODULE_IND_SYNC_DONE);
          sync_executed = 0;
          synchronisation = 1;
        }

        delay(1);
        led.blink(1, 50, 100);
      }
      else{
        led.blink(1, 200);
        state = WAIT_FOR_CONNECTION;
      }
    }
    break;


    /***    Calibration State   ****
     * Calibrate the IMU
     * Send calibration done MSG
    ****                        ***/
    case CALIBRATION:{
      imu.calibrate();
      calibration = 1;
      bt.transmitFrameMsg(IMU_SENSOR_MODULE_IND_CALIBRATION_DONE);
      state = IDLE;
    }
    break;


    /***    Sync State   ****
     * Set BLE module in scanning mode
     * Wait for beacon MSG
     * Update synctime
     * Wait for reconnection
     * Send synchronisation done MSG
    ****                        ***/
    case SYNC:{
      if(sync_now){
        sync_now = 0;

        bt.disconnect();
        delay(10);
      
        bt.startScanning();
      }

      if(sync_executed && bt.isConnected()){
          //uint8_t value = (millis() - sync_time + 1000)/1000;
          //uint32_t starttime = sync_time + (1000 * value);
          //while(millis() < starttime);

          bt.transmitFrameMsg(IMU_SENSOR_MODULE_IND_SYNC_DONE);
          sync_executed = 0;
          synchronisation = 1;
          state = IDLE; 
      }
    }
    break;


    /***    Init Measurement State    ****
     * Initialize buffers
    ****                              ***/
    case INIT_MES:{
      imu.initMeasurement();
      state = RUNNING;
    }

    /***    Running State                 ****
     * Wait for MPU6050 Intterupts
     * Process the data
     * Send the data through BLE
     * Blink the LED during measurements
    ****                                  ***/
    case RUNNING:{
      if(bt.isConnected()){
        if(interruptIMU){
          interruptIMU = false;
          processIMUData();
          led.blink_Running();
        }
      }
      else state = SLEEP;
    }
    break;


    /***    Charging State                  ****
     * Shutdown everthing: IMU, BT module
     * Go to SLEEP and charge the battery
    ****                                    ***/
    case CHARGING:{
      led.on();
      if(status_periferals) shutdown_periferals();
      if(!bms.batCharging()){
        state = SLEEP;
        led.off();
      }
    }
    break;


    /***    Battery Low State   ****
     * Send battery low msg
     * Go to SLEEP
    ****                        ***/
    case BATTERY_LOW:{
      if(bt.isConnected()){
        bt.transmitFrameMsg(IMU_SENSOR_MODULE_IND_BATTERY_LOW_ERROR);
      }
      led.blink(6, 100);
      state = SLEEP;
    }
    break;


    /***    default State        ****
     * Go to IDLE mode
    ****                         ***/
    default:
      state = IDLE;
  }
}


void processIMUData(){
  uint8_t len = 0;
  //uint8_t* data_send_buffer = imu.getDataBuffer();
  uint8_t data_send_buffer [255];
  if(imu.getIMUData(&len, data_send_buffer)){
    switch(dataformat){
      case QUAT:{               *(data_send_buffer + 0) = IMU_SENSOR_MODULE_RSP_SEND_DATA_F1;    } break;
      case GYRO:{               *(data_send_buffer + 0) = IMU_SENSOR_MODULE_RSP_SEND_DATA_F2;    } break;
      case ACC:{                *(data_send_buffer + 0) = IMU_SENSOR_MODULE_RSP_SEND_DATA_F3;    } break;
      case GYRO_ACC:{           *(data_send_buffer + 0) = IMU_SENSOR_MODULE_RSP_SEND_DATA_F4;    } break;
      case QUART_GYRO_ACC:{     *(data_send_buffer + 0) = IMU_SENSOR_MODULE_RSP_SEND_DATA_F5;    } break;
    }
    bt.transmitData(len, data_send_buffer);
  }
}


void shutdown_periferals(void){
  calibration = 0;
  sync_executed = 0;
  synchronisation = 0;
  status_periferals = 0;

  dataformat = QUAT;

  imu.reset_counters();
  bt.stopScanning(); // If there goes something wrong with the synchronisation or a reset of the bt module could also help
  imu.powerdown();
  if(bt.isConnected()){
    bt.disconnect();
  }
  delay(100);
  bt.sleep_mode();
  delay(10);
  Serial.flush();
  led.blink(2, 200);
  wdt_disable();
}

