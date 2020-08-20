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


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define IMU


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
 * ----- Add WatchDog Timer
 * ----- Do synchronisation based on beacons and advertisements
 */

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

//bool dmpReady = false;            //  Set true if DMP init was successful
//uint8_t mpuIntStatus;               //  Holds actual interrupt status byte from MPU

uint8_t buffer_counter = 0;         //  Global variable (number of sensor read outs before sending)
uint8_t packet_number_counter = 1;  //  Send packet after counter is same as PACK_NRS_BEFORE_SEND
uint8_t mpu_read_counter = 0;
uint8_t pack_nr_before_send = DEFAULT_PACK_NRS_BEFORE_SEND;

uint32_t sync_time = 0;

uint8_t data [8* MAX_BUFFER_SIZE + 1];  //  Data buffer
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

//static void communication_management();
//static void rsv_msg_handler(uint8_t * command_msg);

static void MPU_powerdown(void);
static void MPU_powerup(void);
static void MPU_calibrate(void);
//static void MPU_Start_DMP(void);

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
    
      //  Clear Receive Buffer
    //while(Serial.available() > 0){
    //  communication_management();
    //}

      //  Update UART Baudrate (MCU + BT module)
    //bt.updateBaudrate(BT_UART_BAUDRATE);


      /*  MAG WEG  */
    //delay(4000);
    //Serial.write(0x14);
    Serial.write(0x99);
    //delay(100);

    bt.changeScanTiming();    // Niet Low Power !!!
    bt.changeScanFactor();    // Niet Low Power !!!

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

//#define CHECK_FRAMES_CONSTANTLY                   //  Causes a program crash
#define CHECK_FRAMES_EXCEPT_DURING_RUNNING_STATE
//#define PROBEERSEL

uint16_t num = 0;

void loop(){
  
  // Eerst kijken of de batterij spanning voldoende is 
  if(!bms.batStatus())    { state = BATTERY_LOW;    battery_low_state = 1;  }
  // Vervolgens kijken of de batterij wordt geladen
  if(bms.batCharging())   { state = CHARGING;       battery_low_state = 0;  }


  #ifdef PROBEERSEL
    if(status_periferals && state != RUNNING){
      if(Serial.available()){
        communication_management();
      }
    }
    if(state == RUNNING){
      num++;
      if(num == 500){
        num = 0;
        if(Serial.available()){
          communication_management();
        }
        Serial.write(0x71);
      }
    }
  #endif


  #ifdef CHECK_FRAMES_CONSTANTLY
    if(status_periferals){
      if(Serial.available()){
        communication_management();
      }
    }
  #endif


  #ifdef CHECK_FRAMES_EXCEPT_DURING_RUNNING_STATE

    if(status_periferals){
      if(state != RUNNING && state != SYNC && state != IDLE){
        if(Serial.available()){
          btcom.communication_management();
        }
      }

      //  Nog steeds een porbleem met het uitvoeren van communication_management tijdens RUNNING
      /*
      if(state == RUNNING && (buffer_counter == 2 || buffer_counter == 6)) {
        communication_management();
      }*/

      //---------------------------------------------------//
      // Alternatieve oplossing
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
    //  BLIJKT GOED TE WERKEN, EVENTUEEL ALLES OP DEZE MANIER UITLEZEN

    //---------------------------------------------------//
  #endif
/*
  if(status_periferals){
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
*/

  switch(state){
    case SLEEP:{
      detachInterrupt(0);
      if(status_periferals) shutdown_periferals();

      attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, FALLING);
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      detachInterrupt(0);

      // Eerst nog snel batterij controleren?

      if(!digitalRead(BUTTON_INT)){
        if(battery_low_state)   { state = SLEEP;  LED_blink(3, 100);  }
        else                    { state = STARTUP;                       }
      }

    }
    break;

    case STARTUP:{
      status_periferals = 1;
      LED_blink(3, 100);
      delay(10);
      MPU_powerup();
      delay(10);
      Serial.flush();
      bt.wakeup();
      delay(10);
      //catch_receive_msg();//7, 500);
      //delay(10);
      //bt.reset();
      //Serial.flush();
      //delay(100);
      //Serial.flush();
      attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, FALLING);
      attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpData, FALLING);
      state = IDLE;
    }
    break;
    
    case IDLE:{
      // wachten op een start commando of een BT connectie
      //if(!status_periferals)  start_periferals();
      if(bt.isConnected())  {   
        //if(!calibration){
        //LED_blink(1, 50, 100); 
        delay(10);
        LED_blink(1, 50, 100);
        //}
        //else delay(5); 
        //else 
        //state = SYNC;
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

    /*
    case SYNC:{
      delay(5);
      if(sync_now){
        sync_now = 0;
        MPU_Start_DMP();      //    Start IMU DMP
        state = RUNNING;
      }
    }
    break;
    */

    case SYNC:{
      
      //delay(5);   //  Is this delay NECESSARY???????
      
      if(sync_now){
        sync_now = 0;

        bt.disconnect();
        delay(10);
      
        bt.startScanning();
      }

      //if(sync_executed && bt.isConnected()){
      //    MPU_Start_DMP();      //    Start IMU DMP
      //    state = RUNNING; 
      //}
    }
    break;

      //    Running zal moeten worden aangepast en wachten op een start measurements commando
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
}


// ================================================================
// ===                  STATIC VOID FUNCTIONS                   ===
// ================================================================

void processIMUData(void){

    //  Reset the FIFO Buffer if we don't want to save this data
  if(packet_number_counter < pack_nr_before_send){
    mpu.resetFIFO();
    packet_number_counter++;
    //digitalWrite(IND_LED, LOW);
    return;
  }

  uint16_t fifoCount;                 //  Count of all bytes currently in FIFO
  uint8_t fifoBuffer[64];             //  FIFO storage buffer
  
  fifoCount = mpu.getFIFOCount();
  uint8_t mpuIntStatus = mpu.getIntStatus();
	
    //  Check the number of bytes in the FIFO buffer 
  if (fifoCount > PACKETSIZE) Error_Handler();
  
    //  Check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    #ifdef DEBUG
      Serial.println(F("FIFO overflow!"));
    #endif
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }


  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)){
      //  Read a packet from FIFO
    while(fifoCount >= PACKETSIZE){ // Lets catch up to NOW, someone is using the dreaded delay()!
      mpu.getFIFOBytes(fifoBuffer, PACKETSIZE);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= PACKETSIZE;
    }

    if(buffer_counter >= MAX_BUFFER_SIZE){
      buffer_counter = 0;
      bt.transmitData((8 * MAX_BUFFER_SIZE + 1), data);
      //digitalWrite(IND_LED, HIGH);
    }
    
    data [0] = IMU_SENSOR_MODULE_REQ_SEND_DATA;
    data [1 + 8*buffer_counter] = fifoBuffer[0];
    data [2 + 8*buffer_counter] = fifoBuffer[1];
    data [3 + 8*buffer_counter] = fifoBuffer[4];
    data [4 + 8*buffer_counter] = fifoBuffer[5];
    data [5 + 8*buffer_counter] = fifoBuffer[8];
    data [6 + 8*buffer_counter] = fifoBuffer[9];
    data [7 + 8*buffer_counter] = fifoBuffer[12];
    data [8 + 8*buffer_counter] = fifoBuffer[13];
    
    packet_number_counter = 1;
    buffer_counter++;
  }
}

void LED_blink(uint8_t num, uint16_t wait_time){
  for(int i = 0; i < num; i++){
    digitalWrite(IND_LED, HIGH);
    delay(wait_time);
    digitalWrite(IND_LED, LOW);
    delay(wait_time);
  }
}

/*
void LED_blink(uint8_t num, uint16_t on_time, uint16_t off_time){
  for(int i = 0; i < num; i++){
    digitalWrite(IND_LED, HIGH);
    delay(on_time);
    digitalWrite(IND_LED, LOW);
    delay(off_time);
  }
}
*/

uint8_t on_counter = 0;
uint8_t off_counter = 0;

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
  bt.stopScanning(); // If there goes something wrong with the synchronisation or a reset of the bt module could also help
  MPU_powerdown();
  if(bt.isConnected()){
    bt.disconnect();
    //catch_receive_msg();//12, 500);
  }
  delay(100);
  bt.sleep_mode();
  //catch_receive_msg();//6, 500);

  delay(10);
  
  for(uint8_t i = 0; i < 5; i++)  btcom.communication_management();

  delay(10);
  Serial.flush();
  LED_blink(2, 200);
}

/*
void start_periferals(void){
  status_periferals = 1;
  LED_blink(3, 100);
  delay(10);
  MPU_powerup();
  delay(10);
  Serial.flush();
  bt.wakeup();
  delay(10);
  //catch_receive_msg();//7, 500);
  //delay(10);
  //bt.reset();
  //Serial.flush();
  //delay(100);
  //Serial.flush();
  attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpData, FALLING);
}
*/

// ================================================================
// ===                        MPU6050                           ===
// ================================================================


void MPU_calibrate(void){
  #ifdef DEBUG
    DEBUG Serial.println(F("Initializing I2C devices...")); 
  #endif

  mpu.initialize();

  //  Verify connection
  #ifdef DEBUG 
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again
  #endif

  //  Load and configure the DMP
  #ifdef DEBUG
    Serial.println(F("Initializing DMP..."));
  #endif

  uint8_t devStatus;                  //  Return status after each device operation (0 = success, !0 = error)
  devStatus = mpu.dmpInitialize();

  //  Supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip


  //  Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      LED_blink(1, 100);
      mpu.CalibrateAccel(6);
      LED_blink(1, 100);
      mpu.CalibrateGyro(6);
      LED_blink(1, 100);
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


//void MPU_Start_DMP(void){
  //mpu.setDMPEnabled(true);
  //mpuIntStatus = mpu.getIntStatus();
//}

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



// ================================================================
// ===                      ERROR HANDLER                       ===
// ================================================================

void Error_Handler(void){
  state = SLEEP;
  /*
    while(1){
      digitalWrite(IND_LED, !digitalRead(IND_LED));
      delay(100);
    }
  */
}