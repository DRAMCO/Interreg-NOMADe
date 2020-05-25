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
 *      Version: V1.0
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
 * ----- Check received message from ble module when go low power (voor de zekerheid)
 * OK -- When sending data check first if there is a connection
 * ----- Send Battery voltage
 * ...
 */

// ================================================================
// ===                      Create objects                      ===
// ================================================================

MPU6050 mpu;
BMS bms;
BLUETOOTH bt;

// ================================================================
// ===                      State Machine                       ===
// ================================================================


enum Sensor_Reader_State{SLEEP, CALIBRATION, IDLE, RUNNING, CHARGING};
uint8_t state = SLEEP;

bool calibartion = 0;
bool status_periferals = 0;


// ================================================================
// ===              Global buffers and variables                ===
// ================================================================

bool dmpReady = false;              //  Set true if DMP init was successful
bool blinkState = false;
bool running = true;
bool interruptIMU = false;
uint8_t mpuIntStatus;               //  Holds actual interrupt status byte from MPU
uint8_t devStatus;                  //  Return status after each device operation (0 = success, !0 = error)
uint16_t fifoCount;                 //  Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];             //  FIFO storage buffer

uint8_t buffer_counter = 0;         //  Global variable (number of sensor read outs before sending)
uint8_t packet_number_counter = 0;  //  Send packet after counter is same as PACK_NRS_BEFORE_SEND
uint8_t data [8* MAX_BUFFER_SIZE];  //  Data buffer


// ================================================================
// ===                  STATIC VOID FUNCTIONS                   ===
// ================================================================

static void LED_blink(uint8_t num, uint8_t wait_time);
static void IND_LED_On();
static void IND_LED_Off();
static void processIMUData(void);
static void runcode(void);
static void shutdown_periferals();
static void start_periferals();

static void MPU_powerdown();
static void MPU_powerup();
static void MPU_calibrate();

static void Error_Handler(void);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void dmpData(void){
  interruptIMU = true;
}

void wakeUpISR(void){
}

void sleepISR()
{
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
    bt.reset();

      //  Set analog reference to 1.1V
    analogReference(INTERNAL);

    //MPU_calibrate();

    bt.reset();

    //start_periferals();

    //attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpData, RISING);
    //attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, RISING);
    //attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpData, FALLING);
    //attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, FALLING);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
/*
void loop() 
{
    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, RISING);
    
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    
    // Disable external pin interrupt on wake up pin.
    detachInterrupt(0); 
    

    LED_blink(2, 200);
    // Do something here
    // Example: Read sensor, data logging, data transmission.
}
*/



void loop(){
  
  // Eerst kijken of de batterij spanning voldoende is 
  if(!bms.batStatus())    state = SLEEP;
  // Vervolgens kijken of de batterij wordt geladen
  if(bms.batCharging())   state = CHARGING;
  
  switch(state){
    case SLEEP:
      detachInterrupt(0);
      if(status_periferals) shutdown_periferals();

      attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, FALLING);
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
      detachInterrupt(0);

      // Eerst nog snel batterij controleren?

      if(!digitalRead(BUTTON_INT)) state = IDLE;

      break;
    
    case CALIBRATION:
      if(calibartion)
        state = RUNNING;
      else {
        MPU_calibrate();
        calibartion = 1;
      }
      break;
    
    case IDLE:
      // wachten op een start commando of een BT connectie
      if(!status_periferals)  start_periferals();
      if(bt.isConnected())    state = CALIBRATION;
      else                    LED_blink(2, 200);
      break;
    
    case RUNNING:
      if(bt.isConnected()){
        if(interruptIMU){
          interruptIMU = false;
          processIMUData();
        }
      }
      else state = SLEEP;
      break;

    case CHARGING: 
      IND_LED_On();
      if(status_periferals) shutdown_periferals();
      
      if(!bms.batCharging()){
        state = SLEEP;
        IND_LED_Off();
      }

      break;

    default:
      state = IDLE;
  }
}




// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
/*
void loop() {
  if(bms.batCharging())   running = 0;  //{ running = 0;  Serial.println("Battery charging");   }
  if(!bms.batStatus())    running = 0;  //{ running = 0;  Serial.println("Low battery");        }

  if(running){
    runcode();
  }
  else{
      //  Not running so go low power and check bat charging
    detachInterrupt(0);
    LED_blink(2, 200);
    MPU_powerdown();
    bt.sleep_mode();
    delay(100);
    //LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF); 
    attachInterrupt(0, wakeUpISR, LOW);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    detachInterrupt(0);
    running = true;
    LED_blink(3, 100);
    delay(10);
    bms.batCharging();
    MPU_powerup();
    delay(10);
    //mpu.resetFIFO();
    bt.wakeup();
    delay(10);
    bt.reset();
    //MPU kalibration
    MPU_calibrate();
    Serial.flush();
    attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpData, RISING);
  }
}
*/


// ================================================================
// ===                  STATIC VOID FUNCTIONS                   ===
// ================================================================

void runcode(void){
  #ifdef IMU
    if(interruptIMU){
      interruptIMU = false;
      processIMUData();
    }
    /*else{
      Serial.flush();
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    }*/
  #endif
}


void processIMUData(void){

    //  Reset the FIFO Buffer if we don't want to save this data
  if(packet_number_counter < PACK_NRS_BEFORE_SEND){
    mpu.resetFIFO();
    packet_number_counter++;
    digitalWrite(IND_LED, LOW);
    return;
  }

    //  Reset interrupt flag and get INT_STATUS byte
  //mpuInterrupt = false;
  
  fifoCount = mpu.getFIFOCount();
  mpuIntStatus = mpu.getIntStatus();
	
    //  Check the number of bytes in the FIFO buffer 
  if (fifoCount > PACKETSIZE) Error_Handler();
  
    //  Check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
    #ifdef DEBUG
      Serial.println(F("FIFO overflow!"));
    #endif
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }


  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

      //  Read a packet from FIFO
    while(fifoCount >= PACKETSIZE){ // Lets catch up to NOW, someone is using the dreaded delay()!
      mpu.getFIFOBytes(fifoBuffer, PACKETSIZE);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= PACKETSIZE;
    }

    //digitalWrite(IND_LED, LOW);
    if(buffer_counter >= MAX_BUFFER_SIZE){
      buffer_counter = 0;
      bt.transmitData(8*MAX_BUFFER_SIZE, data);
      digitalWrite(IND_LED, HIGH);
    }

    
    data [0 + 8*buffer_counter] = fifoBuffer[0];
    data [1 + 8*buffer_counter] = fifoBuffer[1];
    data [2 + 8*buffer_counter] = fifoBuffer[4];
    data [3 + 8*buffer_counter] = fifoBuffer[5];
    data [4 + 8*buffer_counter] = fifoBuffer[8];
    data [5 + 8*buffer_counter] = fifoBuffer[9];
    data [6 + 8*buffer_counter] = fifoBuffer[12];
    data [7 + 8*buffer_counter] = fifoBuffer[13];
    
    packet_number_counter = 0;
    buffer_counter++;
  }
}

void LED_blink(uint8_t num, uint8_t wait_time){
  for(int i = 0; i < num; i++){
    digitalWrite(IND_LED, HIGH);
    delay(wait_time);
    digitalWrite(IND_LED, LOW);
    delay(wait_time);
  }
}

void IND_LED_On(){
  digitalWrite(IND_LED, HIGH);
}

void IND_LED_Off(){
  digitalWrite(IND_LED, LOW);
}




void shutdown_periferals(){
  calibartion = 0;
  status_periferals = 0;
  MPU_powerdown();
  bt.disconnect();
  delay(10);
  bt.sleep_mode();
  delay(100);
  LED_blink(2, 200);
}

void start_periferals(){
  status_periferals = 1;
  LED_blink(3, 100);
  delay(10);
  MPU_powerup();
  delay(10);
  bt.wakeup();
  delay(10);
  bt.reset();
  delay(100);
  Serial.flush();
  attachInterrupt(digitalPinToInterrupt(BUTTON_INT), sleepISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpData, FALLING);
}


// ================================================================
// ===                        MPU6050                           ===
// ================================================================


void MPU_calibrate(){
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

      mpu.setDMPEnabled(true);

      #ifdef DEBUG
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(MPU_INT));
        Serial.println(F(")..."));
      #endif

      //attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      #ifdef DEBUG
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
      #endif

      dmpReady = true;

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

void MPU_powerdown(){
  digitalWrite(MPU_ON, LOW);
}

void MPU_powerup(){
  digitalWrite(MPU_ON, HIGH);
}



// ================================================================
// ===                      ERROR HANDLER                       ===
// ================================================================

void Error_Handler(void){
  while(1){
    digitalWrite(IND_LED, !digitalRead(IND_LED));
    delay(100);
  }
}