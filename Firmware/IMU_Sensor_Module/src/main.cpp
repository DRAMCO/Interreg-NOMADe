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
// ===                      Create objects                      ===
// ================================================================

MPU6050 mpu;
BMS bms;
BLUETOOTH bt;


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

static void BT_transmitData(uint8_t len, uint8_t *data);
static void BT_transmitFrame(uint8_t cmd, uint16_t len, uint8_t * data);
static void BT_transmitFrame(uint8_t cmd, uint8_t data);
static void EUSART1_Write_Block(uint8_t * data, uint8_t len);
static uint8_t calculateCS(uint8_t * data, uint8_t len);
static void BT_sleep_mode(void);
static void BT_wakeup(void);
static void processIMUData(void);
static void Error_Handler(void);
static void runcode(void);
static void MPU_powerdown();
static void MPU_powerup();

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void dmpData(void){
  interruptIMU = true;
}

void buttonISR(void){
  running = !running;
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




      // Initialize device
    #ifdef DEBUG
      DEBUG Serial.println(F("Initializing I2C devices...")); 
    #endif

    #ifdef IMU
      mpu.initialize();
    #endif

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

    #ifdef IMU
      devStatus = mpu.dmpInitialize();
    #endif

    //  Supply your own gyro offsets here, scaled for min sensitivity
    #ifdef IMU
      mpu.setXGyroOffset(220);
      mpu.setYGyroOffset(76);
      mpu.setZGyroOffset(-85);
      mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    #endif

    //  Make sure it worked (returns 0 if so)
    #ifdef IMU
      if (devStatus == 0) {
          // Calibration Time: generate offsets and calibrate our MPU6050
          mpu.CalibrateAccel(6);
          mpu.CalibrateGyro(6);
          mpu.PrintActiveOffsets();

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
    #endif

    //#ifdef DEBUG 
      else {
          // ERROR!
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
      }
    //#endif



      // if programming failed, don't try to do anything
    #ifdef DEBUG 
      if (!dmpReady) Error_Handler();
    #endif


    bt.reset();

    attachInterrupt(digitalPinToInterrupt(BUTTON_INT), buttonISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpData, RISING);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  if(bms.batCharging())   running = 0;  //{ running = 0;  Serial.println("Battery charging");   }
  if(!bms.batStatus())    running = 0;  //{ running = 0;  Serial.println("Low battery");        }

  if(running){
    runcode();
  }
  else{
      //  Not running so go low power and check bat charging
    MPU_powerdown();
    bt.BT_sleep_mode();
    delay(10);
    //LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF); 
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    bms.batCharging();
    MPU_powerup();
    mpu.resetFIFO();
    //MPU kalibration
  }
}


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
      bt.BT_transmitData(8*MAX_BUFFER_SIZE, data);
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


// ================================================================
// ===                        MPU6050                           ===
// ================================================================


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