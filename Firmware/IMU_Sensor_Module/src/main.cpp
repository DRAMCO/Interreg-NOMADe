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


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "LowPower.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

// ================================================================
// ===                    Define constants                      ===
// ================================================================

#define PACKETSIZE              42
#define max_buffer_size         10      //  Determine number of sensor read outs before sending
#define SAMPLING_FREQ           25
#define DEFAULT_SAMPLING_FREQ   200
#define PACK_NRS_BEFORE_SEND    DEFAULT_SAMPLING_FREQ/SAMPLING_FREQ

// ================================================================
// ===                    Define MCU pins                       ===
// ================================================================

#define MPU_INT             2
#define BUTTON_INT          3
#define CHECK_BAT           4
#define BT_STATUS           5     
#define BT_CTS              6
#define BT_RTS              7
#define IND_LED             13
//#define WAKE_UP_PIN


// ================================================================
// ===              Global buffers and variables                ===
// ================================================================

//volatile bool mpuInterrupt = false; //  Indicates whether MPU interrupt pin has gone high
bool dmpReady = false;              //  Set true if DMP init was successful
bool blinkState = false;
uint8_t mpuIntStatus;               //  Holds actual interrupt status byte from MPU
uint8_t devStatus;                  //  Return status after each device operation (0 = success, !0 = error)
uint16_t fifoCount;                 //  Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];             //  FIFO storage buffer

uint8_t buffer_counter = 0;         //  Global variable (number of sensor read outs before sending)
uint8_t packet_number_counter = 0;  //  Send packet after counter is same as PACK_NRS_BEFORE_SEND
uint8_t data [8* max_buffer_size];  //  Data buffer


// ================================================================
// ===                  STATIC VOID FUNCTIONS                   ===
// ================================================================

static void BT_transmitData(uint8_t len, uint8_t *data);
static void BT_transmitFrame(uint8_t cmd, uint16_t len, uint8_t * data);
static void EUSART1_Write_Block(uint8_t * data, uint8_t len);
static uint8_t calculateCS(uint8_t * data, uint8_t len);
static void BT_sleep_mode(void);
static void BT_wakeup(void);
static void processIMUData(void);
static void Error_Handler(void);


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

void dmpDataReady(void){
    //mpuInterrupt = true;
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

    // initialize device
    #ifdef DEBUG
      DEBUG Serial.println(F("Initializing I2C devices...")); 
    #endif

    mpu.initialize();
    pinMode(MPU_INT, INPUT);

    //  Verify connection
    #ifdef DEBUG 
      Serial.println(F("Testing device connections..."));
      Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

      // wait for ready
      Serial.println(F("\nSend any character to begin DMP programming and demo: "));
      while (Serial.available() && Serial.read()); // empty buffer
      while (!Serial.available());                 // wait for data
      while (Serial.available() && Serial.read()); // empty buffer again
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

      //  Configure IN- and OUTPUTS
    pinMode(IND_LED, OUTPUT);
    pinMode(BT_CTS, OUTPUT);
    pinMode(BT_RTS, OUTPUT);
    pinMode(CHECK_BAT, OUTPUT);
    pinMode(BT_STATUS, INPUT);
    //pinMode(WAKE_UP_PIN, OUTPUT);

    digitalWrite(IND_LED, LOW);
    //digitalWrite(WAKE_UP_PIN, HIGH);

      // if programming failed, don't try to do anything
    if (!dmpReady) Error_Handler();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // Allow wake up pin to trigger interrupt on low.
    //attachInterrupt(0, wakeUp, LOW);
  attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
  Serial.flush();
  
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
  
    // Disable external pin interrupt on wake up pin.
  detachInterrupt(0); 

    //  Check Battery

    //  Check BT connection

  processIMUData();
}



void processIMUData(void){

    //  Reset the FIFO Buffer if we don't want to save this data
  if(packet_number_counter < PACK_NRS_BEFORE_SEND){
    mpu.resetFIFO();
    packet_number_counter++;
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

    if(buffer_counter >= max_buffer_size){
      buffer_counter = 0;
      BT_transmitData(8*max_buffer_size, data);
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


void BT_transmitData(uint8_t len, uint8_t *data){
    BT_transmitFrame(0x04, len, data);
}

void BT_transmitFrame(uint8_t cmd, uint16_t len, uint8_t * data){
    uint8_t tx_buffer [len + 4];
    tx_buffer [0] = 0x02;
    tx_buffer [1] = cmd;
    tx_buffer [2] = (uint8_t)len;
    tx_buffer [3] = (uint8_t)(len >> 8);
    for(uint8_t i = 0; i < len; i++){
        tx_buffer [i + 4] = *(data + i);
    }
    
    EUSART1_Write_Block(tx_buffer, (4 + len));
}

void EUSART1_Write_Block(uint8_t * data, uint8_t len){
    //BT_check_receive_buffer();
    uint8_t cs = calculateCS(data, len);
    /*
    for(uint8_t i = 0; i < len; i++){
        Serial.write(*(data + i));
    }*/
    Serial.write(data, len);
    Serial.write(cs);
}

uint8_t calculateCS(uint8_t * data, uint8_t len){
    uint8_t checksum = *(data);
    for(uint8_t i = 1; i < len; i++){
        checksum = checksum ^ *(data + i);
    }
    return checksum;
}


void BT_sleep_mode(void){
  BT_transmitFrame(0x02, NULL, NULL);
}

void BT_wakeup(void){
  //digitalWrite(WAKE_UP_PIN, LOW);
  //delay(5);
  //digitalWrite(WAKE_UP_PIN, HIGH);
}


void Error_Handler(void){
  while(1){
    digitalWrite(IND_LED, !digitalRead(IND_LED));
    delay(100);
  }
}