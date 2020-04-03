// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


/*
void showDat(uint8_t len, uint8_t *buf){
  int16_t data [4];
  data[0] = ((buf[0] << 8) | buf[1]);
  data[1] = ((buf[2] << 8) | buf[3]);
  data[2] = ((buf[4] << 8) | buf[5]);
  data[3] = ((buf[6] << 8) | buf[7]);

  print(data);

  float q [4];
	q[0] = (float)data[0] / 16384.0f;   //  w
	q[1] = (float)data[1] / 16384.0f;   //  x
	q[2] = (float)data[2] / 16384.0f;   //  y
	q[3] = (float)data[3] / 16384.0f;   //  z

	// gravity
	float gravity [3];
	gravity[0] = 2 * (q[1] * q[3] - q[0] * q[2]);													//	x
	gravity[1] = 2 * (q[0] * q[1] + q[2] * q[3]);													//	y
	gravity[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];		//	z

	//	Euler
	//float euler [3];
	//euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);     // psi
	//euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]);                                            // theta
	//euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1);     // phi
	
  float newdata [3];

	// yaw: (about Z axis)
	newdata[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);
	// pitch: (nose up/down, about Y axis)
	newdata[1] = atan2(gravity[0] , sqrt(gravity[1] * gravity[1] + gravity[2] * gravity[2]));
	// roll: (tilt left/right, about X axis)
	newdata[2] = atan2(gravity[1], gravity[2]);
	
	if (gravity[2] < 0) {
		if(newdata[1] > 0)		newdata[1] = PI 	- newdata[1]; 
		else 									newdata[1] = -PI 	- newdata[1];
	}


  Serial.print("ypr\t");
  Serial.print(newdata[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(newdata[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(newdata[2] * 180/M_PI);




}

void print(uint16_t *print_buf){
  Serial.print("buf\t");
  Serial.print(print_buf[0]);
  Serial.print("\t");
  Serial.print(print_buf[1]);
  Serial.print("\t");
  Serial.print(print_buf[2]);
  Serial.print("\t");
  Serial.println(print_buf[3]);
}

*/



LOOP

/*
    // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < PACKETSIZE) {
      if (mpuInterrupt && fifoCount < PACKETSIZE) {
        // try to get out of the infinite loop 
        fifoCount = mpu.getFIFOCount();
      }  
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
  }*/



  
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
#define SAMPLING_FREQ           200
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

volatile bool mpuInterrupt = false; //  Indicates whether MPU interrupt pin has gone high
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
    mpuInterrupt = true;
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
  if (fifoCount < PACKETSIZE) fifoCount = mpu.getFIFOCount();
  else  Error_Handler();

    //  Reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

    //  Get current FIFO count
  //fifoCount = mpu.getFIFOCount();
	
  if(fifoCount < PACKETSIZE){
	  //  Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
		//  This is blocking so don't do it while (fifoCount < PACKETSIZE) fifoCount = mpu.getFIFOCount();
	}
  
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
      //BT_wakeup();
      BT_transmitData(8*max_buffer_size, data);
      //BT_sleep_mode();
      //delay(1000);
    }

    if(packet_number_counter == PACK_NRS_BEFORE_SEND){
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
    else packet_number_counter++;


      // blink LED to indicate activity
      //blinkState = !blinkState;
      //digitalWrite(IND_LED, blinkState);
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
#define SAMPLING_FREQ           200
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

volatile bool mpuInterrupt = false; //  Indicates whether MPU interrupt pin has gone high
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
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
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

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(MPU_INT));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        //packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(IND_LED, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < PACKETSIZE) {
        if (mpuInterrupt && fifoCount < PACKETSIZE) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
	if(fifoCount < PACKETSIZE){
	        //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
			// This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
	}
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
	while(fifoCount >= PACKETSIZE){ // Lets catch up to NOW, someone is using the dreaded delay()!
		mpu.getFIFOBytes(fifoBuffer, PACKETSIZE);
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= PACKETSIZE;
	}


        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(IND_LED, blinkState);
    } 
}
