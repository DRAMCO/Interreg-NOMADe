#ifndef CONFIG_h
#define CONFIG_h

#include "Arduino.h"

// ================================================================
// ===                    Software version                      ===
// ================================================================

#define SW_VERSION  SW_V32

#define SW_V10  10
#define SW_V20  20
#define SW_V30  30
#define SW_V31  31
#define SW_V32  32

// ================================================================
// ===                    Define constants                      ===
// ================================================================

#define PACKETSIZE                      42
#define MAX_BUFFER_SIZE                 10      //  Number of samples in one BLE packet
#define DEFAULT_SAMPLING_FREQ           100
#define IMU_SAMPLING_FREQ               100
#define DEFAULT_PACK_NRS_BEFORE_SEND    IMU_SAMPLING_FREQ/DEFAULT_SAMPLING_FREQ
#define THRESHOLD_MIN_BAT_VOLTAGE       3.5
#define STARTUP_TIMEOUT                 30000   //  In milliseconds

#define SAMPLE_DATA_LEN_ALL             20
#define SAMPLE_DATA_LEN_QUATERNIONS     8
#define SAMPLE_DATA_LEN                 SAMPLE_DATA_LEN_QUATERNIONS

// ================================================================
// ===          Define Bluetooth communication baudrate         ===
// ================================================================

#define BAUDRATE_9600               0x00
#define BAUDRATE_19200              0x01
#define BAUDRATE_38400              0x02
#define BAUDRATE_115200             0x03
#define BAUDRATE_230400             0x04
#define BAUDRATE_460800             0x05
#define BAUDRATE_921600             0x06

#define BT_UART_BAUDRATE            BAUDRATE_115200

// ================================================================
// ===                    Define MPU Settings                   ===
// ================================================================

    // If external clock 19.2MHz is available
#define MPU_EXTERNAL_CLOCK

// ================================================================
// ===                    Define MCU pins                       ===
// ================================================================

#define MPU_INT             2
#define BUTTON_INT          3
#define CHECK_BAT           4
#define BT_STATUS           5     
#define BT_CTS              6
#define BT_RTS              7
#define MPU_ON              8
#define IND_LED             9

#define V_BAT               A0
#define BAT_CHG             A1
#define BT_RES              A2
#define WAKE_UP_PIN         A3


enum Sensor_Reader_State{SLEEP, STARTUP, WAIT_FOR_CONNECTION, CALIBRATION, IDLE, SYNC, INIT_MES, RUNNING, CHARGING, BATTERY_LOW};
enum Data_Format_State{QUAT, GYRO, ACC, GYRO_ACC, QUART_GYRO_ACC};

//#define DEBUG



struct Variables {
  uint8_t buffer_counter = 0;                                   //  Global variable (number of sensor readouts before sending)
  uint8_t packet_number_counter = 1;                            //  Send packet after counter is same as PACK_NRS_BEFORE_SEND
  uint8_t mpu_read_counter = 0;                                 //  Counter for the indication led during RUNNING MODE
  uint8_t pack_nr_before_send = DEFAULT_PACK_NRS_BEFORE_SEND;   //  Counter for lower sample rates, ignoring samples
  uint16_t packet_send_counter = 0;                             //  Counter increments during RUNNING MODE after each BLE MSG
};

#endif