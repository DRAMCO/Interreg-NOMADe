#ifndef CONFIG_h
#define CONFIG_h

#include "Arduino.h"

// ================================================================
// ===                    Define constants                      ===
// ================================================================

#define PACKETSIZE                      42
#define MAX_BUFFER_SIZE                 10      //  Determine number of sensor read outs before sending
#define DEFAULT_SAMPLING_FREQ           100
#define IMU_SAMPLING_FREQ               100
#define DEFAULT_PACK_NRS_BEFORE_SEND    IMU_SAMPLING_FREQ/DEFAULT_SAMPLING_FREQ
#define THRESHOLD_MIN_BAT_VOLTAGE       3.5
#define STARTUP_TIMEOUT                 30000   //  In milliseconds

// ================================================================
// ===          Define Bluetooth communication baudrate         ===
// ================================================================

#define BT_UART_BAUDRATE            BAUDRATE_115200

#define BAUDRATE_9600               0x00
#define BAUDRATE_19200              0x01
#define BAUDRATE_38400              0x02
#define BAUDRATE_115200             0x03
#define BAUDRATE_230400             0x04
#define BAUDRATE_460800             0x05
#define BAUDRATE_921600             0x06

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


enum Sensor_Reader_State{SLEEP, STARTUP, WAIT_FOR_CONNECTION, CALIBRATION, IDLE, SYNC, RUNNING, CHARGING, BATTERY_LOW};

//#define DEBUG

#endif