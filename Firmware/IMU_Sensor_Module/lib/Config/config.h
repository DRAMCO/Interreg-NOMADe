#ifndef CONFIG_h
#define CONFIG_h

#include "Arduino.h"


// ================================================================
// ===                    Define constants                      ===
// ================================================================

#define PACKETSIZE                  42
#define MAX_BUFFER_SIZE             10      //  Determine number of sensor read outs before sending
#define SAMPLING_FREQ               50
#define DEFAULT_SAMPLING_FREQ       100
#define PACK_NRS_BEFORE_SEND        DEFAULT_SAMPLING_FREQ/SAMPLING_FREQ
#define THRESHOLD_MIN_BAT_VOLTAGE   3.5

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



// ================================================================
// ===              Define Communication Commands               ===
// ================================================================

#define IMU_SENSOR_MODULE_GET_STATUS                0x30

#define IMU_SENSOR_MODULE_GET_BATTERY_VOLTAGE       0x40
#define IMU_SENSOR_MODULE_SEND_BATTERY_VOLTAGE      0x41
#define IMU_SENSOR_MODULE_SEND_BATTERY_LOW_ERROR    0x42

#define IMU_SENSOR_MODULE_GET_START_SYNC            0x60
#define IMU_SENSOR_MODULE_SEND_SYNC_DONE            0x61

#define IMU_SENSOR_MODULE_GET_START_CALIBRATION     0x70
#define IMU_SENSOR_MODULE_SEND_CANNOT_CALIBRATE     0x71
#define IMU_SENSOR_MODULE_SEND_CALIBRATION_DONE     0x72
#define IMU_SENSOR_MODULE_SEND_NEED_TO_CALIBRATE    0x73

//#define DEBUG

#endif