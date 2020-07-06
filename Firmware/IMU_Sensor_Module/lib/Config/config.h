#ifndef CONFIG_h
#define CONFIG_h

#include "Arduino.h"


// ================================================================
// ===                    Define constants                      ===
// ================================================================

#define PACKETSIZE                      42
#define MAX_BUFFER_SIZE                 10      //  Determine number of sensor read outs before sending
#define DEFAULT_SAMPLING_FREQ           25
#define IMU_SAMPLING_FREQ               100
#define DEFAULT_PACK_NRS_BEFORE_SEND    IMU_SAMPLING_FREQ/DEFAULT_SAMPLING_FREQ
#define THRESHOLD_MIN_BAT_VOLTAGE       3.5

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

#define IMU_SENSOR_MODULE_REQ_STATUS                0x30
#define IMU_SENSOR_MODULE_IND_STATUS                0x31

#define IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE       0x40
#define IMU_SENSOR_MODULE_IND_BATTERY_VOLTAGE       0x41
#define IMU_SENSOR_MODULE_IND_BATTERY_LOW_ERROR     0x42

#define IMU_SENSOR_MODULE_REQ_START_SYNC            0x60
#define IMU_SENSOR_MODULE_IND_SYNC_DONE             0x61
#define IMU_SENSOR_MODULE_REQ_STOP_MEASUREMENTS     0x62
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_STOPPED  0x63

#define IMU_SENSOR_MODULE_REQ_START_CALIBRATION     0x70
#define IMU_SENSOR_MODULE_IND_CANNOT_CALIBRATE      0x71
#define IMU_SENSOR_MODULE_IND_CALIBRATION_DONE      0x72
#define IMU_SENSOR_MODULE_IND_NEED_TO_CALIBRATE     0x73

#define IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED 0x80
#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_10HZ    0x81
#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_20HZ    0x82
#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_25HZ    0x83
#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_50HZ    0x84
#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_100HZ   0x85

#define IMU_SENSOR_MODULE_REQ_GO_TO_SLEEP           0x90
#define IMU_SENSOR_MODULE_IND_SLEEP_MODE            0x91

//#define DEBUG

#endif