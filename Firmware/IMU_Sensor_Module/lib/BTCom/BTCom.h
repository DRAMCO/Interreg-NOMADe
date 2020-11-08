#ifndef _BT_COM_H_
#define _BT_COM_H_

#include "Arduino.h"
#include "Config.h"
#include "Bluetooth.h"
#include "BMS.h"
#include "MPU6050.h"


// ================================================================
// ===              Define Communication Commands               ===
// ================================================================

#define IMU_SENSOR_MODULE_REQ_SEND_DATA             0x20

#define IMU_SENSOR_MODULE_REQ_STATUS                0x30
#define IMU_SENSOR_MODULE_IND_STATUS                0x31

#define IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE       0x40
#define IMU_SENSOR_MODULE_IND_BATTERY_VOLTAGE       0x41
#define IMU_SENSOR_MODULE_IND_BATTERY_LOW_ERROR     0x42

#define IMU_SENSOR_MODULE_REQ_START_SYNC            0x50
#define IMU_SENSOR_MODULE_IND_SYNC_STARTED          0x51
#define IMU_SENSOR_MODULE_IND_SYNC_DONE             0x52
#define IMU_SENSOR_MODULE_IND_CANNOT_SYNC           0x53
#define IMU_SENSOR_MODULE_IND_NEED_TO_SYNCHRONISE   0x54
#define IMU_SENSOR_MODULE_REQ_GET_SYNC_TIME         0x55
#define IMU_SENSOR_MODULE_IND_SYNC_TIME             0x56
#define IMU_SENSOR_MODULE_REQ_CHANGE_SYNC_TIME      0x57
#define IMU_SENSOR_MODULE_IND_SYNC_TIME_CHANGED     0x58

#define IMU_SENSOR_MODULE_REQ_STOP_MEASUREMENTS     0x60
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_STOPPED  0x61
#define IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS    0x62
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_STARTED  0x63

#define IMU_SENSOR_MODULE_REQ_START_CALIBRATION     0x70
#define IMU_SENSOR_MODULE_IND_CALIBRATION_STARTED   0x74
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

#define IMU_SENSOR_MODULE_REQ_CHANGE_BAT_LOW_VOLT   0xA0    // Hier moet extra data mee worden doorgestuurd
#define IMU_SENSOR_MODULE_IND_BAT_LOW_VOLT_CHANGED  0xA1

#define IMU_SENSOR_MODULE_REQ_MILLIS                0xB0
#define IMU_SENSOR_MODULE_RSP_MILLIS                0xB1

#define ADV_MSG                                     0xF0

extern uint8_t state;

extern bool calibration;
extern bool status_periferals;
extern bool sync_now;
extern bool sync_executed;
extern bool synchronisation;
extern bool battery_low_state;

extern uint32_t sync_time;
extern uint32_t startup_time;

class BTCOM {
    public:
        BTCOM(BLUETOOTH *bt, BMS *bms, MPU6050 *mpu, Variables *counters);

        void incoming_data_handler();
        void communication_management(uint8_t *rsv_buffer);
        void rsv_msg_handler(uint8_t *rsv_buffer);

    private:
        BLUETOOTH *bt;
        BMS *bms;
        MPU6050 *mpu;
        Variables *counters;
};

#endif