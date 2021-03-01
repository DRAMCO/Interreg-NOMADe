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

#define IMU_SENSOR_MODULE_RSP_SEND_DATA_F1                              0x20    //  QUAT
#define IMU_SENSOR_MODULE_RSP_SEND_DATA_F2                              0x21    //  GYRO
#define IMU_SENSOR_MODULE_RSP_SEND_DATA_F3                              0x22    //  ACC
#define IMU_SENSOR_MODULE_RSP_SEND_DATA_F4                              0x23    //  GYRO + ACC
#define IMU_SENSOR_MODULE_RSP_SEND_DATA_F5                              0x24    //  QUAT + GYRO + ACC
#define IMU_SENSOR_MODULE_REQ_CHANGE_DF                                 0x25
#define IMU_SENSOR_MODULE_IND_DF_CHANGED                                0x26

#define DATA_FORMAT_1                                                   0x01
#define DATA_FORMAT_2                                                   0x02
#define DATA_FORMAT_3                                                   0x03
#define DATA_FORMAT_4                                                   0x04
#define DATA_FORMAT_5                                                   0x05

#define IMU_SENSOR_MODULE_REQ_STATUS                                    0x30
#define IMU_SENSOR_MODULE_IND_STATUS                                    0x31

#define IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE                           0x40
#define IMU_SENSOR_MODULE_IND_BATTERY_VOLTAGE                           0x41
#define IMU_SENSOR_MODULE_IND_BATTERY_LOW_ERROR                         0x42

#define IMU_SENSOR_MODULE_REQ_START_SYNC                                0x50
#define IMU_SENSOR_MODULE_IND_SYNC_STARTED                              0x51
#define IMU_SENSOR_MODULE_IND_SYNC_DONE                                 0x52
#define IMU_SENSOR_MODULE_IND_CANNOT_SYNC                               0x53
#define IMU_SENSOR_MODULE_IND_NEED_TO_SYNCHRONISE                       0x54
#define IMU_SENSOR_MODULE_REQ_GET_SYNC_TIME                             0x55
#define IMU_SENSOR_MODULE_IND_SYNC_TIME                                 0x56
#define IMU_SENSOR_MODULE_REQ_CHANGE_SYNC_TIME                          0x57
#define IMU_SENSOR_MODULE_IND_SYNC_TIME_CHANGED                         0x58

#define IMU_SENSOR_MODULE_REQ_STOP_MEASUREMENTS                         0x60
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_STOPPED                      0x61
#define IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS                        0x62
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_STARTED                      0x63
#define IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS_WITHOUT_SYNC           0x64
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_STARTED_WITHOUT_SYNC         0x65

#define IMU_SENSOR_MODULE_REQ_START_CALIBRATION                         0x70
#define IMU_SENSOR_MODULE_IND_CALIBRATION_STARTED                       0x74
#define IMU_SENSOR_MODULE_IND_CANNOT_CALIBRATE                          0x71
#define IMU_SENSOR_MODULE_IND_CALIBRATION_DONE                          0x72
#define IMU_SENSOR_MODULE_IND_NEED_TO_CALIBRATE                         0x73

#define IMU_SENSOR_MODULE_REQ_SAMPLING_FREQ_CHANGED                     0x80
#define IMU_SENSOR_MODULE_IND_SAMPLING_FREQ_CHANGED                     0x81
#define SAMPLING_FREQ_10HZ                                              0x01
#define SAMPLING_FREQ_20HZ                                              0x02
#define SAMPLING_FREQ_25HZ                                              0x03
#define SAMPLING_FREQ_50HZ                                              0x04
#define SAMPLING_FREQ_100HZ                                             0x05

#define IMU_SENSOR_MODULE_REQ_GO_TO_SLEEP                               0x90
#define IMU_SENSOR_MODULE_IND_SLEEP_MODE                                0x91

#define IMU_SENSOR_MODULE_REQ_CHANGE_BAT_LOW_VOLT                       0xA0    // Hier moet extra data mee worden doorgestuurd
#define IMU_SENSOR_MODULE_IND_BAT_LOW_VOLT_CHANGED                      0xA1

#define IMU_SENSOR_MODULE_REQ_MILLIS                                    0xB0
#define IMU_SENSOR_MODULE_RSP_MILLIS                                    0xB1

#define IMU_SENSOR_MODULE_REQ_SW_VERSION                                0xC0
#define IMU_SENSOR_MODULE_RSP_SW_VERSION                                0xC1

#define ADV_MSG                                                         0xF0

extern uint8_t state;
extern uint8_t dataformat;

extern bool calibration;
extern bool status_periferals;
extern bool sync_now;
extern bool sync_executed;
extern bool synchronisation;
extern bool battery_low_state;
extern bool baud_correct;

extern uint32_t sync_time;
extern uint32_t startup_time;
extern uint32_t baudrate_array [];
extern uint32_t baud_pointer;

class BTCOM {
    public:
        BTCOM(BLUETOOTH *bt, BMS *bms, MPU6050 *mpu, Variables *counters);

        void incoming_data_handler();
        void communication_management(uint8_t *rsv_buffer);
        void rsv_msg_handler(uint8_t *rsv_buffer);
        void update_baudrate();

        void controle_check_baudrate();
        bool baudrate_ok();

    private:
        BLUETOOTH *bt;
        BMS *bms;
        MPU6050 *mpu;
        Variables *counters;

        bool baud_changed = false;
        bool check_baud = true;
        uint8_t counter_baud_check = 0;
};

#endif