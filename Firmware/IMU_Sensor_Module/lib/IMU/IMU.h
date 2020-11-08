#ifndef _IMU_H_
#define _IMU_H_

#include "Arduino.h"
#include "Config.h"
#include "DMP.h"
#include "Bluetooth.h"

extern uint8_t state;
extern struct Variables counters;


class IMU {
    public:

        IMU(DMP *dmp, MPU6050 *mpu, Variables *counters);

        void powerup(void);
        void powerdown(void);
        uint8_t getIMUData(uint8_t *len);
        uint8_t* getDataBuffer();
        void reset_counters(void);
        void calibrate(void);


    private:
        MPU6050 *mpu;
        DMP *dmp;
        Variables *counters;
        uint8_t data_buffer[255];
};

#endif