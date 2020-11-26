#ifndef _IMU_H_
#define _IMU_H_

#include "Arduino.h"
#include "Config.h"
#include "DMP.h"
#include "LED.h"

extern uint8_t state;
extern uint8_t dataformat;
extern struct Variables counters;


class IMU {
    public:

        IMU(DMP *dmp, MPU6050 *mpu, Variables *counters, LED *led);

        void powerup(void);
        void powerdown(void);
        void initMeasurement();
        uint8_t getIMUData(uint8_t *len, uint8_t *send_buffer);
        uint8_t* getDataBuffer();
        void reset_counters(void);
        void calibrate(void);


    private:
        MPU6050 *mpu;
        DMP *dmp;
        Variables *counters;
        LED *led;
        uint8_t data_buffer[255];
        uint8_t pstruc_start = 0, pstruc_stop = 0;
};

#endif