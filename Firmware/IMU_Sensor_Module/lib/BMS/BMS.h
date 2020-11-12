#ifndef _BMS_H_
#define _BMS_H_

#include "Arduino.h"
#include "Config.h"


class BMS {
    public:

        BMS();

        void init();
        float getBatVoltage();
        bool batStatus();
        bool batCharging();

    private:

};

#endif