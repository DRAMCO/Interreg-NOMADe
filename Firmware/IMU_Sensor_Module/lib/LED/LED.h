
#ifndef _LED_H_
#define _LED_H_

#include "Arduino.h"
#include "Config.h"


class LED {
    public:

        LED(Variables *counters);

        void blink(uint8_t num, uint16_t wait_time);
        void blink(uint8_t num, uint16_t on_time, uint16_t off_time);
        void blink_Running();

        void on(void);
        void off(void);

    private:
        uint8_t on_counter = 0;
        uint8_t off_counter = 0;
        Variables *counters;

};

#endif