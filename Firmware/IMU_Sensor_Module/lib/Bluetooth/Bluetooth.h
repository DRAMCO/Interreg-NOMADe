
#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include "Arduino.h"
#include "Config.h"


class BLUETOOTH {
    public:

        BLUETOOTH();

        void init();
        void reset();
        void BT_transmitData(uint8_t len, uint8_t *data);   
        void BT_transmitFrame(uint8_t cmd, uint16_t len, uint8_t * data);
        void BT_transmitFrame(uint8_t cmd, uint8_t data);
        void UART_Write_Block(uint8_t * data, uint8_t len);
        uint8_t calculateCS(uint8_t * data, uint8_t len);

        void BT_sleep_mode(void);
        void BT_wakeup(void);

    private:

};

#endif