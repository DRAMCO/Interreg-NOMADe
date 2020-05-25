
#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include "Arduino.h"
#include "Config.h"


#define CMD_SCANSTART_REQ       0x09
#define CMD_SCANSTOP_REQ        0x0A
#define CMD_GETDEVICES_REQ      0x0B
#define CMD_RSSI_IND            0x8B


//  Setup connections

#define CMD_CONNECT_REQ         0x06
#define CMD_CONNECT_IND         0x86
#define CMD_SECURITY_IND        0x88
#define CMD_CHANNELOPEN_RSP     0xC6
#define CMD_DISCONNECT_REQ      0x07
#define CMD_DISCONNECT_IND      0x87
#define CMD_PHYUPDATE_REQ       0x1A
#define CMD_PHYUPDATE_IND       0x9A
#define CMD_PASSKEY_REQ         0x0D
#define CMD_PASSKEY_IND         0x8D
#define CMD_GETBONDS_REQ        0x0F
#define CMD_DELETEBONDS_REQ     0x0E


//  Transmit and receive data

#define CMD_DATA_REQ            0x04
#define CMD_TXCOMPLETE_RSP      0xC4
#define CMD_DATA_IND            0x84
#define CMD_SETBEACON_REQ       0x0C
#define CMD_BEACON_IND          0x8C


//  Configuring the module and modifying the divice settings

#define CMD_SET_REQ             0x11
#define CMD_GET_REQ             0x10


//  Mange the device state

#define CMD_GETSTATE_REQ        0x01
#define CMD_RESET_REQ           0x00
#define CMD_SLEEP_REQ           0x02
#define CMD_SLEEP_IND           0x82
#define CMD_FACTORYRESET_REQ    0x1C
#define CMD_UARTDISABLE_REQ     0x1B
#define CMD_UARTENABLE_IND      0x9B
#define CMD_BOOTLOADER_REQ      0x1F


//  Run the Bluetooth test modes

#define CMD_DTMSTART_REQ        0x1D
#define CMD_DTM_REQ             0x1E




//  Table 17
#define FS_FWVersion                0x01
#define RF_DeviceName               0x02
#define FS_MAC                      0x03
#define FS_BTMAC                    0x04
#define RF_AdvertisingTimeout       0x07
#define RF_ConnectionTiming         0x08
#define RF_ScanTiming               0x09
#define RF_ScanFactor               0x0A
#define UART_BaudrateIndex          0x0B
#define RF_SecFlags                 0x0C
#define RF_ScanFlags                0x0D
#define RF_BeaconFlags              0x0E
#define FS_DeviceInfo               0x0F
#define FS_SerialNumber             0x10
#define RF_TXPower                  0x11

//  Table 18
#define RF_StaticPasskey            0x12
#define DIS_Flags                   0x13
#define DIS_ManufacturerName        0x14
#define DIS_ModelNumber             0x15
#define DIS_SerialNumber            0x16
#define DIS_HWVersion               0x17
#define DIS_SWVersion               0x18
#define RF_Appearance               0x19
#define RF_SPPBaseUUID              0x1A
#define UART_Flags                  0x1B
#define CFG_Flags                   0x1C
#define RF_SecFlagsPerOnly          0x2C

class BLUETOOTH {
    public:

        BLUETOOTH();

        void init();
        void reset();
        void transmitData(uint8_t len, uint8_t *data);   
        void transmitFrame(uint8_t cmd, uint16_t len, uint8_t * data);
        void transmitFrame(uint8_t cmd, uint8_t data);
        void UART_Write_Block(uint8_t * data, uint8_t len);
        uint8_t calculateCS(uint8_t * data, uint8_t len);
        uint8_t isConnected();
        void disconnect();

        void sleep_mode(void);
        void wakeup(void);

    private:

};

#endif