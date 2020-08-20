/**
  ******************************************************************************
  * @file    ble.h
  * @brief   This file contains the headers for the bluetooth methods.
	******************************************************************************
 **/

#ifndef PROTEUSII_DRIVER_H_
#define PROTEUSII_DRIVER_H_

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32h7xx_hal.h"
#include "uart_init.h"
#include "uart_com.h"

//  Table 10: Message overview: Requests
#define CMD_RESET_REQ           0x00    //  Reset the module
#define CMD_GETSTATE_REQ        0x01    //  Request the current module state
#define CMD_SLEEP_REQ           0x02    //  Go to sleep
#define CMD_DATA_REQ            0x04    //  Send data to the connected device
#define CMD_CONNECT_REQ         0x06    //  Setup a connection with another device
#define CMD_DISCONNECT_REQ      0x07    //  Close the connection
#define CMD_SCANSTART_REQ       0x09    //  Start scan
#define CMD_SCANSTOP_REQ        0x0A    //  Stop scan
#define CMD_GETDEVICES_REQ      0x0B    //  Request the scanned/detected devices
#define CMD_SETBEACON_REQ       0x0C    //  Place data in scan response packet
#define CMD_PASSKEY_REQ         0x0D    //  Respond to a pass key request
#define CMD_DELETEBONDS_REQ     0x0E    //  Delete bonding information 
#define CMD_GETBONDS_REQ        0x0F    //  Read the MACs of bonded devices 
#define CMD_GET_REQ             0x10    //  Read the module settings in flash
#define CMD_SET_REQ             0x11    //  Modify the module settings in flash
#define CMD_PHYUPDATE_REQ       0x1A    //  Update the PHY
#define CMD_UARTDISABLE_REQ     0x1B    //  Disable the UART
#define CMD_FACTORYRESET_REQ    0x1C    //  Perform a factory reset
#define CMD_DTMSTART_REQ        0x1D    //  Enable the direct test mode
#define CMD_DTM_REQ             0x1E    //  Start/stop a test of the direct test mode
#define CMD_BOOTLOADER_REQ      0x1F    //  Switch to the bootloader

//  Table 11: Message overview: Confirmations
#define CMD_RESET_CNF           0x40    //  Reset request received
#define CMD_GETSTATE_CNF        0x41    //  Return the current module state
#define CMD_SLEEP_CNF           0x42    //  Sleep request received
#define CMD_DATA_CNF            0x44    //  Data transmission request received
#define CMD_CONNECT_CNF         0x46    //  Connection setup request received
#define CMD_DISCONNECT_CNF      0x47    //  Disconnection request received
#define CMD_SCANSTART_CNF       0x49    //  Scan started
#define CMD_SCANSTOP_CNF        0x4A    //  Scan stopped
#define CMD_GETDEVICES_CNF      0x4B    //  Output the scanned/detected devices
#define CMD_SETBEACON_CNF       0x4C    //  Data is placed in scan response packet
#define CMD_PASSKEY_CNF         0x4D    //  Received the pass key
#define CMD_DELETEBONDS_CNF     0x4E    //  Deleted bonding information 
#define CMD_GETBONDS_CNF        0x4F    //  Return the MAC of all bonded devices 
#define CMD_GET_CNF             0x50    //  Return the requested module flash settings
#define CMD_SET_CNF             0x51    //  Module flash settings have been modified
#define CMD_PHYUPDATE_CNF       0x5A    //  Update Phy request received
#define CMD_UARTDISABLE_CNF     0x5B    //  Disable UART request received
#define CMD_FACTORYRESET_CNF    0x5C    //  Factory reset request received
#define CMD_DTMSTART_CNF        0x5D    //  Enable the direct test mode now
#define CMD_DTM_CNF             0x5E    //  Test of direct test mode started/stopped
#define CMD_BOOTLOADER_CNF      0x5F    //  Will switch to bootloader now

// Table 12: Message overview: Indications
#define CMD_SLEEP_IND           0x82    //  State will be changed to ACTION_SLEEP
#define CMD_DATA_IND            0x84    //  Data has been received
#define CMD_CONNECT_IND         0x86    //  Connection established
#define CMD_DISCONNECT_IND      0x87    //  Disconnected
#define CMD_SECURITY_IND        0x88    //  Secured connection established
#define CMD_RSSI_IND            0x8B    //  Advertising package detected
#define CMD_BEACON_IND          0x8C    //  Received Beacon data
#define CMD_PASSKEY_IND         0x8D    //  Received a pass key request 
#define CMD_PHYUPDATE_IND       0x9A    //  PHY has been updated
#define CMD_UARTENABLE_IND      0x9B    //  UART was re-enabled
#define CMD_ERROR_IND           0xA2    //  Entered error state
#define CMD_TXCOMPLETE_RSP      0xC4    //  Data has been sent
#define CMD_CHANNELOPEN_RSP     0xC6    //  Channel open, data transmission possible

/*
 * Following defines gives the "Setting index" 
 * Use command CMD_GET_REQ or CMD_SET_REQ
 */

//  Table 17: Table of settings (Part 1)
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

//  Table 18: Table of settings (Part 2)
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


	//	Init and other functions
void BT_Initialize(void);
void BT_getMAC(UART_HandleTypeDef *haurt);

	//  Connection management functions
void BT_connect(UART_HandleTypeDef *haurt, uint8_t *mac);
void BT_disconnect(UART_HandleTypeDef *haurt);

	//  Scan management functions
void BT_changeScanTiming(UART_HandleTypeDef *haurt);
void BT_changeScanFactor(UART_HandleTypeDef *haurt);
void BT_startScanning(UART_HandleTypeDef *haurt);
void BT_stopScanning(UART_HandleTypeDef *haurt);

		//  Baudrate management functions
void BT_updateBaudrate(UART_HandleTypeDef *haurt, uint8_t baudrate_index);
uint8_t BT_getUARTBaudrate(UART_HandleTypeDef *haurt);
uint8_t BT_setUARTBaudrate(UART_HandleTypeDef *haurt, uint8_t baudrate_index);

		//  Send message through Bluetooth
void BT_transmitMsg_CMD(UART_HandleTypeDef *haurt, uint8_t communication_cmd);
void BT_transmitMsg_CMD_Data(UART_HandleTypeDef *haurt, uint8_t cmd, uint8_t len, uint8_t * sndbuf);

		//  Transmit messages to BT module
void BT_transmitData(UART_HandleTypeDef *haurt, uint8_t len, uint8_t *data);   
void BT_transmit_CMD_Bytes(UART_HandleTypeDef *haurt, uint8_t cmd, uint16_t len, uint8_t * data);
void BT_transmit_CMD_Byte(UART_HandleTypeDef *haurt, uint8_t cmd, uint8_t data);
void BT_transmit_CMD(UART_HandleTypeDef *haurt, uint8_t cmd);


		//  Receive BT module messages
/*
uint8_t BT_receiveFrame_TimeOut(uint8_t * rsvbuf, uint16_t timeout);
uint8_t BT_receiveFrame(uint8_t * rsvbuf);
*/

//	Checksum calculation
uint8_t BT_calculateCS(uint8_t * data, uint8_t len);


#ifdef __cplusplus
}
#endif

#endif
/**********************************END OF FILE**********************************/
