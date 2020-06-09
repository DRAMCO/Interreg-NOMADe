/**
  ******************************************************************************
  * @file           : def.h
  * @brief          : defines
  ******************************************************************************
  */


#define BT_BAUDRATE					115200
#define COM_BAUDRATE				1000000 //921600
#define TABLET_BAUDRATE			1000000 //921600

#define NUMBER_OF_BT_PACKETS										2

#define PACKET_START_POS												11
#define NUMBER_OF_DATA_READS_IN_BT_PACKET				10
#define SIZE_SAMPLE_FRAME												8
#define SIZE_CHECKSUM														1
#define SIZE_BT_PACKET													92 //PACKET_START_POS + NUMBER_OF_DATA_READS_IN_BT_PACKET * SIZE_SAMPLE_FRAME + SIZE_CHECKSUM  //92
#define SIZE_PING_PONG_BUFFER										SIZE_BT_PACKET * NUMBER_OF_BT_PACKETS
#define SIZE_SD_CARD_READ_BUF										NUMBER_OF_BT_PACKETS * NUMBER_OF_DATA_READS_IN_BT_PACKET



#define I2C_ADDRESS_IC5_DATASHEET 	0x38
#define I2C_ADDRESS_IC6_DATASHEET   0x39
#define I2C_ADDRESS_IC7_DATASHEET   0x40

#define I2C_ADDRESS_IC5 						I2C_ADDRESS_IC5_DATASHEET << 1
#define I2C_ADDRESS_IC6   					I2C_ADDRESS_IC6_DATASHEET << 1
#define I2C_ADDRESS_IC7   					I2C_ADDRESS_IC7_DATASHEET << 1

//  Command Bytes -- Send to control register in the TCA9554A
#define CMD_REG_INPUT   						0x00
#define CMD_REG_OUTPUT  						0x01
#define CMD_REG_POL_INV 						0x02
#define CMD_REG_CONFIG  						0x03



// ================================================================
// ===       	Define Communication Commands (IMU Module)        ===
// ================================================================

#define IMU_SENSOR_MODULE_GET_STATUS               0x30

#define IMU_SENSOR_MODULE_SEND_BATTERY_VOLTAGE     0x40
#define IMU_SENSOR_MODULE_GET_BATTERY_VOLTAGE      0x41
#define IMU_SENSOR_MODULE_GET_BATTERY_LOW_ERROR    0x42

#define IMU_SENSOR_MODULE_SEND_START_SYNC          0x60
#define IMU_SENSOR_MODULE_GET_SYNC_DONE            0x61

#define IMU_SENSOR_MODULE_SEND_START_CALIBRATION   0x70
#define IMU_SENSOR_MODULE_GET_CANNOT_CALIBRATE     0x71
#define IMU_SENSOR_MODULE_GET_CALIBRATION_DONE     0x72
#define IMU_SENSOR_MODULE_GET_NEED_TO_CALIBRATE    0x73









