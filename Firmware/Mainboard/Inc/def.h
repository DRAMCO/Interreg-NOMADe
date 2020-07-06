/**
  ******************************************************************************
  * @file           : def.h
  * @brief          : defines
  ******************************************************************************
  */


#define BT_BAUDRATE					115200
#define COM_BAUDRATE				1000000 //921600
#define TABLET_BAUDRATE			1000000 //921600

#define NUMBER_OF_BT_PACKETS										1 // 2

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

#define IMU_SENSOR_MODULE_REQ_STATUS                0x30
#define IMU_SENSOR_MODULE_IND_STATUS                0x31

#define IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE       0x40
#define IMU_SENSOR_MODULE_IND_BATTERY_VOLTAGE       0x41
#define IMU_SENSOR_MODULE_IND_BATTERY_LOW_ERROR     0x42

#define IMU_SENSOR_MODULE_REQ_END_MEASUREMENTS      0x50
#define IMU_SENSOR_MODULE_IND_MEASUREMENTS_ENDED    0x51

#define IMU_SENSOR_MODULE_REQ_START_SYNC            0x60
#define IMU_SENSOR_MODULE_IND_SYNC_DONE             0x61

#define IMU_SENSOR_MODULE_REQ_START_CALIBRATION     0x70
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









