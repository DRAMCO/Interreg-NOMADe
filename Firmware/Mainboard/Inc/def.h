/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */


#define BT_BAUDRATE			115200
#define COM_BAUDRATE		1000000 //921600


#define PACKET_START_POS												11
#define NUMBER_OF_DATA_READS_IN_BT_PACKET				10
#define NUMBER_OF_BT_PACKETS										2
#define SIZE_BT_PACKET													92
#define SIZE_PING_PONG_BUFFER										SIZE_BT_PACKET * NUMBER_OF_BT_PACKETS


