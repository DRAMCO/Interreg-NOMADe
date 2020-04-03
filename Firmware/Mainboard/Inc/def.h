/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */


#define BT_BAUDRATE			115200
#define COM_BAUDRATE		1000000 //921600

#define NUMBER_OF_BT_PACKETS										2

#define PACKET_START_POS												11
#define NUMBER_OF_DATA_READS_IN_BT_PACKET				10
#define SIZE_SAMPLE_FRAME												8
#define SIZE_CHECKSUM														1
#define SIZE_BT_PACKET													92 //PACKET_START_POS + NUMBER_OF_DATA_READS_IN_BT_PACKET * SIZE_SAMPLE_FRAME + SIZE_CHECKSUM  //92
#define SIZE_PING_PONG_BUFFER										SIZE_BT_PACKET * NUMBER_OF_BT_PACKETS


