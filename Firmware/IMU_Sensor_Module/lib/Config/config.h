#ifndef CONFIG_h
#define CONFIG_h

#include "Arduino.h"


// ================================================================
// ===                    Define constants                      ===
// ================================================================

#define PACKETSIZE                  42
#define MAX_BUFFER_SIZE             10      //  Determine number of sensor read outs before sending
#define SAMPLING_FREQ               50
#define DEFAULT_SAMPLING_FREQ       100
#define PACK_NRS_BEFORE_SEND        DEFAULT_SAMPLING_FREQ/SAMPLING_FREQ
#define THRESHOLD_MIN_BAT_VOLTAGE   3.5

// ================================================================
// ===                    Define MCU pins                       ===
// ================================================================

#define MPU_INT             2
#define BUTTON_INT          3
#define CHECK_BAT           4
#define BT_STATUS           5     
#define BT_CTS              6
#define BT_RTS              7
#define MPU_ON              8
#define IND_LED             9

#define V_BAT               A0
#define BAT_CHG             A1
#define BT_RES              A2
#define WAKE_UP_PIN         A3



//#define DEBUG

#endif