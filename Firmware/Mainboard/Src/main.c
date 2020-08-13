/*  ____  ____      _    __  __  ____ ___
 * |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
 * | | | | |_) |  / _ \ | |\/| | |  | | | |
 * | |_| |  _ <  / ___ \| |  | | |__| |_| |
 * |____/|_| \_\/_/   \_\_|  |_|\____\___/
 *                           research group
 *                             dramco.be/
 *
 *  KU Leuven - Technology Campus Gent,
 *  Gebroeders De Smetstraat 1,
 *  B-9000 Gent, Belgium
 *
 *         File: main.c
 *      Created: 2020-02-27
 *       Author: Jarne Van Mulders
 *      Version: V1.0
 *
 *  Description: Firmware IMU sensor module for the NOMADe project
 *
 *  Interreg France-Wallonie-Vlaanderen NOMADe
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "config.h"
#include "def.h"
#include "uart_init.h"
//#include "uart_com.h"
#include "usb_com.h"
//#include "imu_com.h"
//#include "proteusII_driver.h"
//#include "ble_lib.h"
#include <string.h>
#include <stdio.h>
//#include "UartRingBuffer.h"
//#include "UartRingBufferManager.h"

FATFS myFATAFS = {0};
FIL myFILE;
UINT testByte;

float PI = 3.14159265358979323846;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;  //  BT1
UART_HandleTypeDef huart5;  //  CH340
UART_HandleTypeDef huart7;  //  BT3
UART_HandleTypeDef huart8;  //  BT4
UART_HandleTypeDef huart1;  //  BT5
UART_HandleTypeDef huart2;  //  BT6
UART_HandleTypeDef huart3;  //  FT312D -- TABLET
UART_HandleTypeDef huart6;  //  BT2



// Make IMU module 1 with hardcoded MAC address
imu_module imu_1 = {&huart4, {0xC8, 0x0B, 0x20, 0xDA, 0x18, 0x00}, "IMU Module 1: ", 0, 0, 0 };


/* Read from ringbuffer ------------------------------------------------------*/
typedef struct{
	uint8_t rsvbuf [200];
	volatile uint8_t reading_frame;
	volatile uint8_t header_readed;
	volatile uint16_t len;
	volatile uint8_t rd_len_counter;
} read_from_ringbuffer;

read_from_ringbuffer uart1_buffer = { {0}, 0, 0, 0, 0};
read_from_ringbuffer uart2_buffer = { {0}, 0, 0, 0, 0};
read_from_ringbuffer uart3_buffer = { {0}, 0, 0, 0, 0};
read_from_ringbuffer uart4_buffer = { {0}, 0, 0, 0, 0};
read_from_ringbuffer uart5_buffer = { {0}, 0, 0, 0, 0};
read_from_ringbuffer uart6_buffer = { {0}, 0, 0, 0, 0};
read_from_ringbuffer uart7_buffer = { {0}, 0, 0, 0, 0};
read_from_ringbuffer uart8_buffer = { {0}, 0, 0, 0, 0};



uint16_t file_nummer = 0;

uint8_t gpio_buf_IC5 = 0x00;

int num_send_1 = 0, num_send_2 = 0;



// ================================================================
// ===                    Static functions                      ===
// ================================================================

static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM2_Init(void);


static void startUpLeds(void);


/********Check packets in UART Ringbuffers**********/
void BT_read_ringbuffer_pakket(UART_HandleTypeDef *huart, read_from_ringbuffer *buffer, imu_module *imu);

/********Handlers**************/
static void rsv_bt_packet_handler(uint8_t * rsvbuf, uint8_t len, imu_module *imu);
static void rsv_data_msg_handler(uint8_t * rsvbuf, uint8_t len, imu_module *imu);


/**********IOExpander**********/
static void IOExpander_init(I2C_HandleTypeDef *hi2c, uint8_t address);
static void IOExpander_set(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t io);
static void IOExpander_clear(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t io);
static void IOExpander_clearAll(I2C_HandleTypeDef *hi2c, uint8_t address);
static void IOExpander_update(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t buf);


/**********SD Card************/
static void SDCard_create_new_file(void);
static FRESULT SDCard_mount(void);


/*********Calculations********/
static void convertBuffer(uint8_t * buf, uint8_t sensor_number);
static void getYawPitchRoll(int16_t *data, float *newdata);


/************Other************/
static void printSystemTick(void);




/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize GPIO */
  MX_GPIO_Init();
  
  /* Initialize I2C1 */
  MX_I2C1_Init();
  
  /* Initialize timer */
  MX_TIM2_Init();
  
  /* Initialize SD card periferals */
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  
  /* Initialize UART */
  MX_UART4_Init(BT_BAUDRATE);
  MX_UART5_Init(COM_BAUDRATE);
  MX_UART7_Init(BT_BAUDRATE);
  MX_UART8_Init(BT_BAUDRATE);
  MX_USART1_UART_Init(BT_BAUDRATE);
  MX_USART2_UART_Init(BT_BAUDRATE);
  MX_USART3_UART_Init(TABLET_BAUDRATE);
  MX_USART6_UART_Init(BT_BAUDRATE);
	
	
	UART_Ringbuffer_Init(&huart4);
	UART_Ringbuffer_Init(&huart5);

	USB_COM_print("************************************\n   NOMADe Mainboard V1.0\n************************************\n");
 
	USB_COM_show_menu();
 
  /* Start timer 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  
  /* Start leds */
  startUpLeds();
  
  /* Mount SD Card */
  FRESULT res = SDCard_mount();
  
  /* Check for file to write the data */
  if(res == FR_OK){
    HAL_GPIO_TogglePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin);
    SDCard_create_new_file();
    HAL_GPIO_TogglePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin);
  }
	
	//IMU_connect(&imu_1);
	
	/*
	uint8_t b [] = {RF_ScanTiming, 0x02};
  BT_transmit_CMD_Bytes(&huart4, CMD_SET_REQ, 2, b);
	HAL_Delay(200);
 */
	
  /* Connect with all the Bluetooth modules */
  //SENS_connect(1);
	//SENS_connect(2);
	//SENS_connect(3);
  
  //HAL_Delay(1000);
  
  
  /* Clear UART4 buffer */
  //HAL_UART_Abort(&huart4);
	//HAL_UART_Abort(&huart6);
	//HAL_UART_Abort(&huart7);
	
	
	//uint8_t dat [1]; 
	//uint8_t buffer [100];
	
	
	//----------- PRINT BATTERY VOLTAGE ----------//
	//BT_transmitMsg_CMD(&huart4, IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE);						//****	SENSOR 1: REQ BATTERY VOLTAGE	****//
	
	//HAL_UART_Receive(&huart4, buffer, 12, 1000);					//****	SENSOR 1: GET FRAME "MSG SENDED"	****//
	
	//HAL_UART_Receive(&huart4, buffer, 15, 1000); 					//****	SENSOR 1: GET AND CHECK ANWSER	****//
	
	//float voltage = (buffer[12] | buffer[13] << 8)/100.0;
	//sprintf(string, "Battery voltage: %.02f V\n", voltage); 
	//HAL_UART_Transmit(&huart5, (uint8_t *)string, strlen(string), 25);
	
	
	
	//BT_transmitMsg_CMD(&huart6, IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE);						//****	SENSOR 2: REQ BATTERY VOLTAGE	****//
	
	//HAL_UART_Receive(&huart6, buffer, 12, 1000);					//****	SENSOR 2: GET FRAME "MSG SENDED"	****//
	
	//HAL_UART_Receive(&huart6, buffer, 15, 1000); 					//****	SENSOR 2: GET AND CHECK ANWSER	****//
	
	//voltage = (buffer[12] | buffer[13] << 8)/100.0;
	//sprintf(string, "Battery voltage: %.02f V\n", voltage); 
	//HAL_UART_Transmit(&huart5, (uint8_t *)string, strlen(string), 25);
	
	
	
	//BT_transmitMsg_CMD(&huart7, IMU_SENSOR_MODULE_REQ_BATTERY_VOLTAGE);						//****	SENSOR 3: REQ BATTERY VOLTAGE	****//
	
	//HAL_UART_Receive(&huart7, buffer, 12, 1000);					//****	SENSOR 3: GET FRAME "MSG SENDED"	****//
	
	//HAL_UART_Receive(&huart7, buffer, 15, 1000); 					//****	SENSOR 3: GET AND CHECK ANWSER	****//
	
	//voltage = (buffer[12] | buffer[13] << 8)/100.0;
	//sprintf(string, "Battery voltage: %.02f V\n", voltage); 
	//HAL_UART_Transmit(&huart5, (uint8_t *)string, strlen(string), 25);
	
	
	//-------------------------------------------//
	
	
		/* Clear UART4 buffer */
  //HAL_UART_Abort(&huart4);
	////HAL_UART_Abort(&huart6);
	//HAL_UART_Abort(&huart7);
	
	//HAL_Delay(100);
	
	
	//------------- CALIBRATIE -------------//
	
	//BT_transmitMsg_CMD(&huart4, IMU_SENSOR_MODULE_REQ_START_CALIBRATION);						//****	SENSOR 1: SEND CALIBRATION START MSG	****//
	//BT_transmitMsg_CMD(&huart6, IMU_SENSOR_MODULE_REQ_START_CALIBRATION);						//****	SENSOR 2: SEND CALIBRATION START MSG	****//
	//BT_transmitMsg_CMD(&huart7, IMU_SENSOR_MODULE_REQ_START_CALIBRATION);						//****	SENSOR 3: SEND CALIBRATION START MSG	****//
	
	//ja &huart4->RxXferCount
	
	
	//HAL_UART_Receive(&huart4, buffer, 12, 1000);					//****	SENSOR 1: GET FRAME "MSG SENDED"	****//
	//HAL_UART_Receive(&huart6, buffer, 12, 1000);					//****	SENSOR 2: GET FRAME "MSG SENDED"	****//
	//HAL_UART_Receive(&huart7, buffer, 12, 1000);					//****	SENSOR 3: GET FRAME "MSG SENDED"	****//
	
	//HAL_UART_Transmit(&huart5, buffer, 12, 1000);   		// 	DEBUG PRINT -- eventueel nakijken??
	
	
	//HAL_Delay(3000);																			//	Waiting during the calibration	//
	
	
	//HAL_UART_Receive(&huart4, buffer, 13, 1000); 					//****	SENSOR 1: GET AND CHECK ANWSER	****//
	//HAL_UART_Receive(&huart6, buffer, 13, 1000); 					//****	SENSOR 2: GET AND CHECK ANWSER	****//
	//HAL_UART_Receive(&huart7, buffer, 13, 1000); 					//****	SENSOR 2: GET AND CHECK ANWSER	****//
	
	//HAL_UART_Transmit(&huart5, buffer, 13, 1000);   		// 	DEBUG PRINT -- eventueel nakijken??
	
	//HAL_Delay(1000);
	
	// 	CHECK ANSWER -- EVENTUALLY DO RECALIBRATION
	
	//-------------------------------------//
	
	// !!! CHECK IF SYNC IS DONE OF ALL THE MODULES AND SEND SYNC NOW 
	
	//----------- START GELIJKTIJDIG DE SENSOREN OP ----------//			
	//BT_transmitMsg_CMD(&huart4, IMU_SENSOR_MODULE_REQ_START_SYNC);						//****	SENSOR 1: SEND START MEASUREMENTS MSG	****//
	//BT_transmitMsg_CMD(&huart6, IMU_SENSOR_MODULE_REQ_START_SYNC);						//****	SENSOR 2: SEND START MEASUREMENTS MSG	****//
	//BT_transmitMsg_CMD(&huart7, IMU_SENSOR_MODULE_REQ_START_SYNC);						//****	SENSOR 2: SEND START MEASUREMENTS MSG	****//
	//uint32_t tick_now = HAL_GetTick();
	
	//HAL_UART_Receive(&huart4, buffer, 12, 1000);					//****	SENSOR 1: GET FRAME "MSG SENDED"	****//
	//HAL_UART_Receive(&huart6, buffer, 12, 1000);					//****	SENSOR 1: GET FRAME "MSG SENDED"	****//
	
	//HAL_UART_Transmit(&huart5, buffer, 12, 1000);   		// 	DEBUG PRINT -- eventueel nakijken??
	
	//HAL_UART_Receive(&huart4, buffer, 13, 1000); 					//****	SENSOR 1: GET AND CHECK ANWSER	****//
	//HAL_UART_Receive(&huart6, buffer, 13, 1000); 					//****	SENSOR 2: GET AND CHECK ANWSER	****//
	// 	CHECK ANSWER -- EVENTUALLY ABORT IF THERE WAS A SENSOR NOT STARTED
	
	//--------------------------------------------------------//
	
	//HAL_Delay(1000);
	
	
	//uint8_t buf [4]	= {ADV_MSG};
	//BT_transmit_CMD_Bytes(&huart4, 0x0C, 0x01, buf);
	//HAL_UART_Receive(&huart4, buffer, 6, 1000);	
	
	//HAL_Delay(500);
	
	//SENS_connect(1);
	//SENS_connect(2);
	//SENS_connect(3);
	
	//while(HAL_GetTick() < tick_now + 4000){
		//HAL_Delay(10);
	//}
	
	
	
		//----------- START GELIJKTIJDIG DE SENSOREN OP ----------//		
	//BT_transmitMsg_CMD(&huart4, IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS);						//****	SENSOR 1: SEND START MEASUREMENTS MSG	****//
	//BT_transmitMsg_CMD(&huart6, IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS);						//****	SENSOR 2: SEND START MEASUREMENTS MSG	****//
	//BT_transmitMsg_CMD(&huart7, IMU_SENSOR_MODULE_REQ_START_MEASUREMENTS);						//****	SENSOR 2: SEND START MEASUREMENTS MSG	****//
	
	//HAL_UART_Receive(&huart4, buffer, 12, 1000);					//****	SENSOR 1: GET FRAME "MSG SENDED"	****//
	//HAL_UART_Receive(&huart6, buffer, 12, 1000);					//****	SENSOR 1: GET FRAME "MSG SENDED"	****//
	
	//HAL_UART_Transmit(&huart5, buffer, 12, 1000);   		// 	DEBUG PRINT -- eventueel nakijken??
	
	//HAL_UART_Receive(&huart4, buffer, 13, 1000); 					//****	SENSOR 1: GET AND CHECK ANWSER	****//
	//HAL_UART_Receive(&huart6, buffer, 13, 1000); 					//****	SENSOR 2: GET AND CHECK ANWSER	****//
	// 	CHECK ANSWER -- EVENTUALLY ABORT IF THERE WAS A SENSOR NOT STARTED
	
	//--------------------------------------------------------//
	
	
	
	//HAL_Delay(1000);

	/*
	uint8_t reading_frame = 0, header_readed = 0;
	uint8_t rsvbuf [200];
	uint16_t len;
	uint8_t rd_len_counter;
	*/

	//******************************//
	//		***		BEGIN LOOP		***		//
	//******************************//
  while (1){	
		BT_read_ringbuffer_pakket(&huart4, &uart4_buffer, &imu_1);
		USB_COM_check_rx_buffer();
	}		
	//*****************************//
	//		***		 END LOOP		***		 //
	//*****************************//
		

		/*
    if(send_1 == 1){
      send_1 = 0;
      serPrintln(&huart5, "\nSENSOR 1: Send_1_1 active");
      convertBuffer(&buf_1[0], 1);
			num_send_1++;
    }
    if(send_1 == 2){
      send_1 = 0;
      serPrintln(&huart5, "\nSENSOR 1: Send_1_2 active");
      convertBuffer(&buf_1[SIZE_PING_PONG_BUFFER], 1);
    }
		
		if(send_2 == 1){
      send_2 = 0;
      serPrintln(&huart5, "\nSENSOR 2: Send_2_1 active");
      convertBuffer(&buf_2[0], 2);
			num_send_2++;
    }
    if(send_2 == 2){
      send_2 = 0;
      serPrintln(&huart5, "\nSENSOR 2: Send_2_2 active");
      convertBuffer(&buf_2[SIZE_PING_PONG_BUFFER], 2);
    }
		
		if(send_3 == 1){
      send_3 = 0;
      serPrintln(&huart5, "\nSENSOR 3: Send_3_1 active");
      convertBuffer(&buf_3[0], 3);
			//num_send_2++;
    }
    if(send_3 == 2){
      send_3 = 0;
      serPrintln(&huart5, "\nSENSOR 3: Send_3_2 active");
      convertBuffer(&buf_3[SIZE_PING_PONG_BUFFER], 3);
    }				
  }
*/

}



// ================================================================
// ===              	BT Pakket Handler Functions               ===
// ================================================================

void BT_read_ringbuffer_pakket(UART_HandleTypeDef *huart, read_from_ringbuffer *buffer, imu_module *imu){
	uint8_t rd_byte;
	if(UART_IsDataAvailable(huart)){			
		if(!buffer->reading_frame) rd_byte = UART_COM_read(huart);
		if(rd_byte == 0x02 && !buffer->reading_frame){
			buffer->reading_frame = 1;
		}
		if(buffer->reading_frame){
			if(!buffer->header_readed){
				buffer->header_readed = 1;
				HAL_Delay(1);
				*(buffer->rsvbuf + 0) = 0x02;
				*(buffer->rsvbuf + 1) = UART_COM_read(huart);
				*(buffer->rsvbuf + 2) = UART_COM_read(huart);
				*(buffer->rsvbuf + 3) = UART_COM_read(huart);
				buffer->len = *(buffer->rsvbuf + 2) | (*(buffer->rsvbuf + 3) << 8);
				
				#ifdef USB_DEBUG
					sprintf(string, "len: %d\n", buffer->len); 
					HAL_UART_Transmit(&huart5, (uint8_t *)string, strlen(string), 25);
				#endif
			}
			else{
				if(buffer->len < 255){
					if(buffer->rd_len_counter < buffer->len){
						*(buffer->rsvbuf + 4 + buffer->rd_len_counter) = UART_COM_read(huart);
						buffer->rd_len_counter++;
					}
					else{
						buffer->reading_frame = 0;
						buffer->header_readed = 0;
						buffer->rd_len_counter = 0;
						
						//	BEGIN: Do something with the readed frame
						
						//USB_COM_print_buffer_hex(buffer->rsvbuf, buffer->len);
						
						#ifdef USB_DEBUG
							sprintf(string, "Command: %02X Len: %02X\n", buffer->rsvbuf [1], buffer->rsvbuf [2]); 
							UART_COM_write(&huart5, (uint8_t *)string, strlen(string));
						#endif
						
						rsv_bt_packet_handler(buffer->rsvbuf, buffer->len, imu);
						
						//	END: Do something with the readed frame
						
						memset(buffer->rsvbuf, 1, buffer->len + 5);
					}
				}
			}
		}
	}
}




void rsv_bt_packet_handler(uint8_t * rsvbuf, uint8_t len, imu_module *imu){
	switch(rsvbuf [1]){
		case CMD_RESET_CNF:{
				//  Reset request received
		}
		break;
		case CMD_DATA_CNF:{
				//  Data transmission request received
		}
		break;

		case CMD_TXCOMPLETE_RSP:{
				//  Data has been sent
		}
		break;

		case CMD_GETSTATE_CNF:{
				//  Current module state
		}
		break;
		
		case CMD_CONNECT_CNF:{
				//	Request received
				if(*(rsvbuf + 4) == 0x00)	USB_COM_print_info(imu->name, "Request received, try to connect");
				else											USB_COM_print_info(imu->name, "Operation failed");
		}
		break;

		case CMD_CONNECT_IND:{
				//  Connection established
				USB_COM_print_info(imu->name, "Connection established");
				imu->connected = 1;
		}
		break;

		case CMD_CHANNELOPEN_RSP:{
				//  Channel open, data transmission possible
				USB_COM_print_info(imu->name, "Channel open, data transmission possible");
		}
		break;

		case CMD_DISCONNECT_CNF:{
				//  Disconnection request received
		}
		break;
		
		case CMD_DISCONNECT_IND:{
				//  Disconnected
				USB_COM_print_info(imu->name, "Disconnected");
				imu->connected = 0;
		}
		break;

		case CMD_SLEEP_CNF:{
				//  Sleep request received
		}
		break;

		case CMD_SET_CNF:{
				//  Module flash settings have been modified
		}
		break;

		case CMD_DATA_IND:{
				//  Data has been received
				rsv_data_msg_handler(rsvbuf, len, imu);
		}
		break;

		case CMD_SCANSTART_CNF:{
				//  Scanning started
		}
		break;

		case CMD_SCANSTOP_CNF:{
				//  Scanning stopped
		}
		break;

		case CMD_BEACON_IND:{
		}
		break;

		default:
				break;
		}
}


void rsv_data_msg_handler(uint8_t * rsvbuf, uint8_t len, imu_module *imu){
	if(len > 50){	//	Nog extra commando toevoegen, wanneer er data binnenkomt !!!!
		convertBuffer(rsvbuf, 1);
	}

	/*
	char string [10];
	for(int i = 0; i < len; i++){
		sprintf(string, "%02X ", *(rsvbuf + i)); 
		HAL_UART_Transmit(&huart5, (uint8_t *)string, strlen(string), 25);
	}
	sprintf(string, "\n"); 
	HAL_UART_Transmit(&huart5, (uint8_t *)string, strlen(string), 25);
	*/

	switch(*(rsvbuf + 11)){

		case IMU_SENSOR_MODULE_IND_BATTERY_VOLTAGE:{
			float voltage = (*(rsvbuf + 12) | *(rsvbuf + 13) << 8)/100.0;
			
			char string [50];
			sprintf(string, "Battery voltage: %.02f V\n", voltage); 
			UART_COM_write(&huart5, (uint8_t *)string, strlen(string));
		}
		
		case IMU_SENSOR_MODULE_IND_SYNC_DONE:{
			IMU_send_adv_msg(imu);	
		}
		
		case IMU_SENSOR_MODULE_IND_SLEEP_MODE:{
			USB_COM_print_info(imu->name, "goes sleeping ...");
			imu->connected = 0;
		}
		


		default:{
		}
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == USER_BUTTON_Pin){ //GPIO_PIN_2
    HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
		IMU_go_to_sleep(&imu_1);
		//IMU_go_to_sleep(&imu_2);
		//IMU_go_to_sleep(&imu_3);
		//IMU_go_to_sleep(&imu_4);
		//IMU_go_to_sleep(&imu_5);
		//IMU_go_to_sleep(&imu_6);
  }
}




void SDCard_create_new_file(void){
  char string [100];
  FRESULT res;
  while(1){
    char path [25];
    sprintf(path, "MET_%d.TXT", file_nummer);
    res = f_open(&myFILE, path, FA_WRITE);  
    f_close(&myFILE);
    if(res == FR_OK){
      //sprintf(string, "File \"MET_%d.TXT\" bestaat\n", file_nummer); 
      //UART_COM_write(&huart5, (uint8_t *)string, strlen(string));
      file_nummer++;
    }
    else{
      sprintf(string, "File \"MET_%d.TXT\" bestaat NIET\n", file_nummer); 
      UART_COM_write(&huart5, (uint8_t *)string, strlen(string));
      break;
    }
  }
  
  sprintf(string, "Nieuw file nummer: %d \n", file_nummer); 
  UART_COM_write(&huart5, (uint8_t *)string, strlen(string));

  char path [25];
  sprintf(path, "MET_%d.TXT", file_nummer);
  HAL_UART_Transmit(&huart5, (uint8_t *)path, strlen(path), 25);
  res = f_open(&myFILE, path, FA_CREATE_NEW);
  f_close(&myFILE);
  if(res == FR_OK){
		USB_COM_print_ln(" created succesfully"); 
  }
  else{
    USB_COM_print_ln(" NOT created"); 
  }
}   



FRESULT SDCard_mount(void){
  char string [100];
  FRESULT code;
  code = f_mount(&myFATAFS, SDPath, 1);
  if(code == FR_OK){
    sprintf(string, "Mounten van SD kaart succesvol\n"); 
    UART_COM_write(&huart5, (uint8_t *)string, strlen(string));
  }
  else{
    sprintf(string, "Probleem bij het mounten van de SD kaart\n"); 
    UART_COM_write(&huart5, (uint8_t *)string, strlen(string));
  }
  return code;
}





void printSystemTick(void){
  char buffer[10];
  uint32_t tick = HAL_GetTick();
  sprintf(buffer, "%d : ", tick);
  UART_COM_write(&huart5, (uint8_t *)buffer, sizeof(buffer));
}








#define SEND_DATA_VISUAL
//#define SEND_DATA_YPR
//#define SEND_DATA_QUATERNIONS
#define SAVE_SD_CARD

void convertBuffer(uint8_t * buf, uint8_t sensor_number){
	
	uint32_t systemtick = HAL_GetTick();
  
  #ifdef SAVE_SD_CARD
    uint16_t sd_card_buffer [20][3];
  #endif
  
  
  
  
  for(int j = 0; j < NUMBER_OF_BT_PACKETS; j++){
    uint16_t start_pos = j * SIZE_BT_PACKET + PACKET_START_POS;
    for(int i = 0; i < NUMBER_OF_DATA_READS_IN_BT_PACKET; i++){
      int16_t data [4];
      data[0] = ((buf[start_pos + 0 + 8 * i] << 8) | buf[start_pos + 1 + 8 * i]);
      data[1] = ((buf[start_pos + 2 + 8 * i] << 8) | buf[start_pos + 3 + 8 * i]);
      data[2] = ((buf[start_pos + 4 + 8 * i] << 8) | buf[start_pos + 5 + 8 * i]);
      data[3] = ((buf[start_pos + 6 + 8 * i] << 8) | buf[start_pos + 7 + 8 * i]);

      //uint8_t tx_send [8] = {data[0] >> 8, (uint8_t)data[0], data[1] >> 8, (uint8_t)data[1], data[2] >> 8, (uint8_t)data[2], data[3] >> 8, (uint8_t)data[3]};
      //HAL_UART_Transmit(&huart3, tx_send, 8, 25);
      
      float buf_YPR [3];
      getYawPitchRoll(data, buf_YPR);
      
      
      //    ***   Option 1: Visual    ***   //
      #ifdef SEND_DATA_VISUAL
        char string [100];
        //sprintf(string, "Data 0: %.2i | Data 1: %.2i | Data 2: %.2i | Data 3: %.2i\n", data[0], data[1], data[2], data[3]); 
        //sprintf(string, "Yaw: %.2f | Pitch: %.2f | Roll: %.2f\n", (buf_YPR[0]/PI*180+180), (buf_YPR[1]/PI*180+180), (buf_YPR[2]/PI*180+180)); 
        //sprintf(string, "Yaw: %i | Pitch: %i | Roll: %i\n", (uint16_t)(buf_YPR[0]/PI*180+180), (uint16_t)(buf_YPR[1]/PI*180+180), (uint16_t)(buf_YPR[2]/PI*180+180)); 
				sprintf(string, "Yaw: %i | Pitch: %i | Roll: %i, %i, %i, %i\n", (uint16_t)(buf_YPR[0]/PI*180+180), (uint16_t)(buf_YPR[1]/PI*180+180), (uint16_t)(buf_YPR[2]/PI*180+180), num_send_1, num_send_2, systemtick); 
				//sprintf(string, "Timestamp: %i - Yaw: %i | Pitch: %i | Roll: %i - %i\n", systemtick, (uint16_t)(buf_YPR[0]/PI*180+180), (uint16_t)(buf_YPR[1]/PI*180+180), (uint16_t)(buf_YPR[2]/PI*180+180), num_send_1); 
				UART_COM_write(&huart5, (uint8_t *)string, strlen(string));
      #endif
      //    ***   Option 2: Pycharm frame YPR   ***   //
      #ifdef SEND_DATA_YPR
        uint16_t data_hex_16 [3] = {(uint16_t)(buf_YPR[0]/PI*180+180), (uint16_t)(buf_YPR[1]/PI*180+180), (uint16_t)(buf_YPR[2]/PI*180+180)};
        uint8_t data_hex_buf [10];
        data_hex_buf [0] = 0x02;
        data_hex_buf [1] = 0x0A;
        data_hex_buf [2] = 0x00;
        data_hex_buf [3] = (uint8_t)(data_hex_16 [0]);
        data_hex_buf [4] = (uint8_t)(data_hex_16 [0] >> 8);
        data_hex_buf [5] = (uint8_t)(data_hex_16 [1]);
        data_hex_buf [6] = (uint8_t)(data_hex_16 [1] >> 8);
        data_hex_buf [7] = (uint8_t)(data_hex_16 [2]);
        data_hex_buf [8] = (uint8_t)(data_hex_16 [2] >> 8);
        data_hex_buf [9] = calculateCS(data_hex_buf, 9);
        
        HAL_UART_Transmit(&huart5, data_hex_buf, sizeof(data_hex_buf), 25);
      #endif
      
      //    ***   Option 3: Pycharm frame QUATERNIONS   ***   //      
      #ifdef SEND_DATA_QUATERNIONS
        uint8_t data_hex_buf [12];
        data_hex_buf [0]  = 0x02;
        data_hex_buf [1]  = 0x0A;
        data_hex_buf [2]  = 0x00;
        data_hex_buf [3]  = (uint8_t)(data [0]);
        data_hex_buf [4]  = (uint8_t)(data [0] >> 8);
        data_hex_buf [5]  = (uint8_t)(data [1]);
        data_hex_buf [6]  = (uint8_t)(data [1] >> 8);
        data_hex_buf [7]  = (uint8_t)(data [2]);
        data_hex_buf [8]  = (uint8_t)(data [2] >> 8);
        data_hex_buf [9]  = (uint8_t)(data [3]);
        data_hex_buf [10] = (uint8_t)(data [3] >> 8);
        data_hex_buf [11] = calculateCS(data_hex_buf, 9);
        
        HAL_UART_Transmit(&huart5, data_hex_buf, sizeof(data_hex_buf), 25);
      #endif
      
      //    ***   Option 4: Save YPR on SD card   ***   //      
      #ifdef SAVE_SD_CARD
        sd_card_buffer [j*NUMBER_OF_DATA_READS_IN_BT_PACKET + i][0] = (uint16_t)(buf_YPR[0]/PI*180+180);
        sd_card_buffer [j*NUMBER_OF_DATA_READS_IN_BT_PACKET + i][1] = (uint16_t)(buf_YPR[1]/PI*180+180);
        sd_card_buffer [j*NUMBER_OF_DATA_READS_IN_BT_PACKET + i][2] = (uint16_t)(buf_YPR[2]/PI*180+180);        
      #endif
    }
  }
  
  
  
  //    ***   Option 4: Save YPR on SD card   ***   //      
  #ifdef SAVE_SD_CARD
    HAL_GPIO_TogglePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin);
    char path [25];
    sprintf(path, "MET_%d.TXT", file_nummer);
    f_open(&myFILE, path, FA_OPEN_APPEND | FA_WRITE); 
    for(int i = 0; i < SIZE_SD_CARD_READ_BUF; i++){
      f_printf(&myFILE, "%d,%d,%d,%d,%d\n", systemtick, sensor_number, (uint16_t)(sd_card_buffer[i][0]), (uint16_t)(sd_card_buffer[i][1]), (uint16_t)(sd_card_buffer[i][2]));
    }
    f_close(&myFILE);
    HAL_GPIO_TogglePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin);   
  #endif
  
}




void getYawPitchRoll(int16_t *data, float *newdata){
    //  q = quaternion
  float q [4];
  q[0] = (float)data[0] / 16384.0f;   //  w
  q[1] = (float)data[1] / 16384.0f;   //  x
  q[2] = (float)data[2] / 16384.0f;   //  y
  q[3] = (float)data[3] / 16384.0f;   //  z

  // gravity
  float gravity [3];
  gravity[0] = 2 * (q[1] * q[3] - q[0] * q[2]);                         //  x
  gravity[1] = 2 * (q[0] * q[1] + q[2] * q[3]);                         //  y
  gravity[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];   //  z

  //  Euler
  //float euler [3];
  //euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);     // psi
  //euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]);                                            // theta
  //euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1);     // phi
  
  
  /*
    // calculate gravity vector
  gravity[0] = 2 * (q[1]*q[3] - q[0]*q[2]);
  gravity[1] = 2 * (q[0]*q[1] + q[2]*q[3]);
  gravity[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

  // calculate Euler angles
  euler[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
  euler[1] = -asin(2*q[1]*q[3] + 2*q[0]*q[2]);
  euler[2] = atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);

  // calculate yaw/pitch/roll angles
  ypr[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
  ypr[1] = atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
  ypr[2] = atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));
  
  
  */
  
  // yaw: (about Z axis)
  newdata[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);
  // pitch: (nose up/down, about Y axis)
  newdata[1] = atan2(gravity[0] , sqrt(gravity[1] * gravity[1] + gravity[2] * gravity[2]));
  // roll: (tilt left/right, about X axis)
  newdata[2] = atan2(gravity[1], gravity[2]);
  
  if (gravity[2] < 0) {
    if(newdata[1] > 0)    newdata[1] = PI   - newdata[1]; 
    else                  newdata[1] = -PI  - newdata[1];
  }
  
}









/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 16;
  hsd1.Init.TranceiverPresent = SDMMC_TRANSCEIVER_NOT_PRESENT;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}






void startUpLeds(void){
  IOExpander_init(&hi2c1, I2C_ADDRESS_IC5);
  
  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  for(uint8_t i = 0; i < 6; i++){ 
    IOExpander_set(&hi2c1, I2C_ADDRESS_IC5, i);
    HAL_Delay(100);
  }
  IOExpander_clearAll(&hi2c1, I2C_ADDRESS_IC5);
  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
}



/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  HAL_GPIO_TogglePin(LED_GOOD_GPIO_Port, LED_GOOD_Pin);
	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
	
	
	//char string [100];
	//sprintf(string, "num_send_1: %i | num_send_2: %i \n", num_send_1, num_send_2);
	//HAL_UART_Transmit(&huart5, (uint8_t *)string, strlen(string), 25);
	
  //char string [100];
  //sprintf(string, "Busy\n"); 
  //HAL_UART_Transmit_IT(&huart5, (uint8_t *)string, strlen(string));
}

/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/





void IOExpander_init(I2C_HandleTypeDef *hi2c, uint8_t address){
  uint8_t send_buf [] = {CMD_REG_CONFIG, 0x00};
  HAL_I2C_Master_Transmit(hi2c, address, send_buf, sizeof(send_buf), 25);
}

void IOExpander_set(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t io){
  gpio_buf_IC5 |= 1 << io;
  IOExpander_update(hi2c, address, gpio_buf_IC5);
}

void IOExpander_clear(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t io){
  gpio_buf_IC5 &= 0 << io;
  IOExpander_update(hi2c, address, gpio_buf_IC5);
}

void IOExpander_clearAll(I2C_HandleTypeDef *hi2c, uint8_t address){
  gpio_buf_IC5 = 0x00;
  IOExpander_update(hi2c, address, gpio_buf_IC5);
}

void IOExpander_update(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t buf){
  uint8_t send_buf [] = {CMD_REG_OUTPUT, buf};
  HAL_I2C_Master_Transmit(hi2c, address, send_buf, sizeof(send_buf), 25);
}










/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
