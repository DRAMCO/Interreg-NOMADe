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
 *         File: usb_com.c
 *      Created: 2020-08-17
 *       Author: Jarne Van Mulders
 *      Version: V1.0
 *
 *  Description: Firmware IMU sensor module for the NOMADe project
 *
 *  Interreg France-Wallonie-Vlaanderen NOMADe
 *
 */
#include "sd_card_com.h"


extern FATFS myFATAFS;
extern FIL myFILE;
extern UINT testByte;

uint16_t file_nummer = 0;

uint8_t sd_card_ready = 0;

SD_HandleTypeDef hsd1;


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


FRESULT SD_CARD_COM_mount(void){
	MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
	FATFS_UnLinkDriver(SDPath);
	FATFS_LinkDriver(&SD_Driver, SDPath);
  FRESULT code;
  code = f_mount(&myFATAFS, SDPath, 1);
  if(code == FR_OK)	{	sd_card_ready = 1;	USB_COM_print_ln("SD card mounted successfully");							}
  else							{	
		sd_card_ready = 0;	
		if(SD_CARD_available()) USB_COM_print_ln("SD card mounting problem");
		else 										USB_COM_print_ln("Insert SD card");
	}
	//SD_CARD_COM_print_status_code(code);
  return code;
}

FRESULT SD_CARD_COM_unmount(void){
  FRESULT code;
  code = f_mount(0, SDPath, 0);
  if(code == FR_OK)	{ USB_COM_print_ln("SD card unmounted successfully");							}
  else							{	USB_COM_print_ln("SD card unmounting problem"); 	}
	//SD_CARD_COM_print_status_code(code);
	sd_card_ready = 0;
  return code;
}



void SD_CARD_COM_create_new_file(void){
	if(sd_card_ready){
		FRESULT res;

		file_nummer = SD_CARD_COM_check_existing_files();

		char path [25];
		sprintf(path, "MES_%d.TXT", file_nummer);
		HAL_UART_Transmit(&huart5, (uint8_t *)path, strlen(path), 25);
		res = SD_CARD_COM_open_close(&myFILE, path, FA_CREATE_NEW);
		if(res == FR_OK){
			USB_COM_print_ln(" created succesfully"); 
		}
		else{
			USB_COM_print_ln(" NOT created"); 
		}
	}
	else
		USB_COM_print_ln("Insert and mount SD card first");	
}   



uint16_t SD_CARD_COM_check_existing_files(void){
	
	uint16_t number = 0;
	FRESULT res;
	
	while(1){
    char path [25];
    sprintf(path, "MES_%d.TXT", number);
    res = SD_CARD_COM_open_close(&myFILE, path, FA_WRITE);  
    if(res == FR_OK){
      number++;
    }
    else{
      //USB_COM_print_value_ln("File \"MET_%d.TXT\" bestaat NIET\n", number);
      break;
    }
  }
  
  //USB_COM_print_value_ln("Nieuw file nummer: %d \n", number);
	return number;
}



void SD_CARD_COM_open_file(void){
	char path [25];
  sprintf(path, "MES_%d.TXT", file_nummer);
	f_open(&myFILE, path, FA_OPEN_APPEND | FA_WRITE);
}

void SD_CARD_COM_close_file(void){
	f_close(&myFILE);
}

/*
void SD_CARD_COM_save_data(uint16_t number, uint32_t systemtick, uint8_t sensor_number, uint8_t *sd_card_buffer){
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
	for(uint8_t i = 0; i < SIZE_SD_CARD_READ_BUF; i++){
		uint16_t new_data [3];
		for(uint8_t j = 0; j < 3; j++){
			new_data [j] = (uint16_t)(*(sd_card_buffer + i*6 + j*2) | *(sd_card_buffer + i*6 + j*2 + 1) << 8);
		}
		f_printf(&myFILE, "%d,%d,%d,%d,%d,%d\n", sensor_number, number, systemtick, new_data [0], new_data [1], new_data [2]);
	}
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_RESET);
}
*/


void SD_CARD_COM_save_data(uint16_t number, uint32_t systemtick, uint8_t sensor_number, int16_t *sd_card_buffer, uint8_t dv, uint8_t df){
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
	
	for(uint8_t i = 0; i < SIZE_SD_CARD_READ_BUF; i++){
		int16_t new_data [dv];
		for(uint8_t j = 0; j < dv; j++){
			new_data [j] = (int16_t)(*(sd_card_buffer + i*dv + j));
		}
		switch(df){
			case DATA_FORMAT_1: f_printf(&myFILE, "%d,%d,%d,%d,%d,%d,%d\n", sensor_number, number, systemtick, new_data [0], new_data [1], new_data [2], new_data [3]); break;
			case DATA_FORMAT_2: f_printf(&myFILE, "%d,%d,%d,%d,%d,%d\n", sensor_number, number, systemtick, new_data [0], new_data [1], new_data [2]); break;
			case DATA_FORMAT_3: f_printf(&myFILE, "%d,%d,%d,%d,%d,%d\n", sensor_number, number, systemtick, new_data [0], new_data [1], new_data [2]); break;
			case DATA_FORMAT_4: f_printf(&myFILE, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n", sensor_number, number, systemtick, new_data [0], new_data [1], new_data [2], new_data [3], new_data [4], new_data [5]); break;
			case DATA_FORMAT_5: f_printf(&myFILE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", sensor_number, number, systemtick, new_data [0], new_data [1], new_data [2], new_data [3], new_data [4], new_data [5], new_data [6], new_data [7], new_data [8], new_data [9]); break;
		}
	}
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_RESET);
}



void SD_CARD_COM_save_data_q(uint16_t number, uint32_t systemtick, uint8_t sensor_number, int16_t *sd_card_buffer){
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
	for(uint8_t i = 0; i < SIZE_SD_CARD_READ_BUF; i++){
		int16_t new_data [4];
		for(uint8_t j = 0; j < 4; j++){
			new_data [j] = (int16_t)(*(sd_card_buffer + i*4 + j));
		}
		f_printf(&myFILE, "%d,%d,%d,%d,%d,%d,%d\n", sensor_number, number, systemtick, new_data [0], new_data [1], new_data [2], new_data [3]);
	}
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_RESET);
}

void SD_CARD_COM_save_data_qga(uint16_t number, uint32_t systemtick, uint8_t sensor_number, int16_t *sd_card_buffer){
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
	for(uint8_t i = 0; i < SIZE_SD_CARD_READ_BUF; i++){
		int16_t new_data [10];
		for(uint8_t j = 0; j < 10; j++){
			new_data [j] = (int16_t)(*(sd_card_buffer + i*10 + j));
		}
		f_printf(&myFILE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", sensor_number, number, systemtick, new_data [0], new_data [1], new_data [2], new_data [3], new_data [4], new_data [5], new_data [6], new_data [7], new_data [8], new_data [9]);
	}
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_RESET);
}



FRESULT SD_CARD_COM_open_close(FIL* fp, const TCHAR* p, BYTE mode){
	FRESULT res;
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
	res = f_open(fp, p, mode);
  f_close(fp);
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_RESET);
	return res;
}

void SD_CARD_COM_set_status(uint8_t status){
	sd_card_ready = status;
}

uint8_t SD_CARD_COM_get_status(void){
	return sd_card_ready;
}

uint8_t SD_CARD_available(void){
	if(HAL_GPIO_ReadPin(SD_DETECT_GPIO_Port, SD_DETECT_Pin) == GPIO_PIN_SET) return 0;
	else return 1;
}



void SD_CARD_COM_print_status_code(FRESULT code){
	char *str = "SD Card: ";
	switch(code){
					case FR_OK: 									USB_COM_print_info(str, "Succeeded");
	 break; case FR_DISK_ERR:							USB_COM_print_info(str, "A hard error occurred in the low level disk I/O layer");
	 break; case FR_INT_ERR:							USB_COM_print_info(str, "Assertion failed");
	 break; case FR_NOT_READY:						USB_COM_print_info(str, "he physical drive cannot work");
	 break; case FR_NO_FILE:							USB_COM_print_info(str, "Could not find the file");
	 break; case FR_NO_PATH:							USB_COM_print_info(str, "Could not find the path");
	 break; case FR_INVALID_NAME:					USB_COM_print_info(str, "The path name format is invalid");
	 break; case FR_DENIED:								USB_COM_print_info(str, "Access denied due to prohibited access or directory full");
	 break; case FR_EXIST:								USB_COM_print_info(str, "Access denied due to prohibited access");
	 break; case FR_INVALID_OBJECT:				USB_COM_print_info(str, "The file/directory object is invalid");
	 break; case FR_WRITE_PROTECTED:			USB_COM_print_info(str, "The physical drive is write protected");
	 break; case FR_INVALID_DRIVE:				USB_COM_print_info(str, "The logical drive number is invalid");
	 break; case FR_NOT_ENABLED:					USB_COM_print_info(str, "The volume has no work area");
	 break; case FR_NO_FILESYSTEM:				USB_COM_print_info(str, "There is no valid FAT volume");
	 break; case FR_MKFS_ABORTED:					USB_COM_print_info(str, "The f_mkfs() aborted due to any problem");
	 break; case FR_TIMEOUT:							USB_COM_print_info(str, "Could not get a grant to access the volume within defined period");
	 break; case FR_LOCKED:								USB_COM_print_info(str, "The operation is rejected according to the file sharing policy");
	 break; case FR_NOT_ENOUGH_CORE:			USB_COM_print_info(str, "LFN working buffer could not be allocated");
	 break; case FR_TOO_MANY_OPEN_FILES:	USB_COM_print_info(str, "Number of open files > _FS_LOCK");
	 break; case FR_INVALID_PARAMETER:		USB_COM_print_info(str, "Given parameter is invalid");
	 break; 
	}
}




