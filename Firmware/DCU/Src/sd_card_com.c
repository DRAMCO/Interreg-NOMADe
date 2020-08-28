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


FRESULT SD_CARD_COM_mount(void){
  FRESULT code;
  code = f_mount(&myFATAFS, SDPath, 1);
  if(code == FR_OK)	USB_COM_print_ln("Mounten van SD kaart succesvol");
  else							USB_COM_print_ln("Probleem bij het mounten van de SD kaart"); 
	USB_COM_print_value_ln("Code %d \n", code);
  return code;
}



void SD_CARD_COM_create_new_file(void){
	
  FRESULT res;
	
	file_nummer = SD_CARD_COM_check_existing_files();

  char path [25];
  sprintf(path, "MET_%d.TXT", file_nummer);
  HAL_UART_Transmit(&huart5, (uint8_t *)path, strlen(path), 25);
  res = SD_CARD_COM_open_close(&myFILE, path, FA_CREATE_NEW);
  if(res == FR_OK){
		USB_COM_print_ln(" created succesfully"); 
  }
  else{
    USB_COM_print_ln(" NOT created"); 
  }
}   



uint16_t SD_CARD_COM_check_existing_files(void){
	
	uint16_t number = 0;
	FRESULT res;
	
	while(1){
    char path [25];
    sprintf(path, "MET_%d.TXT", number);
    res = SD_CARD_COM_open_close(&myFILE, path, FA_WRITE);  
    if(res == FR_OK){
      number++;
    }
    else{
      USB_COM_print_value_ln("File \"MET_%d.TXT\" bestaat NIET\n", number);
      break;
    }
  }
  
  USB_COM_print_value_ln("Nieuw file nummer: %d \n", number);
	return number;
}



void SD_CARD_COM_open_file(void){
	char path [25];
  sprintf(path, "MET_%d.TXT", file_nummer);
	f_open(&myFILE, path, FA_OPEN_APPEND | FA_WRITE);
}

void SD_CARD_COM_close_file(void){
	f_close(&myFILE);
}

void SD_CARD_COM_save_data(uint32_t systemtick, uint8_t sensor_number, uint8_t *sd_card_buffer){
	HAL_GPIO_WritePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin, GPIO_PIN_SET);
	for(int i = 0; i < SIZE_SD_CARD_READ_BUF; i++){
		uint16_t new_data [3];
		for(uint8_t j = 0; j < 3; j++){
			new_data [j] = (uint16_t)(*(sd_card_buffer + i*6 + j*2) | *(sd_card_buffer + i*6 + j*2 + 1) << 8);
		}
		f_printf(&myFILE, "%d,%d,%d,%d,%d\n", systemtick, sensor_number, new_data [0], new_data [1], new_data [2]);
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








