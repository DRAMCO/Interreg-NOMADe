
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
#include <string.h>
#include <stdio.h>
#include "uart_lib.h"
#include "config.h"
#include "ble_lib.h"
#include "def.h"

float PI = 3.14159265358979323846;

static void convertBuffer(uint8_t * buf);
static void getYawPitchRoll(int16_t *data, float *newdata);

static void MX_I2C1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM2_Init(void);

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim2;


UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

uint8_t buf_1_pack_len, buf_2_pack_len, buf_3_pack_len, buf_4_pack_len, buf_5_pack_len, buf_6_pack_len;

uint8_t buf_1 [SIZE_PING_PONG_BUFFER * 2];
uint8_t buf_2 [SIZE_PING_PONG_BUFFER * 2];
uint8_t buf_3 [SIZE_PING_PONG_BUFFER * 2];
uint8_t buf_4 [SIZE_PING_PONG_BUFFER * 2];
uint8_t buf_5 [SIZE_PING_PONG_BUFFER * 2];
uint8_t buf_6 [SIZE_PING_PONG_BUFFER * 2];

uint8_t send_1;


//uint8_t cmd_uart_control_buf [10];


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* 	Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* 	Configure the system clock */
  SystemClock_Config();

  /* 	Initialize all configured peripherals */
  MX_GPIO_Init();
	
  MX_UART4_Init(BT_BAUDRATE);
  MX_UART5_Init(BT_BAUDRATE); //COM_BAUDRATE
  MX_UART7_Init(BT_BAUDRATE);
  MX_UART8_Init(BT_BAUDRATE);
  MX_USART1_UART_Init(BT_BAUDRATE);
  MX_USART2_UART_Init(BT_BAUDRATE);
  MX_USART3_UART_Init(COM_BAUDRATE);
	MX_USART6_UART_Init(BT_BAUDRATE);
	
	MX_I2C1_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
	
	char string [100];
	sprintf(string, "START"); 
	HAL_UART_Transmit(&huart3, (uint8_t *)string, strlen(string), 25);
	
	
	/*	Connect with all the Bluetooth modules	*/
	SENS_connect(2);
	
	
	HAL_UART_Receive_IT(&huart5, &(buf_1[0]), 92);
	
	//HAL_UART_Receive_IT(&huart3, cmd_uart_control_buf, 1);


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//HAL_GPIO_TogglePin(LED_BUSY_GPIO_Port, LED_BUSY_Pin);
		//HAL_Delay(1000);
		
		
		if(send_1 == 1){
			send_1 = 0;
			convertBuffer(buf_1);
			//HAL_UART_Transmit_IT(&huart3, buf_1, SIZE_PING_PONG_BUFFER);
		}
		if(send_1 == 2){
			send_1 = 0;
			convertBuffer(&buf_1[SIZE_PING_PONG_BUFFER]);
			//HAL_UART_Transmit_IT(&huart3, &buf_1[SIZE_PING_PONG_BUFFER], SIZE_PING_PONG_BUFFER);
		}
		
		
		
  }
  /* USER CODE END 3 */
}

void convertBuffer(uint8_t * buf){
	
	for(int j = 0; j < NUMBER_OF_BT_PACKETS; j++){
		uint16_t start_pos = j * SIZE_BT_PACKET + PACKET_START_POS;
		for(int i = 0; i < NUMBER_OF_DATA_READS_IN_BT_PACKET; i++){
			int16_t data [4];
			data[0] = ((buf[start_pos + 0 + 8 * i] << 8) | buf[start_pos + 1 + 8 * i]);
			data[1] = ((buf[start_pos + 2 + 8 * i] << 8) | buf[start_pos + 3 + 8 * i]);
			data[2] = ((buf[start_pos + 4 + 8 * i] << 8) | buf[start_pos + 5 + 8 * i]);
			data[3] = ((buf[start_pos + 6 + 8 * i] << 8) | buf[start_pos + 7 + 8 * i]);

			//uint8_t tx_send [8] = {data[0] >> 8, (uint8_t)data[0], data[1] >> 8, (uint8_t)data[1], data[2] >> 8, (uint8_t)data[2], data[3] >> 8, (uint8_t)data[3]};
			//HAL_UART_Transmit(&huart3, tx_send, 8, 25);²
			
			float buf_YPR [3];
			getYawPitchRoll(data, buf_YPR);
			
			char string [100];
			//sprintf(string, "Data 0: %.2i | Data 1: %.2i | Data 2: %.2i | Data 3: %.2i\n", data[0], data[1], data[2], data[3]); 
			sprintf(string, "Yaw: %.2f | Pitch: %.2f | Roll: %.2f\n", (buf_YPR[0]/PI*180), (buf_YPR[1]/PI*180), (buf_YPR[2]/PI*180)); 
			HAL_UART_Transmit(&huart3, (uint8_t *)string, strlen(string), 25);
			
			
		}
	}
	
	
	
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
	gravity[0] = 2 * (q[1] * q[3] - q[0] * q[2]);													//	x
	gravity[1] = 2 * (q[0] * q[1] + q[2] * q[3]);													//	y
	gravity[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];		//	z

	//	Euler
	//float euler [3];
	//euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);     // psi
	//euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]);                                            // theta
	//euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1);     // phi
	
	// yaw: (about Z axis)
	newdata[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1);
	// pitch: (nose up/down, about Y axis)
	newdata[1] = atan2(gravity[0] , sqrt(gravity[1] * gravity[1] + gravity[2] * gravity[2]));
	// roll: (tilt left/right, about X axis)
	newdata[2] = atan2(gravity[1], gravity[2]);
	
	if (gravity[2] < 0) {
		if(newdata[1] > 0)		newdata[1] = PI 	- newdata[1]; 
		else 									newdata[1] = -PI 	- newdata[1];
	}
	
}



/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/

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
  hi2c1.Init.Timing = 0x10C0ECFF;
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
  hsd1.Init.ClockDiv = 0;
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
  htim2.Init.Prescaler = 39999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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


/********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/










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
