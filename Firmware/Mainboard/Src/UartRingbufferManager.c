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
 *         File: UartRingBufferManager.c
 *      Created: 2020-02-27
 *       Author: Jarne Van Mulders
 *      Version: V1.0
 *
 *  Description: Firmware IMU sensor module for the NOMADe project
 *
 *  Interreg France-Wallonie-Vlaanderen NOMADe
 *
 */
#include "UartRingbufferManager.h"
#include <string.h>


extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

ring_buffer huart1_rx_buffer = { { 0 }, 0, 0};
ring_buffer huart1_tx_buffer = { { 0 }, 0, 0};
ring_buffer huart2_rx_buffer = { { 0 }, 0, 0};
ring_buffer huart2_tx_buffer = { { 0 }, 0, 0};
ring_buffer huart3_rx_buffer = { { 0 }, 0, 0};
ring_buffer huart3_tx_buffer = { { 0 }, 0, 0};
ring_buffer huart4_rx_buffer = { { 0 }, 0, 0};
ring_buffer huart4_tx_buffer = { { 0 }, 0, 0};
ring_buffer huart5_rx_buffer = { { 0 }, 0, 0};
ring_buffer huart5_tx_buffer = { { 0 }, 0, 0};
ring_buffer huart6_rx_buffer = { { 0 }, 0, 0};
ring_buffer huart6_tx_buffer = { { 0 }, 0, 0};
ring_buffer huart7_rx_buffer = { { 0 }, 0, 0};
ring_buffer huart7_tx_buffer = { { 0 }, 0, 0};
ring_buffer huart8_rx_buffer = { { 0 }, 0, 0};
ring_buffer huart8_tx_buffer = { { 0 }, 0, 0};


void UART_Ringbuffer_Init(UART_HandleTypeDef *huart){
	RB_Init(huart);
}

uint8_t UART_Read(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		return RB_Uart_read(&huart1_rx_buffer);
	}
	if(huart->Instance == USART2){
		return RB_Uart_read(&huart2_rx_buffer);
	}
	if(huart->Instance == USART3){
		return RB_Uart_read(&huart3_rx_buffer);
	}
	if(huart->Instance == USART6){
		return RB_Uart_read(&huart6_rx_buffer);
	}
	if(huart->Instance == UART4){
		return RB_Uart_read(&huart4_rx_buffer);
	}
	if(huart->Instance == UART5){
		return RB_Uart_read(&huart5_rx_buffer);
	}
	if(huart->Instance == UART7){
		return RB_Uart_read(&huart7_rx_buffer);
	}
	if(huart->Instance == UART8){
		return RB_Uart_read(&huart8_rx_buffer);
	}
	return 0;
}


void UART_Write(UART_HandleTypeDef *huart, uint8_t byte){
	if(huart->Instance == USART1){
		RB_Uart_write(huart, &huart1_tx_buffer, byte);
	}
	if(huart->Instance == USART2){
		RB_Uart_write(huart, &huart2_tx_buffer, byte);
	}
	if(huart->Instance == USART3){
		RB_Uart_write(huart, &huart3_tx_buffer, byte);
	}
	if(huart->Instance == USART6){
		RB_Uart_write(huart, &huart6_tx_buffer, byte);
	}
	if(huart->Instance == UART4){
		RB_Uart_write(huart, &huart4_tx_buffer, byte);
	}
	if(huart->Instance == UART5){
		RB_Uart_write(huart, &huart5_tx_buffer, byte);
	}
	if(huart->Instance == UART7){
		RB_Uart_write(huart, &huart7_tx_buffer, byte);
	}
	if(huart->Instance == UART8){
		RB_Uart_write(huart, &huart8_tx_buffer, byte);
	}
}


uint16_t UART_IsDataAvailable(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		return RB_IsDataAvailable(&huart1_rx_buffer);
	}
	if(huart->Instance == USART2){
		return RB_IsDataAvailable(&huart2_rx_buffer);
	}
	if(huart->Instance == USART3){
		return RB_IsDataAvailable(&huart3_rx_buffer);
	}
	if(huart->Instance == USART6){
		return RB_IsDataAvailable(&huart6_rx_buffer);
	}
	if(huart->Instance == UART4){
		return RB_IsDataAvailable(&huart4_rx_buffer);
	}
	if(huart->Instance == UART5){
		return RB_IsDataAvailable(&huart5_rx_buffer);
	}
	if(huart->Instance == UART7){
		return RB_IsDataAvailable(&huart7_rx_buffer);
	}
	if(huart->Instance == UART8){
		return RB_IsDataAvailable(&huart8_rx_buffer);
	}
	return 0;
}



void UART_ISR_Manager(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		RB_UART_ISR(huart, &huart1_rx_buffer, &huart1_tx_buffer);
	}
	if(huart->Instance == USART2){
		RB_UART_ISR(huart, &huart2_rx_buffer, &huart2_tx_buffer);
	}
	if(huart->Instance == USART3){
		RB_UART_ISR(huart, &huart3_rx_buffer, &huart3_tx_buffer);
	}
	if(huart->Instance == USART6){
		RB_UART_ISR(huart, &huart6_rx_buffer, &huart6_tx_buffer);
	}
	if(huart->Instance == UART4){
		RB_UART_ISR(huart, &huart4_rx_buffer, &huart4_tx_buffer);
	}
	if(huart->Instance == UART5){
		RB_UART_ISR(huart, &huart5_rx_buffer, &huart5_tx_buffer);
	}
	if(huart->Instance == UART7){
		RB_UART_ISR(huart, &huart7_rx_buffer, &huart7_tx_buffer);
	}
	if(huart->Instance == UART8){
		RB_UART_ISR(huart, &huart8_rx_buffer, &huart8_tx_buffer);
	}
}





