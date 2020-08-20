/*
 * UartRingbuffer.h
 *
 *  Created on: 10-Jul-2019
 *      Author: Controllerstech
 */

#ifndef UARTRINGBUFFER_H_
#define UARTRINGBUFFER_H_

#include "stm32h7xx_hal.h"

/* change the size of the buffer */
#define UART_BUFFER_SIZE 2000


typedef struct
{
  unsigned char buffer[UART_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
} ring_buffer;


/* Initialize the ring buffer */
void RB_Init(UART_HandleTypeDef *huart);
/*-------------------------------------------------------------------------------------------------*/

/* reads the data in the rx_buffer and increment the tail count in rx_buffer */
uint8_t RB_Uart_read(ring_buffer *_rx_buffer);
/*-------------------------------------------------------------------------------------------------*/

/* writes the data to the tx_buffer and increment the head count in tx_buffer */
void RB_Uart_write(UART_HandleTypeDef *huart, ring_buffer *_tx_buffer, int c);
/*-------------------------------------------------------------------------------------------------*/

/* function to send the string to the uart */
void RB_Uart_sendstring(UART_HandleTypeDef *huart, ring_buffer *_tx_buffer, const char *s);
/*-------------------------------------------------------------------------------------------------*/

/* Print a number with any base
 * base can be 10, 8 etc*/
void RB_Uart_printbase(UART_HandleTypeDef *huart, ring_buffer *_tx_buffer, long n, uint8_t base);
/*-------------------------------------------------------------------------------------------------*/

/* Checks if the data is available to read in the rx_buffer */
uint16_t RB_IsDataAvailable(ring_buffer *_rx_buffer);
/*-------------------------------------------------------------------------------------------------*/

/* Look for a particular string in the given buffer
 * @return 1, if the string is found and -1 if not found
 * @USAGE:: if (Look_for ("some string", buffer)) do something
 */
int Look_for (char *str, char *buffertolookinto);
/*-------------------------------------------------------------------------------------------------*/

/* Peek for the data in the Rx Bffer without incrementing the tail count 
* Returns the character
* USAGE: if (Uart_peek () == 'M') do something 
*/
int RB_Uart_peek(ring_buffer *_rx_buffer);
/*-------------------------------------------------------------------------------------------------*/

/* Copy the data from the Rx buffer into the bufferr, Upto and including the entered string 
* This copying will take place in the blocking mode, so you won't be able to perform any other operations
* Returns 1 on success and -1 otherwise
* USAGE: while (!(Copy_Upto ("some string", buffer)));
*/
//		###			int Copy_upto (char *string, char *buffertocopyinto);
/*-------------------------------------------------------------------------------------------------*/ //TO BE ADDED

/* Copies the entered number of characters (blocking mode) from the Rx buffer into the buffer, after some particular string is detected
* Returns 1 on success and -1 otherwise
* USAGE: while (!(Get_after ("some string", 6, buffer)));
*/
//		###			int Get_after (char *string, uint8_t numberofchars, char *buffertosave);
/*-------------------------------------------------------------------------------------------------*/ //TO BE ADDED

/* Wait until a paricular string is detected in the Rx Buffer
* Return 1 on success and -1 otherwise
* USAGE: while (!(Wait_for("some string")));
*/
//		###			int Wait_for (char *string);
/*-------------------------------------------------------------------------------------------------*/	//TO BE ADDED

/* the ISR for the uart. put it in the IRQ handler */
void RB_UART_ISR(UART_HandleTypeDef *huart, ring_buffer *_rx_buffer, ring_buffer *_tx_buffer);
/*-------------------------------------------------------------------------------------------------*/


/*** Depreciated For now. This is not needed, try using other functions to meet the requirement ***/
/* get the position of the given string within the incoming data.
 * It returns the position, where the string ends 
 */
//uint16_t Get_position (char *string);

/* once you hit 'enter' (\r\n), it copies the entire string to the buffer*/
//void Get_string (char *buffer);



#endif /* UARTRINGBUFFER_H_ */
