/*
 * serial.h
 *
 *  Created on: 29 nov. 2015
 *      Author: Patrick
 */

#include "stm32f4xx_hal.h"

/* HAL Public Data ------------------------------------------------------------------*/

#define size_of_tx_buffer 1024
#define size_of_rx_circular_buffer 1024

struct t_HAL_Serial_Handler
{
    UART_HandleTypeDef * huart;
    uint8_t tx_buffer[size_of_tx_buffer];
    uint8_t rx_circular_buffer[size_of_rx_circular_buffer];
    uint8_t const * rx_tail_ptr;
    volatile int tx_dma;
    //HAL_LockTypeDef               Lock;            /* Locking object                     */
};

typedef struct t_HAL_Serial_Handler HAL_Serial_Handler;

#define HAL_Serial_Handler_Count 4

/* HAL Functions ------------------------------------------------------------------*/

void HAL_Serial_Init(UART_HandleTypeDef * huart, HAL_Serial_Handler * hserial);
int HAL_Serial_Available(HAL_Serial_Handler * hserial);
char HAL_Serial_GetChar(HAL_Serial_Handler * hserial);
int HAL_Serial_Read(HAL_Serial_Handler * hserial, uint8_t * ptr, int len );
int HAL_Serial_Write(HAL_Serial_Handler * hserial, uint8_t const * ptr, int len );
int HAL_Serial_Print(HAL_Serial_Handler * hserial,const char *fmt, ...);
