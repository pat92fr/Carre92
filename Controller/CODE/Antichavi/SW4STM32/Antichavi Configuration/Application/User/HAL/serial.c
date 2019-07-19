/*
 * serial.c
 *
 *  Created on: 29 nov. 2015
 *      Author: Patrick
 */

#include"serial.h"
#include "string.h"
#include "itoa.h"

#include <stdarg.h>

/// Store hserial to handle HAL_UART_TxCpltCallback
/// This table has to be filled in the application code
HAL_Serial_Handler * hserial_table[HAL_Serial_Handler_Count] = {0,0,0,0};
uint32_t serial_counter = 0;



void HAL_Serial_Init(UART_HandleTypeDef * huart, HAL_Serial_Handler * hserial)
{
    hserial->huart = huart;
	hserial->tx_dma = 0;
    hserial->rx_tail_ptr = hserial->rx_circular_buffer;
	hserial_table[serial_counter++]=hserial;
    HAL_UART_Receive_DMA(hserial->huart, hserial->rx_circular_buffer,size_of_rx_circular_buffer);
}

int HAL_Serial_Available(HAL_Serial_Handler * hserial)
{
    if(hserial->huart == 0)
        return 0;
    uint8_t const * head = hserial->rx_circular_buffer + size_of_rx_circular_buffer - __HAL_DMA_GET_COUNTER(hserial->huart->hdmarx);
    uint8_t const * tail = hserial->rx_tail_ptr;
    if( head>=tail )
        return head-tail;
    else
        return head-tail+size_of_rx_circular_buffer;
}

char HAL_Serial_GetChar(HAL_Serial_Handler * hserial)
{
    if(hserial->huart == 0)
        return 0;
    uint8_t const * head = hserial->rx_circular_buffer + size_of_rx_circular_buffer - __HAL_DMA_GET_COUNTER(hserial->huart->hdmarx);
    uint8_t const * tail = hserial->rx_tail_ptr;
    if(head!=tail)
    {
        char c =  *hserial->rx_tail_ptr++;
        if(hserial->rx_tail_ptr>=hserial->rx_circular_buffer + size_of_rx_circular_buffer)
            hserial->rx_tail_ptr-=size_of_rx_circular_buffer;
        return c;
    }
    else
        return 0;
}

int HAL_Serial_Read(HAL_Serial_Handler * hserial, uint8_t * ptr, int len )
{
    if(hserial->huart == 0)
        return 0;
    uint8_t const * head = hserial->rx_circular_buffer + size_of_rx_circular_buffer - __HAL_DMA_GET_COUNTER(hserial->huart->hdmarx);
    int counter = 0;
    while(counter!=len)
    {
        if(head==hserial->rx_tail_ptr)
            return counter;
        *ptr++=*hserial->rx_tail_ptr++;
        if(hserial->rx_tail_ptr>=hserial->rx_circular_buffer + size_of_rx_circular_buffer)
            hserial->rx_tail_ptr-=size_of_rx_circular_buffer;
        ++counter;
    }
    return counter;
}

int HAL_Serial_Write(HAL_Serial_Handler * hserial, uint8_t const * ptr, int len )
{
    if(hserial->huart == 0)
        return 0;
    while(hserial->tx_dma!=0);
    hserial->tx_dma=1;
    // copy user buffer to serial internal tx buffer
    uint8_t const * ptr_end = ptr + len;
    uint8_t * dst = hserial->tx_buffer;
    while( ptr < ptr_end )
    {
        *dst++=*ptr++;
    }
    // transmit serial internal tx buffer using DMA if no DMA running
    HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(hserial->huart, hserial->tx_buffer,len);
    if(result!=HAL_OK)
    {
    	HAL_Delay(100);
    }
    //HAL_UART_Transmit(hserial->huart, hserial->tx_buffer,len,100);
    return len;
}

void HAL_USART_ErrorCallback()
{
	HAL_Delay(100);

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    static int id = 0;
    for(id=0;id!=serial_counter;++id)
    {
        if(hserial_table[id]->huart == huart && hserial_table[id]->tx_dma == 1)
        {
    	   hserial_table[id]->tx_dma = 0;
           break;
        }
    }
}


/**
**---------------------------------------------------------------------------
**  Abstract: Convert integer to ascii
**  Returns:  void
**---------------------------------------------------------------------------
*/
void ts_itoa(char **buf, unsigned int d, int base)
{
	int div = 1;
	while (d/div >= base)
		div *= base;

	while (div != 0)
	{
		int num = d/div;
		d = d%div;
		div /= base;
		if (num > 9)
			*((*buf)++) = (num-10) + 'A';
		else
			*((*buf)++) = num + '0';
	}
}

/**
**---------------------------------------------------------------------------
**  Abstract: Writes arguments va to buffer buf according to format fmt
**  Returns:  Length of string
**---------------------------------------------------------------------------
*/
int ts_formatstring(char *buf, const char *fmt, va_list va)
{
	char *start_buf = buf;
	while(*fmt)
	{
		/* Character needs formating? */
		if (*fmt == '%')
		{
			switch (*(++fmt))
			{
			  case 'c':
				*buf++ = va_arg(va, int);
				break;
			  case 'd':
			  case 'i':
				{
					signed int val = va_arg(va, signed int);
					if (val < 0)
					{
						val *= -1;
						*buf++ = '-';
					}
					ts_itoa(&buf, val, 10);
				}
				break;
			  case 's':
				{
					char * arg = va_arg(va, char *);
					while (*arg)
					{
						*buf++ = *arg++;
					}
				}
				break;
			  case 'u':
					ts_itoa(&buf, va_arg(va, unsigned int), 10);
				break;
			  case 'x':
			  case 'X':
					ts_itoa(&buf, va_arg(va, int), 16);
				break;
			  case '%':
				  *buf++ = '%';
				  break;
			}
			fmt++;
		}
		/* Else just copy */
		else
		{
			*buf++ = *fmt++;
		}
	}
	*buf = 0;

	return (int)(buf - start_buf);
}


/**
**---------------------------------------------------------------------------
**  Abstract: Calculate maximum length of the resulting string from the
**            format string and va_list va
**  Returns:  Maximum length
**---------------------------------------------------------------------------
*/
int ts_formatlength(const char *fmt, va_list va)
{
	int length = 0;
	while (*fmt)
	{
		if (*fmt == '%')
		{
			++fmt;
			switch (*fmt)
			{
			  case 'c':
		  		  va_arg(va, int);
				  ++length;
				  break;
			  case 'd':
			  case 'i':
			  case 'u':
				  /* 32 bits integer is max 11 characters with minus sign */
				  length += 11;
				  va_arg(va, int);
				  break;
			  case 's':
			  	  {
			  		  char * str = va_arg(va, char *);
			  		  while (*str++)
			  			  ++length;
			  	  }
				  break;
			  case 'x':
			  case 'X':
				  /* 32 bits integer as hex is max 8 characters */
				  length += 8;
				  va_arg(va, unsigned int);
				  break;
			  default:
				  ++length;
				  break;
			}
		}
		else
		{
			++length;
		}
		++fmt;
	}
	return length;
}

int HAL_Serial_Print(HAL_Serial_Handler * hserial,const char *fmt, ...)
{
	int length = 0;
	va_list va;
	va_start(va, fmt);
	length = ts_formatlength(fmt, va);
	va_end(va);
	{
		char buf[length];
		va_start(va, fmt);
		length = ts_formatstring(buf, fmt, va);
		length = HAL_Serial_Write(hserial, (uint8_t*)buf, length);
		va_end(va);
	}
	return length;
}
