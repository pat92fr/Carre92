/*
 * mmi.h
 *
 *  Created on: 22 avr. 2018
 *      Author: Patrick
 */

#ifndef MMI_H_
#define MMI_H_

#include "serial.h"

typedef enum
{
        MMI_ENTRY_DATA,
        MMI_ENTRY_FUNCTION,
        MMI_ENTRY_FUNCTION_1ARG
} MMI_ENTRY_Type;

typedef void (*HAL_MMI_Callback)(void);

typedef void (*HAL_MMI_Callback_1Arg)(unsigned int value);

typedef struct
{
    char const * code;
    MMI_ENTRY_Type type;
    union {
        float * dptr;
        HAL_MMI_Callback fptr;
        HAL_MMI_Callback_1Arg fptr1arg;
    } callback;
    int32_t weight;
} HAL_MMI_Entry;

void HAL_MMI_Init(HAL_Serial_Handler * serial);
void HAL_MMI_Configure(HAL_MMI_Entry const * entry);
void HAL_MMI_Process();

#endif /* MMI_H_ */
