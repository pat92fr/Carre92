/*
 * delay.c
 *
 *  Created on: 26 d�c. 2015
 *      Author: Patrick
 */

#include "delay.h"

void HAL_Delay(uint32_t ms)
{
    uint32_t end = HAL_GetTick()+ms;
    while(HAL_GetTick()<end);
}

