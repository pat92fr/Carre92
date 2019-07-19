/*
 * battery.c
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

/* Includes ------------------------------------------------------------------*/
#include "battery.h"

/* Private data --------------------------------------------------------------*/

extern ADC_HandleTypeDef hadc1;

__IO uint16_t ADC_Converted_Values[2] = {0,0}; /// I used two storage values to use circular DMA mode in continuous/scan ADM mode. This allow 32-bit DMA transfer.

/* Private functions ---------------------------------------------------------*/


/* APP functions ---------------------------------------------------------*/

void HAL_Battery_Init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Converted_Values, 2);
}

float HAL_Battery_Get(int id)
{
    return (float)(ADC_Converted_Values[id]) * 3.3 / ( 4096.0 * hal_battery_voltage_hw_ration[id] ); /// I take the second conversion.
}

bool HAL_Battery_Is_Low(int id)
{
	float const v = HAL_Battery_Get(id);
    return v<hal_battery_voltage_low_threshold[id];
}


