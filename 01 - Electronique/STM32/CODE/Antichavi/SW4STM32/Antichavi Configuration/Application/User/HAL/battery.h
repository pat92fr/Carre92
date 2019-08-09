/*
 * battery.h
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_BATTERY_H_
#define APPLICATION_USER_HAL_BATTERY_H_


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* HAL settings ------------------------------------------------------------------*/

enum VBATT_ID
{
	VBATT = 0,
	VRX,
	VBATT_COUNT
};

static float const hal_battery_voltage_hw_ration[VBATT_COUNT] = {
	0.323,
	0.313
};

static float const hal_battery_voltage_low_threshold[VBATT_COUNT] = {
	6.4, // V
	4.0
};


/* HAL Public Data ------------------------------------------------------------------*/


/* HAL Functions ------------------------------------------------------------------*/

void HAL_Battery_Init(void);

float HAL_Battery_Get(int id); // volt

bool HAL_Battery_Is_Low(int id);

#endif /* APPLICATION_USER_HAL_BATTERY_H_ */
