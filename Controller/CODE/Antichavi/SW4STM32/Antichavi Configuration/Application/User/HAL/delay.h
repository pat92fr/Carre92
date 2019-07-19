/*
 * delay.h
 *
 *  Created on: 26 déc. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_DELAY_H_
#define APPLICATION_USER_HAL_DELAY_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* HAL Functions ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

void HAL_Delay(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_HAL_DELAY_H_ */
