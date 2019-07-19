/*
 * app_fw.c
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#include "app_fw.h"
#include "stm32f4xx_hal.h"

__weak void APP_SetupCallback(void)
{
  /* NOTE: This function Should not be modified, when the callback is needed,
           the APP_SetupCallback could be implemented in the user file
   */
}

__weak void APP_LoopCallback(void)
{
  /* NOTE: This function Should not be modified, when the callback is needed,
           the APP_LoopCallback could be implemented in the user file
   */
}

__weak void APP_SystickCallback(void)
{
  /* NOTE: This function Should not be modified, when the callback is needed,
           the APP_SystickCallback could be implemented in the user file
   */
}

/* HAL to APP wrapper -------------------------------------------------------*/

void HAL_SYSTICK_Callback(void)
{
  APP_SystickCallback();
}

