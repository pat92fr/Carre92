/*
 * led.h
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_LED_H_
#define APPLICATION_USER_HAL_LED_H_


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HAL settings ------------------------------------------------------------------*/

enum LED_ID
{
	GREEN_LED0 = 0,
	GREEN_LED1,
	RED_LED0,
	RED_LED1,
	GREEN_LED2,
	GREEN_LED3,
	LED_COUNT
};

static uint16_t const hal_led_id_to_pin[LED_COUNT] = {
	LED1_Pin,
	LED0_Pin,
	LED2_Pin,
	LED3_Pin,
	LED4_Pin,
	LED5_Pin
};

static GPIO_TypeDef * const hal_led_id_to_port[LED_COUNT] = {
	LED1_GPIO_Port,
	LED0_GPIO_Port,
	LED2_GPIO_Port,
	LED3_GPIO_Port,
	LED4_GPIO_Port,
	LED5_GPIO_Port
};

/* HAL Public Data ------------------------------------------------------------------*/

enum LED_STATE
{
    LED_OFF = 0,
    LED_ON = 1
};

/* HAL Functions ------------------------------------------------------------------*/

void HAL_Led_Init(void);
void HAL_Led_Process(void);

void HAL_Led_Set(int id);
void HAL_Led_Reset(int id);
void HAL_Led_Toggle(int id);

int HAL_Led_Get(int id);

void HAL_Led_Blink(int id, int times, int period_ms); // times = 0 ==> non stop blinking

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_HAL_LED_H_ */
