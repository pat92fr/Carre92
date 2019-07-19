/*
 * app_main.c
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#include "app_fw.h"
#include "stm32f4xx_hal.h"
#include "led.h"
#include "battery.h"
#include "imu.h"
#include "serial.h"
#include "threshold.h"
#include "ihm.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;

extern UART_HandleTypeDef huart3;
HAL_Serial_Handler com;

static bool configured = false;

#define DEFAULT_WINCH_POSITION 1750
#define SAFE_WINCH_POSITION 1250

enum
{
	MAIN_STATE_NORMAL,
	MAIN_STATE_OVERRIDE,
	MAIN_STATE_COUNT
};

int main_state = MAIN_STATE_NORMAL;

uint16_t period = 0;
uint16_t width = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim9)
	{
		period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		if (period != 0)
		{
			width = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
//		  if(roll>-30 && roll<30)
//			  htim3.Instance->CCR1=width;
//		  else
//			  htim3.Instance->CCR1=SAFE_WINCH_POSITION;

	}

}
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//  {
//    /* Get the Input Capture value */
//    uwIC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
//
//    if (uwIC2Value != 0)
//    {
//      /* Duty cycle computation */
//      uwDutyCycle = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 100) / uwIC2Value;
//
//      /* uwFrequency computation
//      TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */
//      uwFrequency = (HAL_RCC_GetHCLKFreq())/2 / uwIC2Value;
//    }
//    else
//    {
//      uwDutyCycle = 0;
//      uwFrequency = 0;
//    }
//  }
//}


void APP_SetupCallback(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);

	htim1.Instance->CCR1=DEFAULT_WINCH_POSITION;
	htim1.Instance->CCR2=DEFAULT_WINCH_POSITION;
	htim1.Instance->CCR3=DEFAULT_WINCH_POSITION;
	htim1.Instance->CCR4=DEFAULT_WINCH_POSITION;
	htim8.Instance->CCR1=DEFAULT_WINCH_POSITION;
	htim8.Instance->CCR2=DEFAULT_WINCH_POSITION;
	htim8.Instance->CCR3=DEFAULT_WINCH_POSITION;
	htim8.Instance->CCR4=DEFAULT_WINCH_POSITION;

	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim9,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim9,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim12,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim12,TIM_CHANNEL_2);

	HAL_Led_Init();
	HAL_Led_Set(GREEN_LED0);
	HAL_Battery_Init();
	HAL_Imu_Init();
	HAL_Threshold_Init();
	HAL_Ihm_Init();
	HAL_Led_Blink(GREEN_LED1,0,250);
	HAL_Serial_Init(&huart3,&com);
	HAL_Serial_Print(&com,"%d %d %d\n",
			(int)(HAL_Imu_Pitch()),
			(int)(HAL_Imu_Roll()),
			(int)(HAL_Imu_Yaw())
		);
	configured = true;
	main_state = MAIN_STATE_NORMAL;
}

uint32_t imu__iteration = 0;

void APP_LoopCallback(void)
{
	period = htim2.Instance->CCR1;
  width = htim2.Instance->CCR2;

	HAL_Led_Process();

	/// Battery monitoring
	if(!HAL_Battery_Is_Low(VBATT))
	{
		HAL_Led_Set(GREEN_LED2);
	}
	else
	{
		HAL_Led_Reset(GREEN_LED2);
	}
	if(!HAL_Battery_Is_Low(VRX))
	{
		HAL_Led_Set(GREEN_LED3);
	}
	else
	{
		HAL_Led_Reset(GREEN_LED3);
	}

	/// Watch pitch and roll angle
	HAL_Imu_Process();
	HAL_Threshold_Process();
	if(HAL_Threshold_Status(THRESHOLD_ANGLE_PITCH)!=THRESHOLD_STATUS_OK)
	{
		HAL_Led_Set(RED_LED0);
	}
	else
	{
		HAL_Led_Reset(RED_LED0);
	}
	if(HAL_Threshold_Status(THRESHOLD_ANGLE_ROLL)!=THRESHOLD_STATUS_OK)
	{
		HAL_Led_Set(RED_LED1);
	}
	else
	{
		HAL_Led_Reset(RED_LED1);
	}
	switch(main_state)
	{
	case MAIN_STATE_NORMAL:
		{
			htim1.Instance->CCR1=width;
			htim1.Instance->CCR2=width;
			htim1.Instance->CCR3=width;
			htim1.Instance->CCR4=width;
			htim8.Instance->CCR1=width;
			htim8.Instance->CCR2=width;
			htim8.Instance->CCR3=width;
			htim8.Instance->CCR4=width;

			if((HAL_Threshold_Status(THRESHOLD_ANGLE_PITCH)!=THRESHOLD_STATUS_OK)||(HAL_Threshold_Status(THRESHOLD_ANGLE_ROLL)!=THRESHOLD_STATUS_OK))
			{
				HAL_Led_Blink(GREEN_LED1,0,75);
				main_state = MAIN_STATE_OVERRIDE;
			}
		}
		break;
	case MAIN_STATE_OVERRIDE:
		{
			htim1.Instance->CCR1=SAFE_WINCH_POSITION;
			htim1.Instance->CCR2=SAFE_WINCH_POSITION;
			htim1.Instance->CCR3=SAFE_WINCH_POSITION;
			htim1.Instance->CCR4=SAFE_WINCH_POSITION;
			htim8.Instance->CCR1=SAFE_WINCH_POSITION;
			htim8.Instance->CCR2=SAFE_WINCH_POSITION;
			htim8.Instance->CCR3=SAFE_WINCH_POSITION;
			htim8.Instance->CCR4=SAFE_WINCH_POSITION;

			if((HAL_Threshold_Status(THRESHOLD_ANGLE_PITCH)==THRESHOLD_STATUS_OK)&&(HAL_Threshold_Status(THRESHOLD_ANGLE_ROLL)==THRESHOLD_STATUS_OK))
			{
				HAL_Led_Blink(GREEN_LED1,0,250);
				main_state = MAIN_STATE_NORMAL;
			}
		}
		break;
	}
	HAL_Ihm_Process();

	// debug
	HAL_Delay(1);
	++imu__iteration;
	if((imu__iteration%200)==0)
		HAL_Serial_Print(&com,"%d %d %d\n",
				(int)(HAL_Imu_Pitch()),
				(int)(HAL_Imu_Roll()),
				(int)(HAL_Imu_Yaw())
			);
}

void APP_SystickCallback(void)
{
	if(configured)
	{

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==BUTTON_0_Pin)
	{

	}
}
