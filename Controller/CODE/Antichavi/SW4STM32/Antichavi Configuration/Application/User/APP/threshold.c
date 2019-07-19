/*
 * threshold.c
 *
 *  Created on: 3 janv. 2016
 *      Author: Patrick
 */

#include "imu.h"
#include "threshold.h"

static float threshold_max_angle[THRESHOLD_ANGLE_COUNT] = { 25.0f, 90.0f };
static float threshold_min_angle[THRESHOLD_ANGLE_COUNT] = { -25.0f, -10.0f };
static float threshold_hysteresis_angle[THRESHOLD_ANGLE_COUNT] = { 10.0f, 5.0f };
static int threshold_status[THRESHOLD_ANGLE_COUNT] = { THRESHOLD_STATUS_OK, THRESHOLD_STATUS_OK };

#define ABS(x) (x>=0?x:-x)

void HAL_Threshold_Init(void)
{
	// TODO : save/Rstore from rom
	for(short index=0;index<THRESHOLD_ANGLE_COUNT;++index)
		threshold_status[index] = THRESHOLD_STATUS_OK;
}

void HAL_Threshold_Process(void)
{
	// threshold is +/- 1°
	for(short index=0;index<THRESHOLD_ANGLE_COUNT;++index)
	{
		float angle = HAL_Imu_Angle(index);
		switch(threshold_status[index])
		{
		case THRESHOLD_STATUS_OK:
			{
				if( (angle>=threshold_max_angle[index]) || (angle<=threshold_min_angle[index]) )
						threshold_status[index] = THRESHOLD_STATUS_ALERT;
			}
			break;
		case THRESHOLD_STATUS_ALERT:
			{
				if( (angle<threshold_max_angle[index]-threshold_hysteresis_angle[index]) && (angle>threshold_min_angle[index]+threshold_hysteresis_angle[index]) )
						threshold_status[index] = THRESHOLD_STATUS_OK;
			}
			break;
		}
	}
}

float HAL_Threshold_Set(int axe)
{
	// TODO : save/Rstore from rom
	return  threshold_max_angle[axe];
}

void HAL_Threshold_Get(int axe, float angle)
{
	// TODO : save/Rstore from rom
	threshold_max_angle[axe] = angle;
}

int HAL_Threshold_Status(int axe)
{
	return  threshold_status[axe];
}
