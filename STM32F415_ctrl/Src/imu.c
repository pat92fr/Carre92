/*
 * imu.c
 *
 *  Created on: 28 sept. 2019
 *      Author: Patrick
 */

#include "stm32f4xx_hal.h"
#include "imu.h"
#include "serial.h"

#include <string.h>
#include <math.h>

// HW : https://www.pololu.com/product/2468
// DS : https://www.st.com/resource/en/datasheet/l3gd20h.pdf
// AN : https://www.pololu.com/file/0J1088/LSM6DS33-AN4682.pdf

// device I2C address (L3GD20H)
#define GYRO_I2C_ADDRESS 	0x6b

// device internal register addresses (L3GD20H)
#define WHO_AM_I_ADDRESS 	0x0F
#define CTRL1 				0x20
#define CTRL2 				0x21
#define CTRL3 				0x22
#define CTRL4 				0x23
#define CTRL5 				0x24
#define LOW_ODR 			0x39
#define STATUS				0x27
#define OUT_X_L 			0x28
#define OUT_Z_L 			0x2C
#define OUT_Z_H 			0x2D

// register default value (L3GD20H)
#define WHO_AM_I_VALUE 0xD7

// register configuration values (L3GD20H)
#define CTRL4_value_init 	0x80 // GYR: BDU enabled, BLE Data LSB @ lower address, ± 245 dps Full scale
#define LOW_ODR_value_init  0x01 // GYR: Low Speed ODR enabled (<100Hz)
#define CTRL1_value_init  	0xFC // GYR: LOW ODR, ODR 50Hz, cut off 16Hz, Normal mode, Z-only enabled

// constants
#define ANGULAR_RATE_SENSITIVITY_245 0.00875 // factory sensitivity (p.15 datasheet)
#define ANGULAR_RATE_SENSITIVITY_500 0.0175 // factory sensitivity (p.15 datasheet)
#define ANGULAR_RATE_SENSITIVITY_1000 0.0350 // factory sensitivity (p.15 datasheet)

// globals
extern I2C_HandleTypeDef hi2c1;
#ifdef IMU_TRACE
	static HAL_Serial_Handler ai_com;
#endif

// private data ///////////////////////////////////////////////////////////////

typedef struct {
	int16_t raw_value; // 12-bit measure
	float rate; //dps
	float bias; // dps
	float heading; //degres
	uint32_t locked;
} ctx_gyro;

static ctx_gyro ctx;

// private functions //////////////////////////////////////////////////////////

// read helper for I2C operation
// input : device (7bit, not shifted) and register (8bit) addresses
// output : register value (8bit)
uint8_t gyro_read_8bit_register(
		uint8_t device_address,
		uint8_t register_address,
		HAL_StatusTypeDef * res
	)
{
	// send the register address to I2C device
	*res = HAL_I2C_Master_Transmit(&hi2c1, device_address << 1, &register_address , 1, 10);
	if(*res==HAL_OK)
	{
		uint8_t data = 0;
		// read the register value from I2C device
		*res = HAL_I2C_Master_Receive(&hi2c1, device_address << 1, &data, 1, 10);
		if(*res==HAL_OK)
		{
			// return the register value
			return data;
		}
		else
		{
			return 0xFF;
		}
	}
	else
	{
		return 0xFF;
	}
}

// write helper for I2C operation
// input : device (7bit, not shifted) and register (8bit) addresses, register value (8bit)
void gyro_write_8bit_register(
		uint8_t device_address,
		uint8_t register_address,
		uint8_t data,
		HAL_StatusTypeDef * res
	)
{
	// send the register address and data to I2C device
	uint8_t data_buf[]= {register_address, data};
	*res = HAL_I2C_Master_Transmit(&hi2c1, device_address << 1, data_buf , 2, 10);
}

// public functions ///////////////////////////////////////////////////////////

uint32_t gyro_init()
{
	ctx.raw_value = 0;
	ctx.rate = 0.0;
	ctx.bias = INIT_GYRO_BIAS;
	ctx.heading = 0.0f;
	ctx.locked = 0;
	HAL_StatusTypeDef result;
	uint8_t who_am_i = gyro_read_8bit_register(GYRO_I2C_ADDRESS,WHO_AM_I_ADDRESS,&result);
	if(result != HAL_OK)
	{
		return GYRO_NOT_DETECTED;
	}
	if(who_am_i != WHO_AM_I_VALUE)
	{
		return GYRO_NOT_IDENTIFIED;
	}
	gyro_write_8bit_register(GYRO_I2C_ADDRESS, CTRL4, CTRL4_value_init, &result);
	uint8_t res_read = gyro_read_8bit_register(GYRO_I2C_ADDRESS, CTRL4, &result);
	if(res_read!=CTRL4_value_init)
	{
		return GYRO_SETUP_FAILURE;
	}
	gyro_write_8bit_register(GYRO_I2C_ADDRESS, LOW_ODR, LOW_ODR_value_init, &result);
	res_read = gyro_read_8bit_register(GYRO_I2C_ADDRESS, LOW_ODR, &result);
	if(res_read!=LOW_ODR_value_init)
	{
		return GYRO_SETUP_FAILURE;
	}
	gyro_write_8bit_register(GYRO_I2C_ADDRESS, CTRL1, CTRL1_value_init, &result); // enforce BDU
	res_read = gyro_read_8bit_register(GYRO_I2C_ADDRESS, CTRL1, &result);
	if(res_read!=CTRL1_value_init)
	{
		return GYRO_SETUP_FAILURE;
	}
	return GYRO_OK;
}

void gyro_update(float duration_s)
{
	HAL_StatusTypeDef result;
	// TODO : burst read (16bits)
	uint8_t res_read_H = gyro_read_8bit_register(GYRO_I2C_ADDRESS, OUT_Z_H, &result);
	uint8_t res_read_L = gyro_read_8bit_register(GYRO_I2C_ADDRESS, OUT_Z_L, &result);
	ctx.raw_value = ((uint16_t)(res_read_H) << 8) + (uint16_t) res_read_L;
	ctx.rate = (float)(ctx.raw_value*ANGULAR_RATE_SENSITIVITY_245*GYRO_SENSITIVITY_CORRECTION);
	ctx.heading += (ctx.rate - ctx.bias)*duration_s;
}

float gyro_get_dps()
{
	return ctx.rate- ctx.bias;
}

void gyro_reset_heading()
{
	ctx.heading = 0.0F;
}

float gyro_get_heading()
{
	return ctx.heading;
}

bool gyro_is_calibrated()
{
	return ctx.locked >= 128;
}

float mean = 0.0;
float variance = 0.0;
float alpha_mean_update = 0.01;
float alpha_variance_update = 0.05;
float alpha_bias_update = 0.01;

void gyro_auto_calibrate(float duration_s)
{
	gyro_update(duration_s);
	// update mean and variance
	mean = alpha_mean_update *ctx.rate + (1.0-alpha_mean_update) * mean;
	variance = alpha_variance_update * pow( ctx.rate-mean,2)  + (1.0-alpha_variance_update) * variance;
	// if mean stable, update bias
	if(variance<GYRO_AUTOCAL_VARIANCE_THRESHOLD)
	{
		ctx.bias = alpha_bias_update*mean + (1.0-alpha_bias_update)* ctx.bias;
		if(ctx.locked<1024)
			++ctx.locked;
	}
#ifdef IMU_TRACE
		if((time%500)==0)
			HAL_Serial_Print(&ai_com,"dps=%d m=%d v=%d b=%d h=%d rate=%d\r\n",
				(int32_t)(ctx.rate*1000.0),
				(int32_t)(mean*1000.0),
				(int32_t)(variance*1000.0),
				(int32_t)(ctx.bias*1000.0),
				(int32_t)(ctx.heading),
				(int32_t)(gyro_get_dps()*1000.0)
								  );
#endif
}
