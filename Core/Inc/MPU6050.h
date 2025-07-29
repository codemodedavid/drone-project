/*
 * MPU6050.h
 *
 *  Created on: Jul 27, 2025
 *      Author: davv
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"

extern volatile uint8_t mpu_data_ready;

static const float acc_lsb_map[4];
static const float gyr_lsb_map[4];

float get_acc_lsb(uint8_t afs_sel);
float get_gyr_lsb(uint8_t fs_sel);

typedef struct {
	uint8_t ADDRESS;

	uint8_t INT_LEVEL;     // 0
	uint8_t INT_OPEN;      // 0
	uint8_t LATCH_INT_EN;  // 0
	uint8_t INT_RD_CLEAR;  // 1
	uint8_t DATA_RDY_EN;   // 1

	uint8_t DLPF_CFG;      // 1
	uint8_t SMPLRT_DIV;    // 0

	uint8_t FS_SEL;        // 0/1
	uint8_t AFS_SEL;       // 0/1

	uint8_t CLKSEL;        // 0
	uint8_t SLEEP; 		   // 0
} MPU_init;

typedef struct {
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t temp;
	int16_t gx;
	int16_t gy;
	int16_t gz;
} MPU_data_raw;

typedef struct {
	float ax;
	float ay;
	float az;
	float temp;
	float gx;
	float gy;
	float gz;
} MPU_data;

typedef struct {
    float ax_bias, ay_bias, az_bias;
    float gx_bias, gy_bias, gz_bias;
} MPU_bias;


float get_acc_lsb(uint8_t afs_sel);
float get_gyr_lsb(uint8_t fs_sel);
HAL_StatusTypeDef I2C_MPU6050_Init(I2C_HandleTypeDef *hi2c, const MPU_init *MPU_Init);
int calibrate_accel_gyro(MPU_bias *bias_out, MPU_data_raw *raw, int samples, float LSB_ACC);



#endif /* INC_MPU6050_H_ */
