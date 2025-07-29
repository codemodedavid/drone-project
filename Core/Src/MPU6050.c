/*
 * MPU6050.c
 *
 *  Created on: Jul 27, 2025
 *      Author: davv
 */

#include "MPU6050.h"
volatile uint8_t mpu_data_ready = 0;

#include "MPU6050.h"
#include "stm32f4xx_hal.h"  // for HAL_StatusTypeDef, I2C_HandleTypeDef

// LSB per g for ±2g, ±4g, ±8g, ±16g
static const float acc_lsb_map[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};

// LSB per °/s for ±250, ±500, ±1000, ±2000 °/s
static const float gyr_lsb_map[4] = {131.0f, 65.5f, 32.8f, 16.4f};

float get_acc_lsb(uint8_t afs_sel) {
    return acc_lsb_map[afs_sel & 0x03];  // mask to ensure valid index
}

float get_gyr_lsb(uint8_t fs_sel) {
    return gyr_lsb_map[fs_sel & 0x03];
}

HAL_StatusTypeDef I2C_MPU6050_Init(I2C_HandleTypeDef *hi2c, const MPU_init *MPU_Init) {
    uint8_t reg;
    uint8_t DevAddress = MPU_Init->ADDRESS;
    HAL_StatusTypeDef status;

    // CONFIG
    status = HAL_I2C_Mem_Read(hi2c, DevAddress, 0x1A, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;
    reg &= ~(0x07 << 0);
    reg |= (MPU_Init->DLPF_CFG & 0x07) << 0;
    status = HAL_I2C_Mem_Write(hi2c, DevAddress, 0x1A, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;

    // SMPLRT_DIV
    reg = (MPU_Init->DLPF_CFG == 0) ? 8 : MPU_Init->SMPLRT_DIV;
    status = HAL_I2C_Mem_Write(hi2c, DevAddress, 0x19, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;

    // GYRO_CONFIG
    status = HAL_I2C_Mem_Read(hi2c, DevAddress, 0x1B, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;
    reg &= ~((1 << 3) | (1 << 4));
    reg |= (MPU_Init->FS_SEL & 0x03) << 3;
    status = HAL_I2C_Mem_Write(hi2c, DevAddress, 0x1B, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;

    // ACCEL_CONFIG
    status = HAL_I2C_Mem_Read(hi2c, DevAddress, 0x1C, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;
    reg &= ~((1 << 3) | (1 << 4));
    reg |= (MPU_Init->AFS_SEL & 0x03) << 3;
    status = HAL_I2C_Mem_Write(hi2c, DevAddress, 0x1C, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;

    // INT_PIN_CFG
    status = HAL_I2C_Mem_Read(hi2c, DevAddress, 0x37, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;
    reg &= ~((1 << 4) | (1 << 5) | (1 << 6) | (1 << 7));
    reg |= ((MPU_Init->INT_LEVEL & 0x01) << 7)
         | ((MPU_Init->INT_OPEN & 0x01) << 6)
         | ((MPU_Init->LATCH_INT_EN & 0x01) << 5)
         | ((MPU_Init->INT_RD_CLEAR & 0x01) << 4);
    status = HAL_I2C_Mem_Write(hi2c, DevAddress, 0x37, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;

    // INT_ENABLE
    status = HAL_I2C_Mem_Read(hi2c, DevAddress, 0x38, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;
    reg &= ~0x01;
    reg |= (MPU_Init->DATA_RDY_EN & 0x01);
    status = HAL_I2C_Mem_Write(hi2c, DevAddress, 0x38, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;

    // PWR_MGMT_1
    status = HAL_I2C_Mem_Read(hi2c, DevAddress, 0x6B, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;
    reg &= ~((1 << 6) | 0x07);
    reg |= ((MPU_Init->SLEEP & 0x01) << 6)
         |  ((MPU_Init->CLKSEL & 0x07) << 0);
    status = HAL_I2C_Mem_Write(hi2c, DevAddress, 0x6B, 1, &reg, 1, 10);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

int calibrate_accel_gyro(MPU_bias *bias_out, MPU_data_raw *raw, int samples, float LSB_ACC) {
    int32_t ax = 0, ay = 0, az = 0;
    int32_t gx = 0, gy = 0, gz = 0;

    for (int i = 0; i < samples; i++) {
    	while (!mpu_data_ready);
    	mpu_data_ready = 0;
        ax += raw->ax;
        ay += raw->ay;
        az += raw->az;
        gx += raw->gx;
        gy += raw->gy;
        gz += raw->gz;
    }

    // Compute average
    bias_out->ax_bias = ax / (float)samples;
    bias_out->ay_bias = ay / (float)samples;
    bias_out->az_bias = (az / (float)samples) - LSB_ACC;

    bias_out->gx_bias = gx / (float)samples;
    bias_out->gy_bias = gy / (float)samples;
    bias_out->gz_bias = gz / (float)samples;
    return 1;
}

#include "MPU6050.h"



