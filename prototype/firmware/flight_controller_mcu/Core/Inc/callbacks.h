/*
 * callbacks.h
 *
 *  Created on: Jul 27, 2025
 *      Author: davv
 */

#ifndef INC_CALLBACKS_H_
#define INC_CALLBACKS_H_

#include "stm32f4xx_hal.h"
#include "MPU6050.h"
#include "BMP280.h"

extern volatile uint8_t dma_read_enabled;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);

static inline HAL_StatusTypeDef Get_Raw_Data_DMA(I2C_HandleTypeDef *hi2c, MPU_init *init, uint8_t *dma_buffer) {
    return HAL_I2C_Mem_Read_DMA(hi2c, init->ADDRESS, 0x3A, I2C_MEMADD_SIZE_8BIT, dma_buffer, 16);
}

static inline void Parse_Raw_MPU_Data(MPU_data_raw *raw, uint8_t *buf) {
	raw->ax   = (int16_t)(buf[1] << 8 | buf[2]);
	raw->ay   = (int16_t)(buf[3] << 8 | buf[4]);
	raw->az   = (int16_t)(buf[5] << 8 | buf[6]);
	raw->temp = (int16_t)(buf[7] << 8 | buf[8]);
	raw->gx   = (int16_t)(buf[9] << 8 | buf[10]);
	raw->gy   = (int16_t)(buf[11] << 8 | buf[12]);
	raw->gz   = (int16_t)(buf[13] << 8 | buf[14]);
}

#endif /* INC_CALLBACKS_H_ */
