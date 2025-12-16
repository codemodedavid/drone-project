/*
 * callbacks.c
 *
 *  Created on: Jul 27, 2025
 *      Author: davv
 */
#include "callbacks.h"


#define MPU_INT_PIN GPIO_PIN_7

extern I2C_HandleTypeDef hi2c1;
extern MPU_init MPU_Init;
extern MPU_data_raw MPU_Data_Raw;
extern BMP_data_raw BMP_Data_Raw;

volatile uint8_t dma_read_enabled = 1;

static uint8_t mpu_dma_buff[16];
volatile uint8_t dma_in_progress = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU_INT_PIN && dma_read_enabled == 1 && !dma_in_progress)
    {
        // Start DMA I2C read triggered by MPU6050 interrupt
    	dma_in_progress = 1;
    	if (Get_Raw_Data_DMA(&hi2c1, &MPU_Init, mpu_dma_buff) != HAL_OK) {
    		__HAL_RCC_I2C1_FORCE_RESET();
    		HAL_Delay(1); // short delay
    		__HAL_RCC_I2C1_RELEASE_RESET();

    		// 2. Reinitialize the I2C peripheral
			HAL_I2C_DeInit(&hi2c1);
			HAL_I2C_Init(&hi2c1);

			// 3. Optionally clear your DMA flags if used
			__HAL_DMA_CLEAR_FLAG(hi2c1.hdmarx, DMA_FLAG_TCIF0_4);
			__HAL_DMA_CLEAR_FLAG(hi2c1.hdmarx, DMA_FLAG_HTIF0_4);
			__HAL_DMA_CLEAR_FLAG(hi2c1.hdmarx, DMA_FLAG_TEIF0_4);
    		dma_in_progress = 0;
    	}
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    	Parse_Raw_MPU_Data(&MPU_Data_Raw, mpu_dma_buff);
    	mpu_data_ready = 1;
    	dma_in_progress = 0;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
	        BMP_CS_HIGH();
	        bmp_rx_ready = 1;

	        if (expected_read_len == 7) {
	            // Combine pressure bytes into 20-bit raw value
	            BMP_Data_Raw.pressure = ((int32_t)spi_rx_buf[1] << 12) |
	                        ((int32_t)spi_rx_buf[2] << 4)  |
	                        ((int32_t)spi_rx_buf[3] >> 4);

	            // Combine temperature bytes into 20-bit raw value
	            BMP_Data_Raw.temperature = ((int32_t)spi_rx_buf[4] << 12) |
	                       ((int32_t)spi_rx_buf[5] << 4)  |
	                       ((int32_t)spi_rx_buf[6] >> 4);
	            bmp_data_ready = 1;
	        }
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        BMP_CS_HIGH();
        bmp_tx_ready = 1;
        // Write complete
    }
}
