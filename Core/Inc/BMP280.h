/*
 * BMP280.h
 *
 *  Created on: Jul 27, 2025
 *      Author: davv
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stm32f4xx_hal.h"

#define BMP_CS_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BMP_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BMP_MAX_TRANSFER 25

typedef struct {
    uint8_t osrs_t;   // Temperature oversampling
    uint8_t osrs_p;   // Pressure oversampling
    uint8_t mode;     // Sensor mode (sleep/forced/normal)
    uint8_t t_sb;     // Standby time (normal mode only)
    uint8_t filter;   // IIR filter setting
    uint8_t spi3w_en; // 3-wire SPI mode enable (0 = 4-wire, 1 = 3-wire)
} BMP_init;

typedef struct {
	int32_t pressure;
	int32_t temperature;
} BMP_data_raw;

typedef struct {
	float pressure;
	float temperature;
} BMP_data;

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP_calib;

extern uint32_t last_baro_time;
extern float baro_altitude;
extern float baro_rate;

extern volatile uint8_t bmp_data_ready;
extern volatile uint8_t bmp_rx_ready;
extern volatile uint8_t bmp_tx_ready;

extern uint8_t spi_tx_buf[BMP_MAX_TRANSFER];
extern uint8_t spi_rx_buf[BMP_MAX_TRANSFER];
extern uint8_t expected_read_len;
extern BMP_data BMP_Data;



uint8_t SPI_BMP280_Init(SPI_HandleTypeDef *hspi, BMP_init *BMP_Init, BMP_calib *BMP_Calib);
void bmp280_read_registers(SPI_HandleTypeDef *hspi, uint8_t reg_addr, uint8_t len);
void bmp280_write_register(SPI_HandleTypeDef *hspi, uint8_t reg_addr, uint8_t value);
float BMP280_compensate_temperature(int32_t adc_T, const BMP_calib *calib);
float BMP280_compensate_pressure(int32_t adc_P, const BMP_calib *calib);
float BMP_calculate_altitude(float pressure);
void update_baro_rate(void);

#endif /* INC_BMP280_H_ */
