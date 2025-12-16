/*
 * BMP280.c
 *
 *  Created on: Jul 27, 2025
 *      Author: davv
 */
#include "BMP280.h"
#include "math.h"
#include "time_utils.h"


uint32_t last_baro_time = 0;
volatile uint8_t bmp_data_ready = 0;
volatile uint8_t bmp_rx_ready = 0;
volatile uint8_t bmp_tx_ready = 0;

uint8_t spi_tx_buf[BMP_MAX_TRANSFER];
uint8_t spi_rx_buf[BMP_MAX_TRANSFER];
uint8_t expected_read_len = 0;

float baro_altitude = 0.0f;
float baro_rate = 0.0f;

static float t_fine_float;

void bmp280_read_registers(SPI_HandleTypeDef *hspi, uint8_t reg_addr, uint8_t len) {
    spi_tx_buf[0] = reg_addr | 0x80;  // Set MSB for Read

    // Fill the rest of tx buffer with dummy bytes
    for (uint8_t i = 1; i <= len; i++) {
        spi_tx_buf[i] = 0x00;
    }

    expected_read_len = len + 1;  // +1 for register address byte
    bmp_rx_ready = 0;

    BMP_CS_LOW();
    HAL_SPI_TransmitReceive_DMA(hspi, spi_tx_buf, spi_rx_buf, expected_read_len);
}

void bmp280_write_register(SPI_HandleTypeDef *hspi, uint8_t reg_addr, uint8_t value) {
    spi_tx_buf[0] = reg_addr & 0x7F;  // Clear MSB for write
    spi_tx_buf[1] = value;

    BMP_CS_LOW();
    bmp_tx_ready = 0;
    HAL_SPI_Transmit_DMA(hspi, spi_tx_buf, 2);  // Just TX
}

uint8_t SPI_BMP280_Init(SPI_HandleTypeDef *hspi, BMP_init *BMP_Init, BMP_calib *BMP_Calib) {
    // Compose config register (0xF5)
    uint8_t config_reg =
        (BMP_Init->t_sb   << 5) |
        (BMP_Init->filter << 2) |
        (BMP_Init->spi3w_en);

    // Compose ctrl_meas register (0xF4)
    uint8_t ctrl_meas_reg =
        (BMP_Init->osrs_t << 5) |
        (BMP_Init->osrs_p << 2) |
        (BMP_Init->mode);

    // Write config
    bmp_tx_ready = 0;
    bmp280_write_register(hspi, 0xF5, config_reg);
    while (!bmp_tx_ready);

    // Write ctrl_meas
    bmp_tx_ready = 0;
    bmp280_write_register(hspi, 0xF4, ctrl_meas_reg);
    while (!bmp_tx_ready);

    // Verify chip ID
    bmp_rx_ready = 0;
    bmp280_read_registers(hspi, 0xD0, 1);  // Read ID register
    while (!bmp_rx_ready);
    if (spi_rx_buf[1] != 0x58) return 0;  // BMP280 ID is 0x58

    // Read calibration registers (0x88 to 0x9F = 24 bytes)
    bmp_rx_ready = 0;
    bmp280_read_registers(hspi, 0x88, 24);
    while (!bmp_rx_ready);

    uint8_t *d = &spi_rx_buf[1];  // skip the first byte (register address echo)

    BMP_Calib->dig_T1 = (uint16_t)(d[1] << 8 | d[0]);
    BMP_Calib->dig_T2 = (int16_t)(d[3] << 8 | d[2]);
    BMP_Calib->dig_T3 = (int16_t)(d[5] << 8 | d[4]);
    BMP_Calib->dig_P1 = (uint16_t)(d[7] << 8 | d[6]);
    BMP_Calib->dig_P2 = (int16_t)(d[9] << 8 | d[8]);
    BMP_Calib->dig_P3 = (int16_t)(d[11] << 8 | d[10]);
    BMP_Calib->dig_P4 = (int16_t)(d[13] << 8 | d[12]);
    BMP_Calib->dig_P5 = (int16_t)(d[15] << 8 | d[14]);
    BMP_Calib->dig_P6 = (int16_t)(d[17] << 8 | d[16]);
    BMP_Calib->dig_P7 = (int16_t)(d[19] << 8 | d[18]);
    BMP_Calib->dig_P8 = (int16_t)(d[21] << 8 | d[20]);
    BMP_Calib->dig_P9 = (int16_t)(d[23] << 8 | d[22]);

    return 1;
}



// temperature compensation using BMP_calib as argument
float BMP280_compensate_temperature(int32_t adc_T, const BMP_calib *calib) {
    float var1, var2, T;

    var1 = ((adc_T / 16384.0f) - (calib->dig_T1 / 1024.0f)) * calib->dig_T2;
    var2 = (((adc_T / 131072.0f) - (calib->dig_T1 / 8192.0f)) *
            ((adc_T / 131072.0f) - (calib->dig_T1 / 8192.0f))) * calib->dig_T3;

    t_fine_float = var1 + var2;
    T = t_fine_float / 5120.0f;

    return T;  // Temperature in °C
}

// pressure compensation using t_fine_float from static scope
float BMP280_compensate_pressure(int32_t adc_P, const BMP_calib *calib) {
    float var1, var2, p;

    var1 = (t_fine_float / 2.0f) - 64000.0f;
    var2 = var1 * var1 * calib->dig_P6 / 32768.0f;
    var2 = var2 + var1 * calib->dig_P5 * 2.0f;
    var2 = (var2 / 4.0f) + (calib->dig_P4 * 65536.0f);
    var1 = (calib->dig_P3 * var1 * var1 / 524288.0f + calib->dig_P2 * var1) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * calib->dig_P1;

    if (var1 == 0.0f) {
        return 0.0f;  // Avoid division by zero
    }

    p = 1048576.0f - adc_P;
    p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
    var1 = calib->dig_P9 * p * p / 2147483648.0f;
    var2 = p * calib->dig_P8 / 32768.0f;

    p = p + (var1 + var2 + calib->dig_P7) / 16.0f;

    return p;  // Pressure in Pa
}

float BMP_calculate_altitude(float pressure) {
    return 44330.0f * (1.0f - powf(pressure / 101325.0, 0.1903f));
}

void update_baro_rate(void) {
    // Make sure BMP_Data.pressure and BMP_Data.temperature are updated already!

    // Calculate altitude from pressure (in meters)
    baro_altitude = BMP_calculate_altitude(BMP_Data.pressure);

    // Calculate dt in seconds (from microseconds)
    float dt = get_dt_us(&last_baro_time) * 1e-6f;  // us -> s

    if (dt > 0) {
        static float last_altitude = 0.0f;
        baro_rate = (baro_altitude - last_altitude) / dt;  // m/s
        last_altitude = baro_altitude;
    }
}


