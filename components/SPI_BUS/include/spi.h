#pragma once


#include "spi_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SPI_MISO_IO     (gpio_num_t)(13)
#define SPI_MOSI_IO     (gpio_num_t)(11)
#define SPI_SCLK_IO     (gpio_num_t)(12)
#define SPI_CS_IO       (gpio_num_t)(10)
#define SPI_FREQ_HZ     (24 * 1000 * 1000)

// 延时函数声明
static inline void delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

static inline void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

int setup_imu(int use_ln, int accel_en, int gyro_en);
int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc);