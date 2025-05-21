#pragma once


#include "spi_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define IMU_INT_PINNUM  (gpio_num_t)(14)
#define SPI_MISO_IO     (gpio_num_t)(13)
#define SPI_MOSI_IO     (gpio_num_t)(11)
#define SPI_SCLK_IO     (gpio_num_t)(12)
#define SPI_CS_IO       (gpio_num_t)(10)
#define IMU_CLK_IN_PIN  (gpio_num_t)(9)
#define SPI_FREQ_HZ     (24 * 1000 * 1000)


#ifdef IMU_CLK_IN_PIN
    #include "driver/ledc.h"
    #define CLK_IN_TIMER              LEDC_TIMER_0
    #define CLK_IN_MODE               LEDC_LOW_SPEED_MODE
    #define CLK_IN_OUTPUT_IO          IMU_CLK_IN_PIN
    #define CLK_IN_CHANNEL            LEDC_CHANNEL_0
    #define CLK_IN_DUTY_RES           LEDC_TIMER_8_BIT
    #define CLK_IN_DUTY               (128)
    #define CLK_IN_FREQUENCY          (32000) // 32000Hz
#endif

// 延时函数声明
static inline void delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

static inline void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

int setup_imu(int use_ln, int accel_en, int gyro_en, void (*IMU_IRQ_handler)(void *));
int bsp_IcmGetRawData(float accel_mg[3], float gyro_dps[3], float *temp_degc);