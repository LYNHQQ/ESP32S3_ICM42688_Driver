/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "spi.h"

#define IMU_CLK_IN_PIN  (gpio_num_t)(9)
#define IMU_INT_PINNUM  (gpio_num_t)(14)
#define IMU_TICK

#ifdef IMU_CLK_IN_PIN
    #include "driver/ledc.h"
    #define CLK_IN_TIMER              LEDC_TIMER_0
    #define CLK_IN_MODE               LEDC_LOW_SPEED_MODE
    #define CLK_IN_OUTPUT_IO          IMU_CLK_IN_PIN // Define the output GPIO
    #define CLK_IN_CHANNEL            LEDC_CHANNEL_0
    #define CLK_IN_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 13 bits
    #define CLK_IN_DUTY               (128) // Set duty to 50%. (2 ^ 8) * 50% = 128
    #define CLK_IN_FREQUENCY          (32000) // Frequency in Hertz. Set frequency at 32 kHz
    static void clk_in_init(void);
#endif

#ifdef IMU_INT_PINNUM   
    #ifdef IMU_TICK
        volatile uint8_t flag0;
        volatile uint64_t start = 0;
        volatile uint64_t end = 0;
        volatile uint64_t time = 0;
    #endif
// 中断事件队列句柄
static QueueHandle_t imu_queue = NULL;
static void IRAM_ATTR IMU_IRQ_handler(void* arg);
void IRAM_ATTR IMU_IRQ_process(void *pvParameters);
#endif

float accel_mg[3] = {0};
float gyro_dps[3] = {0};
float temp_degc = 0;

void app_main(void)
{

    if(!setup_imu(1,1,1))
        printf("IMU setup done\n");
    else
        printf("IMU setup failed\n");
#ifdef IMU_INT_PINNUM
    imu_queue = xQueueCreate(32, sizeof(bool));
    // 创建数据处理任务
    xTaskCreatePinnedToCore(IMU_IRQ_process, "imu", 4096, NULL, 15, NULL, 1);
    // 配置中断输入引脚
    gpio_set_direction(IMU_INT_PINNUM, GPIO_MODE_INPUT);
    gpio_set_pull_mode(IMU_INT_PINNUM, GPIO_PULLUP_ONLY);
    // 安装GPIO中断服务
    gpio_install_isr_service(0);
    // 注册中断处理函数
    gpio_isr_handler_add(IMU_INT_PINNUM, IMU_IRQ_handler, NULL);
    // 配置为下降沿触发中断
    gpio_set_intr_type(IMU_INT_PINNUM, GPIO_INTR_POSEDGE);
    // 使能中断
    gpio_intr_enable(IMU_INT_PINNUM);
#endif
#ifdef IMU_CLK_IN_PIN
    clk_in_init();
    ledc_set_duty(CLK_IN_MODE, CLK_IN_CHANNEL, CLK_IN_DUTY);
    ledc_update_duty(CLK_IN_MODE, CLK_IN_CHANNEL);
#endif
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        // bsp_IcmGetRawData(accel_mg, gyro_dps,&temp_degc);
        printf("accel_mg: %f %f %f\n", accel_mg[0], accel_mg[1], accel_mg[2]);
        printf("gyro_dps: %f %f %f\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        printf("temp_degc: %f\n", temp_degc);

#ifdef IMU_TICK
        printf("Time: %llu us\n", time);
#endif
    }
}

#ifdef IMU_INT_PINNUM
static void IMU_IRQ_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult;
    bool ready = true;
    xResult = xQueueSendFromISR(imu_queue, &ready, &xHigherPriorityTaskWoken);
    // 如果有高优先级任务等待此事件,则进行任务切换
    if (xResult == pdPASS) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    };
}
void IMU_IRQ_process(void *pvParameters)
{
    bool res;
    while (1)
    {
        // 等待中断事件
        xQueueReceive(imu_queue, &res, portMAX_DELAY);
        if (res)
        {
            #ifdef IMU_TICK
            if(flag0)
            {
                start = esp_timer_get_time();
                flag0 = 0;
            }
            else
            {
                end = esp_timer_get_time();
                time = end - start;
                flag0 = 1;
            } 
            #endif
            /*获取ICM45686*/
            bsp_IcmGetRawData(accel_mg, gyro_dps,&temp_degc);
        }
    }
}
#endif

#ifdef IMU_CLK_IN_PIN
static void clk_in_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = CLK_IN_MODE,
        .duty_resolution  = CLK_IN_DUTY_RES,
        .timer_num        = CLK_IN_TIMER,
        .freq_hz          = CLK_IN_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = CLK_IN_MODE,
        .channel        = CLK_IN_CHANNEL,
        .timer_sel      = CLK_IN_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = IMU_CLK_IN_PIN,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}
#endif