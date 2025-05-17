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
#define IMU_INT_PINNUM  (gpio_num_t)(14)
float accel_mg[3] = {0};
float gyro_dps[3] = {0};
float temp_degc = 0;
void app_main(void)
{
    if(!setup_imu(1,1,1))
        printf("IMU setup done\n");
    else
        printf("IMU setup failed\n");
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        bsp_IcmGetRawData(accel_mg, gyro_dps,&temp_degc);
        // printf("accel_mg: %f %f %f\n", accel_mg[0], accel_mg[1], accel_mg[2]);
        // printf("gyro_dps: %f %f %f\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        printf("temp_degc: %f\n", temp_degc);
    }
}
