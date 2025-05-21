/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "spi.h"
#include "MahonyAHRS.h"

#define IMU_TICK

#ifdef IMU_INT_PINNUM   
// 中断事件队列句柄
static QueueHandle_t imu_queue = NULL;
static void IRAM_ATTR IMU_IRQ_handler(void* arg);
void IRAM_ATTR IMU_IRQ_process(void *pvParameters);
#endif

float accel_mg[3] = {0};
float gyro_dps[3] = {0};
float temp_degc = 0;

// 四元数到欧拉角 (Z-Y-X 顺规)
// 输入: q0, q1, q2, q3 (四元数 w, x, y, z)
// 输出: roll, pitch, yaw (弧度)
// 将四元数转换为欧拉角
float roll, pitch, yaw;
void QuatToEuler(float q0, float q1, float q2, float q3, float* roll, float* pitch, float* yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1)
        *pitch = copysignf(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
    else
        *pitch = asinf(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

void app_main(void)
{
    imu_queue = xQueueCreate(32, sizeof(bool));
    if (imu_queue == NULL) {
        printf("Failed to create imu_queue\n");
        return;
    }
    // 创建数据处理任务
    xTaskCreatePinnedToCore(IMU_IRQ_process, "imu", 4096, NULL, 15, NULL, 1);
    if(!setup_imu(1,1,1,IMU_IRQ_handler))
        printf("IMU setup done\n");
    else
        printf("IMU setup failed\n");

    uint8_t CPU_RunInfo[400];
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        memset(CPU_RunInfo, 0, 400); /* 信息缓冲区清零 */
 
        vTaskList((char *)&CPU_RunInfo); //获取任务运行时间信息
 
        printf("----------------------------------------------------\r\n");
        printf("task_name     task_status     priority stack task_id\r\n");
        printf("%s", CPU_RunInfo);
        printf("----------------------------------------------------\r\n");
 
        memset(CPU_RunInfo, 0, 400); /* 信息缓冲区清零 */
 
        vTaskGetRunTimeStats((char *)&CPU_RunInfo);

        printf("task_name       run_cnt       usage_rate   \r\n");
        printf("%s", CPU_RunInfo);
        printf("----------------------------------------------------\r\n");
        // bsp_IcmGetRawData(accel_mg, gyro_dps,&temp_degc);
        // printf("accel_mg: %f %f %f\n", accel_mg[0], accel_mg[1], accel_mg[2]);
        // printf("gyro_dps: %f %f %f\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        printf("temp_degc: %f\n", temp_degc);
         printf("Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", roll * 180.0f/M_PI, pitch * 180.0f/M_PI, yaw * 180.0f/M_PI);
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
    uint64_t current_timestamp;
    float dt; // 时间间隔，单位秒

    uint64_t last_timestamp = esp_timer_get_time();

    while (1)
    {
        xQueueReceive(imu_queue, &res, portMAX_DELAY);
        if (res)
        {
            bsp_IcmGetRawData(accel_mg, gyro_dps,&temp_degc);

            current_timestamp = esp_timer_get_time();
            dt = (float)(current_timestamp - last_timestamp) / 1000000.0f;
            last_timestamp = current_timestamp;

            if (dt > 0 && dt < 0.1f) {
                 // 将 dps 转换为 rad/s
                 float gx_rads = gyro_dps[0] * M_PI / 180.0f;
                 float gy_rads = gyro_dps[1] * M_PI / 180.0f;
                 float gz_rads = gyro_dps[2] * M_PI / 180.0f;

                 // 将 mg 转换为 g
                 float ax_g = accel_mg[0] / 1000.0f;
                 float ay_g = accel_mg[1] / 1000.0f;
                 float az_g = accel_mg[2] / 1000.0f;

                 // 调用 MahonyAHRS 更新函数 (根据头文件，不传入 dt)
                MahonyAHRSupdateIMU(gx_rads, gy_rads, gz_rads, ax_g, ay_g, az_g);
                QuatToEuler(q0, q1, q2, q3, &roll, &pitch, &yaw);

                // 将弧度转换为度并打印
                // printf("Roll=%.2f, Pitch=%.2f, Yaw=%.2f\n", roll * 180.0f/M_PI, pitch * 180.0f/M_PI, yaw * 180.0f/M_PI);
            }
        }
    }
}
#endif

