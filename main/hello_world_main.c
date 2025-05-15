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
#include "spi_bus.h"
#include "icm42688.h"
#include "esp_timer.h"
#define SPI_MISO_IO     (gpio_num_t)(13)
#define SPI_MOSI_IO     (gpio_num_t)(11)
#define SPI_SCLK_IO     (gpio_num_t)(12)
#define SPI_CS_IO       (gpio_num_t)(10)
#define IMU_INT_PINNUM  (gpio_num_t)(14)
#define SPI_FREQ_HZ     (10 * 1000 * 1000)
spi_bus_device_handle_t spi_device_handle;//deivce


volatile uint8_t flag0;
volatile uint64_t start = 0;
volatile uint64_t end = 0;
volatile uint64_t time = 0;

float GyroCorrected[3];
float AccelCorrected[3];
float GyroCal[3]={0};//陀螺仪校准值
float AccelCal[3]={0};//陀螺仪校准值
float icm42688_acc_x, icm42688_acc_y, icm42688_acc_z  ;// ICM42688加速度原始数据       
float icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z ; // ICM42688角速度原始速度数据
float LSB_ACC_GYRO[2]={0};//陀螺仪数据转换变量
//FUNCTION PROTOTYPES
void icm42688_write_single_byte(uint8_t reg, uint8_t data);
uint8_t icm42688_read_single_byte(uint8_t reg);
void Init_icm42688(void);
void IRAM_ATTR Get_Gyro_ICM42688(void);
void IRAM_ATTR Get_Acc_ICM42688(void);
// 中断事件队列句柄
static QueueHandle_t button_queue = NULL;
/**
 * @brief ICM42688中断处理函数
 * 当传感器产生中断时被调用
 * 向队列发送事件通知数据处理任务
 */
static void IRAM_ATTR IMU_IRQ_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult;
    bool ready = true;
    xResult = xQueueSendFromISR(button_queue, &ready, &xHigherPriorityTaskWoken);
    // 如果有高优先级任务等待此事件,则进行任务切换
    if (xResult == pdPASS) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    };
}
void IRAM_ATTR IMU_IRQ_process(void *pvParameters)
{
    bool res;
    while (1)
    {
        // 等待中断事件
        xQueueReceive(button_queue, &res, portMAX_DELAY);
        if (res)
        {
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
            /*获取ICM42688的加速度原始值*/
            Get_Gyro_ICM42688();
            Get_Acc_ICM42688();
            GyroCorrected[0] = ((float)icm42688_gyro_x + GyroCal[0]) * LSB_ACC_GYRO[1];
            GyroCorrected[1] = ((float)icm42688_gyro_y + GyroCal[1]) * LSB_ACC_GYRO[1];//对原始数据进行校准,并且转换为标准数据
            GyroCorrected[2] = ((float)icm42688_gyro_z + GyroCal[2]) * LSB_ACC_GYRO[1];	
            AccelCorrected[0] = ((float)icm42688_acc_x + AccelCal[0]) * LSB_ACC_GYRO[0];
            AccelCorrected[1] = ((float)icm42688_acc_y + AccelCal[1]) * LSB_ACC_GYRO[0];
            AccelCorrected[2] = ((float)icm42688_acc_z + AccelCal[2]) * LSB_ACC_GYRO[0];
            // printf("Gyro: %f %f %f\n", GyroCorrected[0], GyroCorrected[1], GyroCorrected[2]);
            // printf("Acc: %f %f %f\n", AccelCorrected[0], AccelCorrected[1], AccelCorrected[2]);
        }
    }
}
void app_main(void)
{
    spi_config_t bus_conf = {
        .miso_io_num = SPI_MISO_IO,
        .mosi_io_num = SPI_MOSI_IO,
        .sclk_io_num = SPI_SCLK_IO,
    };
    spi_bus_handle_t spi_bus_handle = spi_bus_create(SPI2_HOST, &bus_conf);
    spi_device_config_t device_conf = {
        .cs_io_num = SPI_CS_IO,
        .mode = 0,
        .clock_speed_hz = SPI_FREQ_HZ,
    };
    spi_device_handle = spi_bus_device_create(spi_bus_handle, &device_conf);
    // 创建中断事件队列
    button_queue = xQueueCreate(32, sizeof(bool));
    
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
    Init_icm42688();
    /*LSB设置*/
	LSB_ACC_GYRO[0] = LSB_ACC_16G;
	LSB_ACC_GYRO[1] = LSB_GYRO_2000_R;
    // 读取传感器ID
    uint8_t id = icm42688_read_single_byte(0x75);
    printf("ICM42688 ID: 0x%02X\n", id);
    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Time: %llu us\n", time);
        // printf("Hello world again!\n");

        // /*获取ICM42688的加速度原始值*/
        // Get_Gyro_ICM42688();
        // Get_Acc_ICM42688();
        // GyroCorrected[0] = ((float)icm42688_gyro_x + GyroCal[0]) * LSB_ACC_GYRO[1];
        // GyroCorrected[1] = ((float)icm42688_gyro_y + GyroCal[1]) * LSB_ACC_GYRO[1];//对原始数据进行校准,并且转换为标准数据
        // GyroCorrected[2] = ((float)icm42688_gyro_z + GyroCal[2]) * LSB_ACC_GYRO[1];	
        // AccelCorrected[0] = ((float)icm42688_acc_x + AccelCal[0]) * LSB_ACC_GYRO[0];
        // AccelCorrected[1] = ((float)icm42688_acc_y + AccelCal[1]) * LSB_ACC_GYRO[0];
        // AccelCorrected[2] = ((float)icm42688_acc_z + AccelCal[2]) * LSB_ACC_GYRO[0];
        printf("Gyro: %f %f %f\n", GyroCorrected[0], GyroCorrected[1], GyroCorrected[2]);
        printf("Acc: %f %f %f\n", AccelCorrected[0], AccelCorrected[1], AccelCorrected[2]);
    }
}

void IRAM_ATTR Get_Acc_ICM42688(void)
{
    unsigned char dat[7]={0};
    unsigned char dat_out[7]={0};
    dat_out[0] = 0x80|ICM42688_ACCEL_DATA_X1;
    spi_bus_transfer_bytes(spi_device_handle,(const uint8_t *)dat_out ,&dat[1] , 7);
    icm42688_acc_x = (short int)(((short int)dat[1] << 8) | dat[2]);
    icm42688_acc_y = (short int)(((short int)dat[3] << 8) | dat[4]);
    icm42688_acc_z = (short int)(((short int)dat[5] << 8) | dat[6]);
}
/*获取ICM42688的角速度原始值*/
void IRAM_ATTR Get_Gyro_ICM42688(void)
{
    unsigned char dat[7]={0};
    unsigned char dat_out[7]={0};
    dat_out[0] = 0x80|ICM42688_GYRO_DATA_X1;
    spi_bus_transfer_bytes(spi_device_handle,(const uint8_t *)dat_out ,&dat[1] , 7);
    icm42688_gyro_x = (short int)(((short int)dat[1] << 8) | dat[2]);
    icm42688_gyro_y = (short int)(((short int)dat[3] << 8) | dat[4]);
    icm42688_gyro_z = (short int)(((short int)dat[5] << 8) | dat[6]);
}

uint8_t icm42688_read_single_byte(uint8_t reg)
{
    uint8_t data_out[2] = {0x80|reg, 0x00};
    uint8_t data_in[2] = {0x00, 0x00};
    spi_bus_transfer_bytes(spi_device_handle, data_out, data_in, 2);
    return data_in[1];
}

void icm42688_write_single_byte(uint8_t reg, uint8_t data)
{
    uint8_t data_out[2] = {reg, data};
    spi_bus_transfer_bytes(spi_device_handle, data_out, NULL, 2);
}

void Init_icm42688(void)
{
	/*指定Bank0*/
	icm42688_write_single_byte(0x76,0x00);
	/*软重启*/
	icm42688_write_single_byte(0x11,0x01);

	vTaskDelay(30);  
	/*指定Bank0*/
	icm42688_write_single_byte(0x76,0x00);
	/*Gyro设置*/
	icm42688_write_single_byte(0x4F,0x06);//2000dps 1KHz
	/*Accel设置*/
	icm42688_write_single_byte(0x50,0x06);//16G 1KHz
	/*电源管理*/
	icm42688_write_single_byte(0x4E,0x0F);//ACC GYRO LowNoise Mode
	
	vTaskDelay(30);      
	/*指定Bank0*/
	icm42688_write_single_byte(0x76,0x00);
	/*中断输出设置*/
	icm42688_write_single_byte(0x14,0x30);//INT1 INT2 脉冲模式，低有效
	/*Gyro设置*/
	// icm42688_write_single_byte(0x4F,0x06);//2000dps 1KHz
    icm42688_write_single_byte(0x4F,0x03);//2000dps 8KHz
	/*Accel设置*/
	// icm42688_write_single_byte(0x50,0x06);//16G 1KHz
    icm42688_write_single_byte(0x50,0x03);//16G 8KHz
	/*LSB设置*/
	// LSB_ACC_GYRO[0] = LSB_ACC_16G;
	// LSB_ACC_GYRO[1] = LSB_GYRO_2000_R;
	/*Tem设置&Gyro_Config1*/
	icm42688_write_single_byte(0x51,0x56);//BW 82Hz Latency = 2ms
	/*GYRO_ACCEL_CONFIG0*/
	icm42688_write_single_byte(0x52,0x11);//1BW
	/*ACCEL_CONFIG1*/
	icm42688_write_single_byte(0x53,0x0D);//Null
	/*INT_CONFIG0*/
	icm42688_write_single_byte(0x63,0x00);//Null
	/*INT_CONFIG1*/
	icm42688_write_single_byte(0x64,0x60);//中断引脚正常启用 8us 大于8KHz时设定
	/*INT_SOURCE0*/
	icm42688_write_single_byte(0x65,0x08);//DRDY INT1
	/*INT_SOURCE1*/
	icm42688_write_single_byte(0x66,0x00);//Null
	/*INT_SOURCE3*/
	icm42688_write_single_byte(0x68,0x00);//Null
	/*INT_SOURCE3*/
	icm42688_write_single_byte(0x69,0x00);//Null
	
/*****抗混叠滤波器@536Hz*****/
	
	/*GYRO抗混叠滤波器配置*/
	/*指定Bank1*/
	icm42688_write_single_byte(0x76,0x01);
	/*GYRO抗混叠滤波器配置*/
	icm42688_write_single_byte(0x0B,0xA0);//开启抗混叠和陷波滤波器
	icm42688_write_single_byte(0x0C,0x0C);//GYRO_AAF_DELT 12 (default 13)
	icm42688_write_single_byte(0x0D,0x90);//GYRO_AAF_DELTSQR 144 (default 170)
	icm42688_write_single_byte(0x0E,0x80);//GYRO_AAF_BITSHIFT 8 (default 8)
	
	/*ACCEL抗混叠滤波器配置*/
	/*指定Bank2*/
	icm42688_write_single_byte(0x76,0x02);
	/*ACCEL抗混叠滤波器配置*/
	icm42688_write_single_byte(0x03,0x18);//开启滤波器 ACCEL_AFF_DELT 12 (default 24)
	icm42688_write_single_byte(0x04,0x90);//ACCEL_AFF_DELTSQR 144 (default 64)
	icm42688_write_single_byte(0x05,0x80);//ACCEL_AAF_BITSHIFT 8 (default 6)

/*****自定义滤波器1号@111Hz*****/

	/*指定Bank0*/
	icm42688_write_single_byte(0x76,0x00);
	/*滤波器顺序*/
	icm42688_write_single_byte(0x51,0x12);//GYRO滤波器1st
	icm42688_write_single_byte(0x53,0x05);//ACCEL滤波器1st
	/*滤波器设置*/
	icm42688_write_single_byte(0x52,0x33);//111Hz 03
	/*指定Bank0*/
	icm42688_write_single_byte(0x76,0x00);
	/*电源管理*/
	icm42688_write_single_byte(0x4E,0x0F);//ACC GYRO LowNoise Mode
	vTaskDelay(50);
}