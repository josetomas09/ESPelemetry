#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "mpu6050.h"
#include "eekf.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sensor_fusion.h"
#include "tpms.h"

#define I2C_MASTER_SDA_IO 8             /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO 9             /*!< gpio number for I2C master clock */
#define I2C_MASTER_FREQ_HZ 400000       /*!< I2C master clock frequency (400kHz for Fast-Mode) */

static void i2c_bus_init(void);

mpu6050_acce_value_t acce_offset;
mpu6050_gyro_value_t gyro_offset;

tpms_data_t tpms_data;

static const char *TAG = "ESPelemetry";
static mpu6050_handle_t mpu = NULL;

static int64_t last_time_us = 0;

void app_main(void){

    uint8_t mpu_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    i2c_bus_init();

    mpu6050_config(mpu, ACCE_FS_2G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu);
    tpms_init();

    ESP_ERROR_CHECK(mpu6050_get_deviceid(mpu, &mpu_deviceid));
    ESP_LOGI(TAG, "WHO_AM_I register value: 0x%02X", mpu_deviceid);

    ESP_LOGW(TAG, "Iniciando calibracion. POR FAVOR NO MOVER EL SENSOR...");
    ESP_ERROR_CHECK(mpu6050_calibrate(mpu, 5000));
    ESP_LOGI(TAG, "Calibracion exitosa!");

    sensor_fusion_init();
    last_time_us = esp_timer_get_time();


    while (1) {
        mpu6050_get_acce(mpu, &acce);
        mpu6050_get_gyro(mpu, &gyro);
        mpu6050_get_temp(mpu, &temp);

        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_time_us) / 1e6f;
        last_time_us = now_us;

        sensor_fusion_update(
            dt,
            acce.acce_x, acce.acce_y, acce.acce_z,
            gyro.gyro_x, gyro.gyro_y, gyro.gyro_z
        );

        ESP_LOGI(TAG, "Temp: %.2f °C", temp.temp);
        ESP_LOGI(TAG, "Acce (g)   X: %.2f \t Y: %.2f \t Z: %.2f", acce.acce_x, acce.acce_y, acce.acce_z);
        ESP_LOGI(TAG, "Gyro (dps) X: %.2f \t Y: %.2f \t Z: %.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
        ESP_LOGI(TAG, "Fused (deg) Roll: %.2f \t Pitch: %.2f \t Yaw: %.2f\n", get_roll(), get_pitch(), get_yaw());

        vTaskDelay(pdMS_TO_TICKS(500));
    }

}

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void){
    i2c_master_bus_config_t bus_conf = {
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .flags.enable_internal_pullup = 1,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
    };
    i2c_master_bus_handle_t bus_handle;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = MPU6050_I2C_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    mpu = mpu6050_create(dev_handle);
}