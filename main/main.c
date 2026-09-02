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
#include "datalogger.h"

#define I2C_MASTER_SDA_IO 8             /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO 9             /*!< gpio number for I2C master clock */
#define I2C_MASTER_FREQ_HZ 400000       /*!< I2C master clock frequency (400kHz for Fast-Mode) */

static const char *tpms_tire_names[TPMS_TIRE_COUNT] = {
    [TPMS_FRONT_LEFT]  = "Front Left",
    [TPMS_FRONT_RIGHT] = "Front Right",
    [TPMS_REAR_LEFT]   = "Rear Left",
    [TPMS_REAR_RIGHT]  = "Rear Right",
};

static void i2c_bus_init(void);

mpu6050_acce_value_t acce_offset;
mpu6050_gyro_value_t gyro_offset;

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

    ESP_ERROR_CHECK(mpu6050_wake_up(mpu));
    ESP_ERROR_CHECK(tpms_init());
    tpms_data_t all_tpms[TPMS_TIRE_COUNT];

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

        
        for(uint8_t i = 0 ; i < TPMS_TIRE_COUNT ; i++){
            tpms_get_data(i, &all_tpms[i]);
        }

        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_time_us) / 1e6f;
        last_time_us = now_us;

        sensor_fusion_update(
            dt,
            acce.acce_x, acce.acce_y, acce.acce_z,
            gyro.gyro_x, gyro.gyro_y, gyro.gyro_z
        );


        ESP_LOGI(TAG, "--- Telemetry Update ---");
        ESP_LOGI(TAG, "IMU  | Temp: %5.2f °C | Roll: %6.2f | Pitch: %6.2f | Yaw: %6.2f", 
            temp.temp, 
            get_roll(), 
            get_pitch(), 
            get_yaw()
        );

        //Teleplot
        printf(">ChassisTilt:%.2f:%.2f\n", get_roll(), get_pitch());
        printf(">Lat_G:%.2f\n", acce.acce_x); 
        printf(">Long_G:%.2f\n", acce.acce_y);
        printf(">GForceMeter:%.2f:%.2f\n", acce.acce_x, acce.acce_y);

        for(uint8_t i = 0 ; i < TPMS_TIRE_COUNT ; i++){
            ESP_LOGI(TAG, "TPMS | %-11s : %4.2f Bar | %5.2f °C %.2f secs", 
                tpms_tire_names[all_tpms[i].tire], 
                all_tpms[i].pressure_bar, 
                all_tpms[i].temp_c,
                dt
            );

            //teleplot

            printf(">%s_pressure (PSI):%.2f\n", 
                tpms_tire_names[all_tpms[i].tire], 
                tpms_bar_to_psi(all_tpms[i].pressure_bar)
            );
            printf(">%s_temp:%.2f\n", 
                tpms_tire_names[all_tpms[i].tire], 
                all_tpms[i].temp_c
            );
        
        }

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