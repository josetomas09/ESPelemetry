#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "mpu6050.h"
#include "eekf.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sensor_fusion.h"
#include "tpms.h"
#include "datalogger.h"

#define I2C_MASTER_SDA_IO GPIO_NUM_8                    /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO GPIO_NUM_9                    /*!< gpio number for I2C master clock */
#define INPUT_PUSH_BUTTON_GPIO GPIO_NUM_47              /*!< gpio number for push button input */
#define I2C_MASTER_FREQ_HZ 400000                       /*!< I2C master clock frequency (400kHz for Fast-Mode) */

static const char *tpms_tire_names[TPMS_TIRE_COUNT] = {
    [TPMS_FRONT_LEFT]  = "Front Left",
    [TPMS_FRONT_RIGHT] = "Front Right",
    [TPMS_REAR_LEFT]   = "Rear Left",
    [TPMS_REAR_RIGHT]  = "Rear Right",
};

static void i2c_bus_init(void);
static esp_err_t nvs_init(void);

mpu6050_acce_value_t acce_offset;
mpu6050_gyro_value_t gyro_offset;

static const char *TAG = "ESPelemetry";
static mpu6050_handle_t mpu = NULL;


void app_main(void){

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << INPUT_PUSH_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);



    uint8_t push_button_state;
    uint8_t last_push_button_state = 0;
    uint8_t logging_state = 0;
    uint8_t last_logging_state = 0;

    int64_t now_us = 0;
    float dt = 0.0f;
    int64_t last_time_EKF = 0; 
    int64_t last_time_Datalogger = 0;

    tpms_data_t all_tpms[TPMS_TIRE_COUNT];

    datalogger_payload_t payload;

    uint8_t mpu_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

    i2c_bus_init();
    mpu6050_config(mpu, ACCE_FS_2G, GYRO_FS_500DPS);
    
    ESP_ERROR_CHECK(nvs_init());

    ESP_ERROR_CHECK(datalogger_init());
    
    ESP_ERROR_CHECK(mpu6050_wake_up(mpu));
    ESP_ERROR_CHECK(tpms_init());
    
    ESP_ERROR_CHECK(mpu6050_get_deviceid(mpu, &mpu_deviceid));
    ESP_LOGI(TAG, "WHO_AM_I register value: 0x%02X", mpu_deviceid);

    ESP_LOGW(TAG, "Iniciando calibracion. POR FAVOR NO MOVER EL SENSOR...");
    ESP_ERROR_CHECK(mpu6050_calibrate(mpu, 5000));
    ESP_LOGI(TAG, "Calibracion exitosa!");

    
    sensor_fusion_init();

    last_time_EKF = esp_timer_get_time();
    last_time_Datalogger = esp_timer_get_time();

    push_button_state = gpio_get_level(INPUT_PUSH_BUTTON_GPIO);
    last_push_button_state = push_button_state;

    while (1) {

        mpu6050_get_acce(mpu, &acce);
        mpu6050_get_gyro(mpu, &gyro);

        for(uint8_t i = 0 ; i < TPMS_TIRE_COUNT ; i++){
            tpms_get_data(i, &all_tpms[i]);
        }

        now_us = esp_timer_get_time();
        dt = (now_us - last_time_EKF) / 1e6f;
        last_time_EKF = now_us;

        sensor_fusion_update(
            dt,
            acce.acce_x, acce.acce_y, acce.acce_z,
            gyro.gyro_x, gyro.gyro_y, gyro.gyro_z
        );


        payload = (datalogger_payload_t){
            .timestamp = (uint32_t) ((now_us - last_time_Datalogger) / 1000),
            .roll = get_roll(),
            .pitch = get_pitch()
        };
        for(uint8_t i = 0 ; i < TPMS_TIRE_COUNT ; i++){
            payload.tires[i] = all_tpms[i];
        }


        push_button_state = gpio_get_level(INPUT_PUSH_BUTTON_GPIO);

        logging_state = (push_button_state == 1 && last_push_button_state == 0) ? !logging_state : logging_state;

        if(last_logging_state == 0 && logging_state == 1){
            ESP_LOGI(TAG, "Logging session started");

            if(datalogger_init_file_handler() != ESP_OK){
                ESP_LOGE(TAG, "Failed to initialize datalogger file handler");
            }else {
                ESP_LOGI(TAG, "Datalogger file handler initialized successfully");
            }
        }

        if(logging_state == 1){
            if(datalogger_save_on_memory(&payload) != ESP_OK){
                ESP_LOGE(TAG, "Failed to save data logger payload");
            }else{
                ESP_LOGI(TAG, "Data logger payload saved successfully");
            }
        }

        last_push_button_state = push_button_state;
        last_logging_state = logging_state;

        /*
        // Teleplot debug
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

            // Teleplot for debug

            printf(">%s_pressure (PSI):%.2f\n", 
                tpms_tire_names[all_tpms[i].tire], 
                tpms_bar_to_psi(all_tpms[i].pressure_bar)
            );
            printf(">%s_temp:%.2f\n", 
                tpms_tire_names[all_tpms[i].tire], 
                all_tpms[i].temp_c
            );
        
        }
        
        */

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

static esp_err_t nvs_init(void){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize NVS flash: %s", esp_err_to_name(err));
            return err;
        }
    }
    ESP_ERROR_CHECK(err);
    return err;
}