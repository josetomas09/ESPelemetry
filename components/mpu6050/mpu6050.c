#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "mpu6050.h"

#define ALPHA                   0.99f           /*!< Weight of gyroscope */
#define RAD_TO_DEG              57.27272727f    /*!< Radians to degrees */


const uint8_t MPU6050_DATA_RDY_INT_BIT =      (uint8_t) BIT0;
const uint8_t MPU6050_I2C_MASTER_INT_BIT =    (uint8_t) BIT3;
const uint8_t MPU6050_FIFO_OVERFLOW_INT_BIT = (uint8_t) BIT4;
const uint8_t MPU6050_MOT_DETECT_INT_BIT =    (uint8_t) BIT6;
const uint8_t MPU6050_ALL_INTERRUPTS = (MPU6050_DATA_RDY_INT_BIT | MPU6050_I2C_MASTER_INT_BIT | MPU6050_FIFO_OVERFLOW_INT_BIT | MPU6050_MOT_DETECT_INT_BIT);

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    gpio_num_t              int_pin;
    uint32_t                counter;
    float                        dt;    /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval           *timer;
    mpu6050_acce_value_t acce_offset;
    mpu6050_gyro_value_t gyro_offset;
} mpu6050_dev_t;


static esp_err_t mpu6050_write(mpu6050_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len){
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    
    uint8_t write_buf[data_len + 1];
    write_buf[0] = reg_start_addr;
    memcpy(write_buf + 1, data_buf, data_len);

    return i2c_master_transmit(sens->i2c_dev, write_buf, sizeof(write_buf), -1);
}


static esp_err_t mpu6050_read(mpu6050_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len){
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    
    return i2c_master_transmit_receive(sens->i2c_dev, &reg_start_addr, 1, data_buf, data_len, -1);
}


mpu6050_handle_t mpu6050_create(i2c_master_dev_handle_t dev_handle){
    mpu6050_dev_t *sensor = (mpu6050_dev_t *) calloc(1, sizeof(mpu6050_dev_t));
    if (sensor == NULL) {
        return NULL;
    }
    sensor->i2c_dev = dev_handle;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (mpu6050_handle_t) sensor;
}

void mpu6050_delete(mpu6050_handle_t sensor){
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    free(sens);
}

esp_err_t mpu6050_get_deviceid(mpu6050_handle_t sensor, uint8_t *const deviceid){
    return mpu6050_read(sensor, MPU6050_WHO_AM_I, deviceid, 1);
}

esp_err_t mpu6050_wake_up(mpu6050_handle_t sensor){
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp &= (~BIT6);
    ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_sleep(mpu6050_handle_t sensor){
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp |= BIT6;
    ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_config(mpu6050_handle_t sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs){
    uint8_t config_regs[2] = {gyro_fs << 3,  acce_fs << 3};
    return mpu6050_write(sensor, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
}

esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t sensor, float *const acce_sensitivity){
    esp_err_t ret;
    uint8_t acce_fs;
    ret = mpu6050_read(sensor, MPU6050_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs) {
    case ACCE_FS_2G:
        *acce_sensitivity = 16384;
        break;

    case ACCE_FS_4G:
        *acce_sensitivity = 8192;
        break;

    case ACCE_FS_8G:
        *acce_sensitivity = 4096;
        break;

    case ACCE_FS_16G:
        *acce_sensitivity = 2048;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t sensor, float *const gyro_sensitivity){
    esp_err_t ret;
    uint8_t gyro_fs;
    ret = mpu6050_read(sensor, MPU6050_GYRO_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs) {
    case GYRO_FS_250DPS:
        *gyro_sensitivity = 131;
        break;

    case GYRO_FS_500DPS:
        *gyro_sensitivity = 65.5;
        break;

    case GYRO_FS_1000DPS:
        *gyro_sensitivity = 32.8;
        break;

    case GYRO_FS_2000DPS:
        *gyro_sensitivity = 16.4;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_config_interrupts(mpu6050_handle_t sensor, const mpu6050_int_config_t *const interrupt_configuration){
    esp_err_t ret = ESP_OK;

    if (NULL == interrupt_configuration) {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    if (GPIO_IS_VALID_GPIO(interrupt_configuration->interrupt_pin)) {
        // Set GPIO connected to MPU6050 INT pin only when user configures interrupts.
        mpu6050_dev_t *sensor_device = (mpu6050_dev_t *) sensor;
        sensor_device->int_pin = interrupt_configuration->interrupt_pin;
    } else {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    uint8_t int_pin_cfg = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
        int_pin_cfg |= BIT7;
    }

    if (INTERRUPT_PIN_OPEN_DRAIN == interrupt_configuration->pin_mode) {
        int_pin_cfg |= BIT6;
    }

    if (INTERRUPT_LATCH_UNTIL_CLEARED == interrupt_configuration->interrupt_latch) {
        int_pin_cfg |= BIT5;
    }

    if (INTERRUPT_CLEAR_ON_ANY_READ == interrupt_configuration->interrupt_clear_behavior) {
        int_pin_cfg |= BIT4;
    }

    ret = mpu6050_write(sensor, MPU6050_INTR_PIN_CFG, &int_pin_cfg, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    gpio_int_type_t gpio_intr_type;

    if (INTERRUPT_PIN_ACTIVE_LOW == interrupt_configuration->active_level) {
        gpio_intr_type = GPIO_INTR_NEGEDGE;
    } else {
        gpio_intr_type = GPIO_INTR_POSEDGE;
    }

    gpio_config_t int_gpio_config = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = gpio_intr_type,
        .pin_bit_mask = (BIT0 << interrupt_configuration->interrupt_pin)
    };

    ret = gpio_config(&int_gpio_config);

    return ret;
}

esp_err_t mpu6050_register_isr(mpu6050_handle_t sensor, const mpu6050_isr_t isr){
    esp_err_t ret;
    mpu6050_dev_t *sensor_device = (mpu6050_dev_t *) sensor;

    if (NULL == sensor_device) {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    ret = gpio_isr_handler_add(
              sensor_device->int_pin,
              ((gpio_isr_t) * (isr)),
              ((void *) sensor)
          );

    if (ESP_OK != ret) {
        return ret;
    }

    ret = gpio_intr_enable(sensor_device->int_pin);

    return ret;
}

esp_err_t mpu6050_enable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources){
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    if (enabled_interrupts != interrupt_sources) {

        enabled_interrupts |= interrupt_sources;

        ret = mpu6050_write(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);
    }

    return ret;
}

esp_err_t mpu6050_disable_interrupts(mpu6050_handle_t sensor, uint8_t interrupt_sources){
    esp_err_t ret;
    uint8_t enabled_interrupts = 0x00;

    ret = mpu6050_read(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);

    if (ESP_OK != ret) {
        return ret;
    }

    if (0 != (enabled_interrupts & interrupt_sources)) {
        enabled_interrupts &= (~interrupt_sources);

        ret = mpu6050_write(sensor, MPU6050_INTR_ENABLE, &enabled_interrupts, 1);
    }

    return ret;
}

esp_err_t mpu6050_get_interrupt_status(mpu6050_handle_t sensor, uint8_t *const out_intr_status){
    esp_err_t ret;

    if (NULL == out_intr_status) {
        ret = ESP_ERR_INVALID_ARG;
        return ret;
    }

    ret = mpu6050_read(sensor, MPU6050_INTR_STATUS, out_intr_status, 1);

    return ret;
}

inline uint8_t mpu6050_is_data_ready_interrupt(uint8_t interrupt_status){
    return (MPU6050_DATA_RDY_INT_BIT == (MPU6050_DATA_RDY_INT_BIT & interrupt_status));
}

inline uint8_t mpu6050_is_i2c_master_interrupt(uint8_t interrupt_status){
    return (uint8_t) (MPU6050_I2C_MASTER_INT_BIT == (MPU6050_I2C_MASTER_INT_BIT & interrupt_status));
}

inline uint8_t mpu6050_is_fifo_overflow_interrupt(uint8_t interrupt_status){
    return (uint8_t) (MPU6050_FIFO_OVERFLOW_INT_BIT == (MPU6050_FIFO_OVERFLOW_INT_BIT & interrupt_status));
}

esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t sensor, mpu6050_raw_acce_value_t *const raw_acce_value){
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

    esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value){
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd));

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));

    return ret;
}

esp_err_t mpu6050_get_acce(mpu6050_handle_t sensor, mpu6050_acce_value_t *const acce_value){
    esp_err_t ret;
    float acce_sensitivity;
    mpu6050_raw_acce_value_t raw_acce;
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;

    ret = mpu6050_get_acce_sensitivity(sensor, &acce_sensitivity);
    if (ret != ESP_OK) return ret;
    
    ret = mpu6050_get_raw_acce(sensor, &raw_acce);
    if (ret != ESP_OK) return ret;

    acce_value->acce_x = (raw_acce.raw_acce_x / acce_sensitivity) - sens->acce_offset.acce_x;
    acce_value->acce_y = (raw_acce.raw_acce_y / acce_sensitivity) - sens->acce_offset.acce_y;
    acce_value->acce_z = (raw_acce.raw_acce_z / acce_sensitivity) - sens->acce_offset.acce_z;
    
    return ESP_OK;
}

esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_gyro_value_t *const gyro_value){
    esp_err_t ret;
    float gyro_sensitivity;
    mpu6050_raw_gyro_value_t raw_gyro;
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;

    ret = mpu6050_get_gyro_sensitivity(sensor, &gyro_sensitivity);
    if (ret != ESP_OK) return ret;
    
    ret = mpu6050_get_raw_gyro(sensor, &raw_gyro);
    if (ret != ESP_OK) return ret;

    gyro_value->gyro_x = (raw_gyro.raw_gyro_x / gyro_sensitivity) - sens->gyro_offset.gyro_x;
    gyro_value->gyro_y = (raw_gyro.raw_gyro_y / gyro_sensitivity) - sens->gyro_offset.gyro_y;
    gyro_value->gyro_z = (raw_gyro.raw_gyro_z / gyro_sensitivity) - sens->gyro_offset.gyro_z;
    
    return ESP_OK;
}

esp_err_t mpu6050_get_temp(mpu6050_handle_t sensor, mpu6050_temp_value_t *const temp_value){
    uint8_t data_rd[2];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_TEMP_XOUT_H, data_rd, sizeof(data_rd));
    temp_value->temp = (int16_t)((data_rd[0] << 8) | (data_rd[1])) / 340.00 + 36.53;
    return ret;
}

esp_err_t mpu6050_calibrate(mpu6050_handle_t sensor, uint16_t num_samples){
    if (sensor == NULL){
        return ESP_ERR_INVALID_ARG;
    };

    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;

    sens->acce_offset.acce_x = 0; sens->acce_offset.acce_y = 0; sens->acce_offset.acce_z = 0;
    sens->gyro_offset.gyro_x = 0; sens->gyro_offset.gyro_y = 0; sens->gyro_offset.gyro_z = 0;

    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;

    mpu6050_acce_value_t temp_acce;
    mpu6050_gyro_value_t temp_gyro;

    // Discard 50 unstable readings
    for(int i = 0; i < 50; i++){
        mpu6050_get_acce(sensor, &temp_acce);
        mpu6050_get_gyro(sensor, &temp_gyro);
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }


    for(int i = 0; i < num_samples; i++){
        mpu6050_get_acce(sensor, &temp_acce);
        mpu6050_get_gyro(sensor, &temp_gyro);
        ax_sum += temp_acce.acce_x; ay_sum += temp_acce.acce_y; az_sum += temp_acce.acce_z;
        gx_sum += temp_gyro.gyro_x; gy_sum += temp_gyro.gyro_y; gz_sum += temp_gyro.gyro_z;
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }

    sens->acce_offset.acce_x = ax_sum / num_samples;
    sens->acce_offset.acce_y = ay_sum / num_samples;
    sens->acce_offset.acce_z = az_sum / num_samples;
    
    sens->gyro_offset.gyro_x = gx_sum / num_samples;
    sens->gyro_offset.gyro_y = gy_sum / num_samples;
    sens->gyro_offset.gyro_z = gz_sum / num_samples;

    if (sens->acce_offset.acce_z > 0) {
        sens->acce_offset.acce_z -= 1.0f;
    } else {
        sens->acce_offset.acce_z += 1.0f;
    }

    return ESP_OK;
}