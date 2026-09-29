#include "datalogger.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "errno.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sys/stat.h"
#include "inttypes.h"
#include "tinyusb.h"
#include "tinyusb_msc.h"
#include "tinyusb_default_config.h"

static char const *TAG = "DataLogger";

static uint8_t session_count;
static nvs_handle_t nvs_datalogger_handler;
static wl_handle_t wl_datalogger_handler;

char data_logger_file_name[256];
char log_file_name[256];


static esp_err_t storage_init_spiflash(wl_handle_t *wl_datalogger_handler){

    esp_partition_t const *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "data_logger");
    
    if(partition == NULL){
        return ESP_FAIL;
    }else if(wl_mount(partition, wl_datalogger_handler) != ESP_OK){
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t datalogger_save_on_memory(datalogger_payload_t *payload){

    if(payload == NULL){
        return ESP_ERR_INVALID_ARG;
    }

    char data[DTLGR_LINE_MAX_LEN];
    size_t size = sizeof(data);

    int len = snprintf(data, size, "%" PRIu32 ",%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
        payload->timestamp,
        payload->roll,
        payload->pitch,
        payload->tires[TPMS_FRONT_LEFT].pressure_psi,
        payload->tires[TPMS_FRONT_LEFT].temp_c,
        payload->tires[TPMS_FRONT_RIGHT].pressure_psi,
        payload->tires[TPMS_FRONT_RIGHT].temp_c,
        payload->tires[TPMS_REAR_LEFT].pressure_psi,
        payload->tires[TPMS_REAR_LEFT].temp_c,
        payload->tires[TPMS_REAR_RIGHT].pressure_psi,
        payload->tires[TPMS_REAR_RIGHT].temp_c
    );

    if(len < 0 || len >= (int)size){
        ESP_LOGE(TAG, "Failed to format data logger payload");
        return ESP_FAIL;
    }

    FILE *f_datalogger = fopen(data_logger_file_name, "a+");

    if(f_datalogger == NULL){
        ESP_LOGE(TAG, "Failed to create data logger or log file");
        return ESP_FAIL;
    }

    if(fwrite(data, len, 1, f_datalogger) != 1){
        ESP_LOGE(TAG, "Failed to write data logger payload");
        fclose(f_datalogger);
        return ESP_FAIL;
    }

    fclose(f_datalogger);

    return ESP_OK;
}

esp_err_t datalogger_init_file_handler(void){

    esp_err_t err;

    mode_t permission = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;

    const char *datalogger_file_path = "/data/sessions";
    const char *logs_file_path = "/data/logs";
    static const char *datalogger_header = "Timestamp,Roll,Pitch,FL_Pressure_PSI,FL_Temp_C,FR_Pressure_PSI,FR_Temp_C,RL_Pressure_PSI,RL_Temp_C,RR_Pressure_PSI,RR_Temp_C\n";
    static const char *logs_header = "Timestamp,Tag,Log_Message\n";
    

    // 1. Create /sessions and /logs directories if they don't exist
    if(mkdir(datalogger_file_path, permission) != 0 && errno != EEXIST){
        ESP_LOGE(TAG, "Failed to create data logger directory");
        return ESP_FAIL;
    }else if(mkdir(logs_file_path, permission) != 0 && errno != EEXIST){
        ESP_LOGE(TAG, "Failed to create logs directory");
        return ESP_FAIL;
    }


    err = nvs_open("storage", NVS_READWRITE, &nvs_datalogger_handler);
    switch (err) {
        case ESP_OK:
            ESP_LOGI(TAG, "NVS opened successfully");
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGE(TAG, "NVS namespace not found");
            return ESP_FAIL;
        case ESP_ERR_NVS_INVALID_HANDLE:
            ESP_LOGE(TAG, "NVS invalid handle");
            return ESP_FAIL;
        default:
            ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
            return ESP_FAIL;
    }



    // 2. TODO: if session_count is not initialized, set it to 0 and save it to NVS
    err = nvs_get_u8(nvs_datalogger_handler, "session_count", &session_count);
    switch (err) {
        case ESP_OK:
            ESP_LOGI(TAG, "Read session_count = %" PRIu8, session_count);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(TAG, "The value is not initialized yet");
            session_count = 0;
            err = nvs_set_u8(nvs_datalogger_handler, "session_count", session_count);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error (%s) writing session_count", esp_err_to_name(err));
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "Initialized session_count = %" PRIu8, session_count);
            break;
        default:
            ESP_LOGE(TAG, "Error (%s) reading", esp_err_to_name(err));
            return ESP_FAIL;
    }


    // 3. session_<session_count>.csv and log_<session_count>.txt files created
    session_count++;
    
    sprintf(data_logger_file_name, "/data/sessions/session_%d.csv", session_count);
    sprintf(log_file_name, "/data/logs/log_%d.txt", session_count);

    err = nvs_set_u8(nvs_datalogger_handler, "session_count", session_count);
    switch (err) {
        case ESP_OK:
            ESP_LOGI(TAG, "Write session_count = %" PRIu8, session_count);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(TAG, "The value is not initialized yet");
            break;
        default:
            ESP_LOGE(TAG, "Error (%s) reading", esp_err_to_name(err));
    }

    if(nvs_commit(nvs_datalogger_handler) != ESP_OK){
        ESP_LOGE(TAG, "Failed to commit NVS changes");
        return ESP_FAIL;
    }
    nvs_close(nvs_datalogger_handler);

    FILE *f_datalogger;
    FILE *f_logs;

    f_datalogger = fopen(data_logger_file_name, "w");
    f_logs = fopen(log_file_name, "w");

    if(f_datalogger == NULL){
        ESP_LOGE(TAG, "Failed to create data logger file");
        fclose(f_datalogger);
        return ESP_FAIL;
    }else if(f_logs == NULL){
        ESP_LOGE(TAG, "Failed to create log file");
        fclose(f_datalogger);
        return ESP_FAIL;
    }



    // 4. Write the headers
    if(fwrite(datalogger_header, strlen(datalogger_header), 1, f_datalogger) != 1){
        ESP_LOGE(TAG, "Failed to write data logger header");
        fclose(f_datalogger);
        fclose(f_logs);
        return ESP_FAIL;
    }

    if(fwrite(logs_header, strlen(logs_header), 1, f_logs) != 1){
        ESP_LOGE(TAG, "Failed to write log header");
        fclose(f_logs);
        fclose(f_datalogger);
        return ESP_FAIL;
    }


    // 5. Close the files
    if(fclose(f_datalogger) != 0){
        ESP_LOGE(TAG, "Failed to close data logger file");
        return ESP_FAIL;
    }else if(fclose(f_logs) != 0){
        ESP_LOGE(TAG, "Failed to close log file");
        return ESP_FAIL;
    }


    return ESP_OK;
}


esp_err_t datalogger_init(void){

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.phy.self_powered = true;
    tusb_cfg.phy.vbus_monitor_io = GPIO_NUM_2;

    if(tinyusb_driver_install(&tusb_cfg) != ESP_OK){
        return ESP_FAIL;
    }else if(storage_init_spiflash(&wl_datalogger_handler) != ESP_OK){
        return ESP_FAIL;
    }
    
    tinyusb_msc_storage_handle_t storage_hdl;
    const tinyusb_msc_storage_config_t cfg = {
        .medium.wl_handle = wl_datalogger_handler,
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP
    };

    
    if(tinyusb_msc_new_storage_spiflash(&cfg, &storage_hdl) != ESP_OK){
        return ESP_FAIL;
    }

    return ESP_OK;
}