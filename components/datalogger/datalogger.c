#include "datalogger.h"
#include "tinyusb_default_config.h"



static esp_err_t storage_init_spiflash(wl_handle_t *wl_handle){

    esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "data_logger");
    
    if(partition == NULL){
        return ESP_FAIL;
    }else if(wl_mount(partition, &wl_handle) != ESP_OK){
        return ESP_FAIL;
    }

    return ESP_OK;
}



esp_err_t datalogger_init(void){

    wl_handle_t wl;

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();

    tusb_cfg.phy.self_powered = true;
    tusb_cfg.phy.vbus_monitor_io = GPIO_NUM_2;

    if(tinyusb_driver_install(&tusb_cfg) != ESP_OK){
        return ESP_FAIL;
    }else if(storage_init_spiflash(&wl) != ESP_OK){
        return ESP_FAIL;
    }

    tinyusb_msc_storage_handle_t storage_hdl;
    const tinyusb_msc_storage_config_t cfg = {
        .medium.wl_handle = wl,
    };

    if(tinyusb_msc_new_storage_spiflash(&cfg, &storage_hdl) != ESP_OK){
        return ESP_FAIL;
    }

    return ESP_OK;
}