#include "datalogger.h"
#include "tinyusb_default_config.h"

static wl_handle_t storage_init_spiflash(void){
  wl_handle_t wl;
  // Find partition
  // Mount Wear Levelling
  return wl;
}




esp_err_t datalogger_init(void){

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();

    tusb_cfg.phy.self_powered = true;
    tusb_cfg.phy.vbus_monitor_io = GPIO_NUM_2;

    if(tinyusb_driver_install(&tusb_cfg) != ESP_OK){
        return ESP_FAIL;
    }

    wl_handle_t wl_handle = storage_init_spiflash();

    if(wl_handle != ESP_OK){
        return ESP_FAIL;
    }

    tinyusb_msc_storage_handle_t storage_hdl;
    const tinyusb_msc_storage_config_t cfg = {
        .medium.wl_handle = wl_handle,
    };
    tinyusb_msc_new_storage_spiflash(&cfg, &storage_hdl);

    return ESP_OK;
}