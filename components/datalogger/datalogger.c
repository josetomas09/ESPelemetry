#include "datalogger.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"


tinyusb_config_t config = {

}

datalogger_init(){

    if(tinyusb_driver_install(*config) != ESP_OK){
        return ESP_FAIL:
    }

}


