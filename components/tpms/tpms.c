#include "tpms.h"
#include "ble.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static tpms_data_t tpms_storage[TPMS_TIRE_COUNT];
static double psi_factor = 0.145038;

static const tpms_ble_addr tpms_know_addrs[TPMS_TIRE_COUNT] = {
    [TPMS_FRONT_LEFT]   = {0xb9, 0x41, 0xfa, 0x00, 0x26, 0xd6},
    [TPMS_FRONT_RIGHT]  = {0xb9, 0x41, 0xfa, 0x00, 0x23, 0x3b},
    [TPMS_REAR_LEFT]    = {0xb9, 0x41, 0xfa, 0x00, 0x27, 0x7c},
    [TPMS_REAR_RIGHT]   = {0xb9, 0x41, 0xfa, 0x00, 0x27, 0xce}

};

esp_err_t tpms_lookup_by_addr(const tpms_ble_addr addr, tpms_t *out_tire){
    int ret;
    esp_err_t result = ESP_FAIL;

    for(uint8_t i = 0 ; i < TPMS_TIRE_COUNT; i++){
        ret = memcmp(addr, tpms_know_addrs[i], sizeof(tpms_ble_addr));
        if(ret == 0){
            *out_tire = i;
            result = ESP_OK;
            break;
        }
    }
    return result;
}

esp_err_t tpms_raw_mfg_payload(const uint8_t *payload, size_t payload_len, mfg_payload_t *out_payload){
    if(payload_len != 11){ // 11 hardcodeado
        return ESP_FAIL;
    }

    out_payload->raw_temp =     (uint8_t) payload[3];
    out_payload->raw_pressure = ((uint16_t)payload[4] << 8) | payload[5];
    for(uint8_t i = 0 ; i < 3 ; i++){
        out_payload->id[i] = payload[6+i]; 
    }
    return ESP_OK;
}

esp_err_t tpms_last_update(tpms_t tire, mfg_payload_t *payload, tpms_data_t *out_value){

    // TODO: add ESP_FAIL when already know the checksum (not implemented yet).

    out_value->tire =         tire;
    out_value->pressure_bar = (float)(payload->raw_pressure - 100.0f) * 0.01043f;
    out_value->temp_c =       (float)payload->raw_temp - 50.0f;
    out_value->last_seen_us = esp_timer_get_time();
    return ESP_OK;
};


static void tpms_ble_scan_cb(const ble_adv_report_t *report, void *ctx){

    tpms_t tire;
    mfg_payload_t decoded;

    if( tpms_lookup_by_addr(report->addr, &tire) != ESP_OK){
        return;
    }

    if( tpms_raw_mfg_payload(report->mfg_payload, report->mfg_payload_len, &decoded) != ESP_OK){
        return;
    }
    
    if(tpms_last_update(tire, &decoded, &tpms_storage[tire]) != ESP_OK){
        return;
    }

}


esp_err_t tpms_init(){

    if(ble_init() != ESP_OK || ble_start() != ESP_OK ){
        return ESP_FAIL;
    }

    esp_err_t scan = ble_register_scan_cb(tpms_ble_scan_cb, NULL);
    if(scan != ESP_OK){
        return ESP_FAIL;
    }

    return ESP_OK;
}


float tpms_bar_to_psi(float pressure_bar){
    float pressure_psi = pressure_bar * psi_factor;
    return pressure_psi;
}
