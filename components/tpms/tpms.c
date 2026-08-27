#include "tpms.h"
#include "ble.h"
#include <string.h>

static tpms_data_t tpms_storage[TPMS_TIRE_COUNT];
static double psi_factor = 0.145038;

static const tpms_ble_addr tpms_know_addrs[TPMS_TIRE_COUNT] = {
    [TPMS_FRONT_LEFT]   = {0xb9, 0x41, 0xfa, 0x00, 0x26, 0xd6},
    [TPMS_FRONT_RIGHT]  = {0xb9, 0x41, 0xfa, 0x00, 0x23, 0x3b},
    [TPMS_REAR_LEFT]    = {0xb9, 0x41, 0xfa, 0x00, 0x27, 0x7c},
    [TPMS_REAR_RIGHT]   = {0xb9, 0x41, 0xfa, 0x00, 0x27, 0xce}

};

esp_err_t tpms_lookup_by_addr(const tpms_ble_addr addr, tpms_t *out_tire){
    esp_err_t result = ESP_FAIL;
    int ret;
    for(uint8_t i = 0 ; i < TPMS_TIRE_COUNT; i++) {
        ret = memcmp(addr, tpms_know_addrs[i], sizeof(tpms_ble_addr));
        if(ret == 0){
            *out_tire = i;
            result = ESP_OK;
            break;
        }
    }
    return result;
}

esp_err_t tpms_decode_mfg_payload(const uint8_t *payload, size_t payload_len, mfg_payload_t *out_payload){
    // Unfinished function.
    esp_err_t result = ESP_FAIL;

    uint16_t raw_kpa = 0;
    float psi, bar;

    if(payload_len != 11){ // fix: numero 11 hardcodeado
        return result;
    }

    /* Payload from BLE (raw)*/
    out_payload->header =       ((uint16_t)payload[0] << 8) | payload[1];
    out_payload->raw_temp =     ((uint16_t)payload[2] << 8) | payload[3];
    out_payload->raw_pressure = ((uint16_t)payload[4] << 8) | payload[5];
    out_payload->raw_battery =  payload[9];
    out_payload->checksum =     payload[10];
    for(uint8_t i = 0 ; i <= 3 ; i++){
        out_payload->id = ((uint16_t)payload[i] << 16); // fix this "expression must be a modifiable lvalue"
    }



    return result;
}

void tpms_last_update(tpms_t tire){
    /* TODO: If tpms_decode_mfg_payload() returns ESP_FAIL, log the error and do not mark the tire as "updated". */
};


static void tpms_ble_scan_cb(const ble_adv_report_t *report, void *ctx){
    // TODO: check whether the arguments are valid.
    // unfinished function

    for(uint8_t i = 0 ; i < TPMS_TIRE_COUNT ; i++){
        tpms_lookup_by_addr(report->addr,);
        tpms_decode_mfg_payload(report->mfg_payload, report->mfg_payload_len,);
    }
    

}


esp_err_t tpms_init(){
    // Unfinished function.
    if (ble_init() != ESP_OK) {
        return ESP_FAIL;
    }

    ble_register_scan_cb(tpms_ble_scan_cb, NULL); // example arguments!
    ble_start();



    return ESP_OK;
}







