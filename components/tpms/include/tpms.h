#ifndef TPMS_H
#define TPMS_H

#include "stdint.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TPMS_FRONT_LEFT,
    TPMS_FRONT_RIGHT,
    TPMS_REAR_LEFT,
    TPMS_REAR_RIGHT,
    TPMS_TIRE_COUNT
} tpms_t;

typedef struct {
    uint16_t header;
    uint16_t raw_temp;
    uint16_t raw_pressure;
    uint8_t  id[3];
    uint8_t  raw_battery;
    uint8_t  checksum;
}mfg_payload_t;

typedef uint8_t tpms_ble_addr[6];

typedef struct {
    tpms_t tire;
    tpms_ble_addr addr;
    int64_t last_seen_us;
    float temp;
    float pressure_bar;
    float pressure_psi;
    float bat;
} tpms_data_t;


/**
 * Initialize the TPMS component.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t tpms_init(void);

/**
 * Finds the tire assigned to a Bluetooth address.
 *
 * @param addr Bluetooth address to look up.
 * @param out_tire Pointer that receives the matching tire position.
 * @return ESP_OK if the address is found; otherwise, an error code.
 */
esp_err_t tpms_lookup_by_addr(const tpms_ble_addr addr, tpms_t *out_tire);


/**
 * Decodes a TPMS manufacturer payload into its structured data fields.
 *
 * @param payload Raw manufacturer payload data.
 * @param payload_len Length of the payload in bytes.
 * @param out_payload Pointer that receives the decoded payload.
 * @return ESP_OK on success, or an error code if decoding fails.
 */
esp_err_t tpms_decode_mfg_payload(const uint8_t *payload, size_t payload_len, mfg_payload_t *out_payload);

/**
 * Marks a tire as having received a new TPMS update.
 *
 * @param tire Tire position to mark as updated.
 */
void tpms_last_update(tpms_t tire);




/* Estas funciones quedan para luego. */
float tpms_get_pressure(tpms_t tire);
float tpms_get_temperature(tpms_t tire);
float tpms_get_battery(tpms_t tire);

#ifdef __cplusplus
}
#endif

#endif /* BLE_H */