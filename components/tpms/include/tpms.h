#ifndef TPMS_H
#define TPMS_H

#include "stdint.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Tire position identifiers used by the TPMS component.
 */
typedef enum {
    TPMS_FRONT_LEFT,
    TPMS_FRONT_RIGHT,
    TPMS_REAR_LEFT,
    TPMS_REAR_RIGHT,
    TPMS_TIRE_COUNT
} tpms_t;

/**
 * @brief Decoded manufacturer payload received from a TPMS sensor.
 */
typedef struct {
    uint8_t  raw_temp;
    uint16_t raw_pressure;
    uint8_t  id[3];
//  uint8_t  raw_battery;   The battery is in the string but is encrypted.
//  uint16_t checksum;      TODO: verify algorithm before use for validation.
} mfg_payload_t;

typedef uint8_t tpms_ble_addr[6];

/**
 * @brief Most recently known state for a monitored tire.
 */
typedef struct {
    tpms_t tire;
    tpms_ble_addr addr;
    int64_t last_seen_us;
    float temp_c;
    float pressure_bar;
} tpms_data_t;

/**
 * @brief Initializes the TPMS component and its internal state.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t tpms_init(void);

/**
 * @brief Finds the tire assigned to a Bluetooth address.
 *
 * @param[in] addr Bluetooth address to look up.
 * @param[out] out_tire Pointer that receives the matching tire position.
 * @return ESP_OK if the address is found; otherwise, an error code.
 */
esp_err_t tpms_lookup_by_addr(const tpms_ble_addr addr, tpms_t *out_tire);

/**
 * @brief Decodes a TPMS manufacturer payload into structured data fields.
 *
 * @param[in] payload Raw manufacturer payload data to decode.
 * @param[in] payload_len Length of the payload in bytes.
 * @param[out] out_payload Pointer that receives the decoded payload.
 * @return ESP_OK on success, or an error code if decoding fails.
 */
esp_err_t tpms_raw_mfg_payload(const uint8_t *payload, size_t payload_len, mfg_payload_t *out_payload);

/**
 * @brief Updates the stored data for a tire using a decoded TPMS payload.
 *
 * @param[in] tire Tire position to update.
 * @param[in] payload Decoded TPMS manufacturer payload used to refresh the state.
 * @param[out] out_value Pointer that receives the updated tire data.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t tpms_last_update(tpms_t tire, mfg_payload_t *payload, tpms_data_t *out_value);

/**
 * @brief Converts a pressure value from bar to psi.
 *
 * @param[in] pressure_bar Pressure value in bar.
 * @return Pressure value converted to psi.
 */
float tpms_bar_to_psi(float pressure_bar);

/**
 * @brief Retrieves the latest cached data for a tire.
 *
 * @param[in] tire Tire position to query.
 * @param[out] out_data Pointer that receives the tire data.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t tpms_get_data(tpms_t tire, tpms_data_t *out_data);


#ifdef __cplusplus
}
#endif

#endif /* TPMS.H */