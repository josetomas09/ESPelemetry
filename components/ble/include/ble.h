/* components/ble/include/ble.h */
#ifndef BLE_H
#define BLE_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_ADDR_LEN 6u

/**
 * @brief BLE advertising report.
 *
 * @warning name/mfg_data point into a buffer owned by the NimBLE host and
 *          are ONLY valid for the duration of the ble_scan_cb_t callback.
 *          Copy any data you need before returning from the callback.
 */
typedef struct {
    uint8_t addr[BLE_ADDR_LEN];
    int8_t  rssi;
    const uint8_t *name;
    uint8_t name_len;
    const uint8_t *mfg_payload;
    uint8_t mfg_payload_len;
} ble_adv_report_t;

typedef void (*ble_scan_cb_t)(const ble_adv_report_t *report, void *ctx);

/**
 * @brief Initialize the BLE stack.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ble_init(void);

/**
 * @brief Register a callback for BLE scan results.
 *
 * @param cb The callback function to register.
 * @param ctx The context to pass to the callback.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ble_register_scan_cb(ble_scan_cb_t cb, void *ctx);

/**
 * @brief Start the BLE scanning process.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ble_start(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_H */