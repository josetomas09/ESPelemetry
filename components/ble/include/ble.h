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
 * Reporte crudo de un advertising packet, ya desempaquetado.
 * No hay ni un solo tipo de NimBLE acá adentro — esa es la regla.
 *
 * IMPORTANTE: `name` NO viene terminado en '\0' (así es como llega por aire).
 * Usá siempre name_len, nunca strlen/strcmp directo sobre name.
 */
typedef struct {
    uint8_t addr[BLE_ADDR_LEN];
    int8_t  rssi;
    const uint8_t *name;
    uint8_t name_len;
    const uint8_t *mfg_data;
    uint8_t mfg_data_len;
} ble_adv_report_t;

/// Se llama una vez por cada advertising packet recibido, desde la tarea del host BLE.
typedef void (*ble_scan_cb_t)(const ble_adv_report_t *report, void *ctx);

/// Arma el host NimBLE. Llamar una sola vez, antes de ble_register_scan_cb/ble_start.
esp_err_t ble_init(void);

/// Se suscribe a los reportes de scan. Solo válido entre ble_init() y ble_start().
esp_err_t ble_register_scan_cb(ble_scan_cb_t cb, void *ctx);

/// Cierra la configuración, arranca la tarea del host y empieza a escanear.
esp_err_t ble_start(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_H */