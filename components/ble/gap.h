/* components/ble/gap.h */
#ifndef BLE_GAP_PRIV_H
#define BLE_GAP_PRIV_H

#include "ble.h" /* ble_scan_cb_t, ble_adv_report_t */

void gap_on_sync(void);

void gap_set_report_cb(ble_scan_cb_t cb, void *ctx);

#endif /* BLE_GAP_PRIV_H */