#include "ble.h"
#include "gap.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

static const char *TAG = "ble";

void ble_store_config_init(void);

static void on_stack_reset(int reason) {
    ESP_LOGW(TAG, "nimble stack reset, reason: %d", reason);
}

static void on_stack_sync(void) {
    gap_on_sync();
}

static void nimble_host_task(void *param) {
    ESP_LOGI(TAG, "nimble host task started");
    nimble_port_run();
    vTaskDelete(NULL);
}

esp_err_t ble_init(void) {


    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d", ret);
        return ret;
    }

    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();

    return ESP_OK;
}

esp_err_t ble_register_scan_cb(ble_scan_cb_t cb, void *ctx) {
    if (cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    gap_set_report_cb(cb, ctx);
    return ESP_OK;
}

esp_err_t ble_start(void) {
    xTaskCreate(nimble_host_task, "NimBLE Host", 4 * 1024, NULL, 5, NULL);
    return ESP_OK;
}