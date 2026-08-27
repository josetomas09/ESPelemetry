/* components/ble/gap.c */
#include "gap.h"

#include <string.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

static const char *TAG = "ble_gap";

static ble_scan_cb_t s_report_cb = NULL;
static void *s_report_ctx = NULL;
static uint8_t s_own_addr_type;

void gap_set_report_cb(ble_scan_cb_t cb, void *ctx) {
    s_report_cb = cb;
    s_report_ctx = ctx;
}

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        const struct ble_gap_disc_desc *disc = &event->disc;

        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
        if (rc != 0) {
            /* Malformed payload, ignore it */
            return 0;
        }

        if (s_report_cb == NULL) {
            return 0;
        }

        ble_adv_report_t report = {0};
        memcpy(report.addr, disc->addr.val, BLE_ADDR_LEN);
        report.rssi = disc->rssi;

        if (fields.name != NULL) {
            report.name = fields.name;
            report.name_len = fields.name_len;
        }
        if (fields.mfg_data != NULL) {
            report.mfg_data = fields.mfg_data;
            report.mfg_data_len = fields.mfg_data_len;
        }

        s_report_cb(&report, s_report_ctx);
        break;
    }
    default:
        break;
    }
    return 0;
}

static esp_err_t gap_start_scan(void) {
    struct ble_gap_disc_params disc_params = {
        .itvl = 160,              /* == itvl -> continuous scanning */
        .window = 160,
        .passive = 1,
        .filter_duplicates = 0,
    };

    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &disc_params,
                           gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc failed, error code: %d", rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TPMS scan started (passive, continuous)");
    return ESP_OK;
}

void gap_on_sync(void) {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return;
    }

    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    gap_start_scan();
}