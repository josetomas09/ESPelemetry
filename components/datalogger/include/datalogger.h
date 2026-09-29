#ifndef DATALOGGER_H
#define DATALOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "esp_err.h"
#include "tpms.h"

#define DTLGR_LINE_MAX_LEN 70

/** 
* @brief Data logger payload structure
*
* @details This structure holds the data for each log entry in the data logger.
*/
typedef struct {
    uint32_t timestamp;
    float roll;
    float pitch;
    tpms_data_t tires[TPMS_TIRE_COUNT];
}datalogger_payload_t;

/**
 * @brief Save data logger payload to memory
 *
 * @param payload Pointer to the datalogger_payload_t structure containing the data to be saved.
 * @return esp_err_t Returns ESP_OK on success, or ESP_ERR_INVALID_ARG if the payload is NULL.
 */
esp_err_t datalogger_save_on_memory(datalogger_payload_t *payload);

/**
 * @brief Initialize the data logger
 *
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t datalogger_init(void);


#ifdef __cplusplus
}
#endif

#endif /* DATALOGGER.H */