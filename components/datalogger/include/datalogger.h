#ifndef DATALOGGER_H
#define DATALOGGER_H


#ifdef __cplusplus
extern "C" {
#endif

static esp_err_t storage_init_spiflash(wl_handle_t *wl_handle);

esp_err_t datalogger_init(void);



#ifdef __cplusplus
}
#endif

#endif /* DATALOGGER.H */
