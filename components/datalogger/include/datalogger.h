#ifndef DATALOGGER_H
#define DATALOGGER_H


#ifdef __cplusplus
extern "C" {
#endif

static wl_handle_t storage_init_spiflash(void);

esp_err_t datalogger_init(void);



#ifdef __cplusplus
}
#endif

#endif /* DATALOGGER.H */