idf_component_register(
    SRCS "datalogger.c"
    INCLUDE_DIRS "include"
    REQUIRES tpms
    PRIV_REQUIRES espressif__esp_tinyusb fatfs wear_levelling nvs_flash
)
