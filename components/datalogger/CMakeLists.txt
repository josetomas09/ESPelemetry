idf_component_register(
    SRCS "datalogger.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES tinyusb fatfs wear_levelling
)
