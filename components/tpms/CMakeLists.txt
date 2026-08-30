idf_component_register(
    SRCS "tpms.c"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash ble
)
