idf_component_register(
    SRCS "ble.c" "gap.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES nvs_flash
    PRIV_REQUIRES bt
)