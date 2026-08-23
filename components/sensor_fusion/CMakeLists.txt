idf_component_register(
    SRCS "sensor_fusion.c"
    INCLUDE_DIRS "include"
    REQUIRES mpu6050 eekf
    )
