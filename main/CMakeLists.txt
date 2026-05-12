idf_component_register(
    SRCS "twai_listen_only.c"
    REQUIRES esp_driver_twai esp_driver_ledc
    INCLUDE_DIRS "."
)
