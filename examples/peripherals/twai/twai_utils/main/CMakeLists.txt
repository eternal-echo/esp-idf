idf_component_register(SRCS "cmd_twai_core.c" "cmd_twai.c" "twai_utils_main.c"
                    REQUIRES esp_driver_twai console
                    INCLUDE_DIRS ".")
