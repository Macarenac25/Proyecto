idf_component_register(SRCS "Tesis_BLE.c"
                    PRIV_REQUIRES bt nvs_flash
                    INCLUDE_DIRS "."
                    REQUIRES driver 
                    )
