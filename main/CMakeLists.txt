idf_component_register(SRCS "hello_world_main.c"
                    PRIV_REQUIRES 
                        spi_flash
                        esp_timer
                        SPI_BUS
                    INCLUDE_DIRS "")
