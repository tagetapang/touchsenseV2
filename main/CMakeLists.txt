idf_component_register(SRCS "touchsenseV2.c"
                    INCLUDE_DIRS ".")
spiffs_create_partition_image(storage ../partition FLASH_IN_PROJECT)