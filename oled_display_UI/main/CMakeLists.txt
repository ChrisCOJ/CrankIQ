idf_component_register(SRCS "src/i2c_display_utils.c" "src/crankiq_ui.c" "src/ble_client.c"
                       INCLUDE_DIRS "include"
                       REQUIRES nvs_flash bt driver)