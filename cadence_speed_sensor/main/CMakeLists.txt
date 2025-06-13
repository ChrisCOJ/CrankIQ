idf_component_register(SRCS "src/mpu_i2c.c" "src/ble_server.c" "src/cadence_speed_sensor.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash bt driver esp_timer)