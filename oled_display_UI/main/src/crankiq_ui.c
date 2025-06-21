#include "driver/i2c_master.h"

#include "../include/i2c_display_utils.h"
#include "../include/ble_client.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"


#define DISPLAY_SDA_IO      22
#define DISPLAY_SCL_IO      23

#define SCAN_INTERVAL       30          // The time limit in seconds for scanning ble devices



void app_main() {
    /* --- I2C Setup --- */
    i2c_master_bus_handle_t mst_bus_handle;
    i2c_master_dev_handle_t dev_handle;

    i2c_master_bus_config_t mst_bus_config = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = DISPLAY_SDA_IO,
        .scl_io_num = DISPLAY_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&mst_bus_config, &mst_bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,  // Sets address length of the slave device
        .device_address = SH1106_I2C_SLAVE_ADDR,
        .scl_speed_hz = I2C_SCL_CLK_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(mst_bus_handle, &dev_config, &dev_handle));

    /* Start scanning for sensors */
    bt_init();
    
    /* --- Wake up display --- */
    display_init(dev_handle);

    /* Create user interface that allows users to connect to peripherals via bluetooth */
    // ...

    /* Show loading on connection attempt */
    int64_t start = esp_timer_get_time();
    double time_passed = 0;
    while (!connection_established() && time_passed < SCAN_INTERVAL) {
        draw_text(dev_handle, "Looking for", 30, 3);
        draw_text(dev_handle, "bt device.", 30, 5);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        draw_text(dev_handle, "Looking for", 30, 3);
        draw_text(dev_handle, "bt device..", 30, 5);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        draw_text(dev_handle, "Looking for", 30, 3);
        draw_text(dev_handle, "bt device...", 30, 5);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        time_passed = (double)(esp_timer_get_time() - start) / 1e6;
        clear_screen(dev_handle);
    }

    /* If connection unsuccessful, allow retry */
    if (!connection_established()) {
        draw_text(dev_handle, "Device not found!", 15, 2);
        draw_text(dev_handle, "Press reset to try", 13, 4);
        draw_text(dev_handle, "again!", 50, 6);
        return;
    }

    /* If connection successful, clear the screen and show data of interest */
    draw_text(dev_handle, "Connected!", 35, 3);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    clear_screen(dev_handle);

    // Draw the base user interface for the speed/cadence sensor
    int speed_next_col = draw_text(dev_handle, "Speed: ", 10, 2);
    int cadence_next_col = draw_text(dev_handle, "Cadence: ", 10, 4);

    /* --- Poll the ble sensor for cadence and speed values and update the user interface --- */
    while (1) {
        if (connection_established()) {
            draw_bt_icon(dev_handle);
        } else {
            clear_bt_icon(dev_handle);
        }
        int next_col;
        next_col = draw_integer(dev_handle, get_crankiq_speed(), speed_next_col, 2);
        draw_text(dev_handle, " km/h", next_col, 2);
        next_col = draw_integer(dev_handle, get_crankiq_cadence(), cadence_next_col, 4);
        draw_text(dev_handle, " rpm", next_col, 4);
    }
}
