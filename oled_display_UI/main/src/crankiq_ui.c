#include "i2c_master.h"
#include "../include/i2c_display_utils.h"


#define DISPLAY_SDA_IO      22
#define DISPLAY_SCL_IO      23



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
        .device_address = SSD1306_I2C_SLAVE_ADDR,
        .scl_speed_hz = I2C_SCL_CLK_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(mst_bus_handle, &dev_config, &dev_handle));

    
    /* --- Wake up display --- */
    
}