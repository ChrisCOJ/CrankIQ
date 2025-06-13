#ifndef i2c_display_utils_h
#define i2c_display_utils_h

#include "driver/gpio.h"
#include "driver/i2c_master.h"


#define I2C_MASTER_PORT                 I2C_NUM_0     // Port number used by esp32
#define I2C_SCL_CLK_HZ                  100000        // I2C Clock speed in hz
#define I2C_MASTER_TIMEOUT_MS           1000          // Time in ms to timeout communication

#define COMMAND_BYTE                    0x00          // The data following this byte is interpreted as a command
#define DATA_BYTE                       0x40          // The data following this byte is interpreted as data to be drawn on the screen

#define SSD1306_I2C_SLAVE_ADDR          0x3C



int display_wake_up(i2c_master_dev_handle_t dev_handle);

int send_command(i2c_master_dev_handle_t dev_handle, uint8_t *data);
int send_data(i2c_master_dev_handle_t dev_handle, uint8_t *data)


#endif