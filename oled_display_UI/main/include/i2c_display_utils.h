#ifndef i2c_display_utils_h
#define i2c_display_utils_h

#include "driver/i2c_master.h"


#define SH1106_COLUMNS                  132
#define SH1106_ROWS                     64                   

#define I2C_MASTER_PORT                 I2C_NUM_0     // Port number used by esp32
#define I2C_SCL_CLK_HZ                  100000        // I2C Clock speed in hz
#define I2C_MASTER_TIMEOUT_MS           1000          // Time in ms to timeout communication

#define COMMAND_BYTE                    0x00          // The data following this byte is interpreted as a command
#define DATA_BYTE                       0x40          // The data following this byte is interpreted as data to be drawn on the screen

#define SH1106_I2C_SLAVE_ADDR           0x3C
#define COLUMN_OFFSET                   2             // This is the start of the display from the left

/* --- Commands --- */
#define SET_DISPLAY_ON                  0xAF          // Display ON in normal mode

#define HORIZONTAL_ADDRESSING_MODE      0x20          // Command to select the horizontal addressing mode
#define VERTICAL_ADDRESSING_MODE        0x21          // Command to select the vertical addressing mode
#define PAGE_ADDRESSING_MODE            0x22          // Command to select the page addressing mode

#define INVERSE_MODE                    0xA7
#define NORMAL_MODE                     0xA6

#define COLUMN_LOWER_NIBBLE             0x00          // Command to set the lower nibble of the column address
#define COLUMN_HIGHER_NIBBLE            0x10          // Command to set the higher nibble of the column address

#define PAGE_START_ADDRESS              0xB0          // Command to set the start page address (0 to 7)
/* ---------------- */

extern const uint8_t bluetooth_icon[2][7];
extern const uint8_t font5x7[][5];


void display_init(i2c_master_dev_handle_t dev_handle);
void clear_screen(i2c_master_dev_handle_t dev_handle);
int set_cursor(i2c_master_dev_handle_t dev_handle, uint8_t column, uint8_t page);

esp_err_t send_command(i2c_master_dev_handle_t dev_handle, uint8_t command);
esp_err_t draw(i2c_master_dev_handle_t dev_handle, uint8_t *pixel_buf, size_t pixel_buf_len);
int draw_text(i2c_master_dev_handle_t dev_handle, char *text, uint8_t x, uint8_t y);
void draw_bt_icon(i2c_master_dev_handle_t dev_handle, uint8_t x, uint8_t y);



#endif