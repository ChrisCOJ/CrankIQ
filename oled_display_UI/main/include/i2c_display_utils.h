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

enum DISPLAY_ERR_CODES {
    OK,
    GENERIC_ERR = 128,
    DISPLAY_BOUNDARY_OUT_OF_RANGE,
};

extern const uint8_t bluetooth_icon[2][7];
extern const uint8_t font5x7[][5];


/**
* @name display_init(i2c_master_dev_handle_t dev_handle)
*
* @brief Initializes the display by sending required configuration commands and clearing the screen.
*
* @return void.
* @param dev_handle The I2C handle of the display device.
*/
void display_init(i2c_master_dev_handle_t dev_handle);


/**
* @name clear_screen(i2c_master_dev_handle_t dev_handle)
*
* @brief Clears the entire screen by writing zeros to all display pages.
*
* @return void.
* @param dev_handle The I2C handle of the display device.
*/
void clear_screen(i2c_master_dev_handle_t dev_handle);


/**
* @name set_cursor(i2c_master_dev_handle_t dev_handle, uint8_t column, uint8_t page)
*
* @brief Sets the display's internal memory cursor to a specific column and page.
* This determines the position for the next drawing operation.
*
* @return Error codes. 0 = successful, otherwise failed.
* @param dev_handle The I2C handle of the display device.
* @param column Horizontal position (0–127) of the cursor.
* @param page Vertical page number (0–7) of the cursor.
*/
int set_cursor(i2c_master_dev_handle_t dev_handle, uint8_t column, uint8_t page);


/**
* @name send_command(i2c_master_dev_handle_t dev_handle, uint8_t command)
*
* @brief Sends a single command byte to the display controller.
*
* @return ESP-IDF error codes. ESP_OK = success, otherwise a communication error occurred.
* @param dev_handle The I2C handle of the display device.
* @param command The command byte to be sent.
*/
esp_err_t send_command(i2c_master_dev_handle_t dev_handle, uint8_t command);


/**
* @note Ensure the cursor is set before calling this function! 
*
* @name draw(i2c_master_dev_handle_t dev_handle, uint8_t *pixel_buf, size_t pixel_buf_len)
*
* @brief Draws a buffer of pixel data to the current cursor location on the display.
* The buffer should be formatted according to the display's memory layout.
*
* @return ESP-IDF error codes. ESP_OK = success, otherwise a communication error occurred.
* @param dev_handle The I2C handle of the display device.
* @param pixel_buf Pointer to the pixel buffer to be written (Do not exceed 127 bytes).
* @param pixel_buf_len Length of the buffer in bytes.
*/
esp_err_t draw(i2c_master_dev_handle_t dev_handle, uint8_t *pixel_buf, size_t pixel_buf_len);


/**
* @name draw_text(i2c_master_dev_handle_t dev_handle, char *text, uint8_t x, uint8_t y)
*
* @brief Function that takes in a string and draws it on the screen with the left corner starting at positioon (x,y).
*
* @return The column number immediately to the right of the last character drawn. If >= 128, then failed.
* @param dev_handle The i2c handle of the display device.
* @param text String to be drawn on the screen.
* @param x (AKA column) Horizontal start position of the text's left corner
* @param y (AKA page) Vertical start position of the text's left corner
*/
int draw_text(i2c_master_dev_handle_t dev_handle, char *text, uint8_t x, uint8_t y);


/**
* @name draw_integer(i2c_master_dev_handle_t dev_handle, int num, uint8_t x, uint8_t y)
*
* @brief Draws a signed integer on the screen, starting from the specified (x, y) position.
* The number is converted to a string and rendered using the existing draw_text() function.
*
* @return The column number immediately to the right of the last character drawn. If >= 128, then failed.
* @param dev_handle The I2C handle of the display device.
* @param number The signed integer to be displayed.
* @param x Horizontal start position of the number's left edge (in pixels).
* @param y Vertical start position of the number's top page (0–7).
*
* @note Negative numbers are supported and will be rendered with a leading minus sign.
*/
int draw_integer(i2c_master_dev_handle_t dev_handle, int num, uint8_t x, uint8_t y);


/**
* @name draw_bt_icon(i2c_master_dev_handle_t dev_handle, uint8_t x, uint8_t y)
*
* @brief Draws a Bluetooth icon at the specified (x, y) position using two rows of pixel data.
*
* @return void.
* @param dev_handle The I2C handle of the display device.
* @param x (AKA column) Horizontal start position of the icon's left corner.
* @param y (AKA page) Vertical start page of the icon's top half (second half is drawn on y+1).
*/
void draw_bt_icon(i2c_master_dev_handle_t dev_handle, uint8_t x, uint8_t y);



#endif