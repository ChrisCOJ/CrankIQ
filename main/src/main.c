#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../include/mpu_i2c.h"


#define I2C_SDA_IO          GPIO_NUM_23
#define I2C_SCL_IO          GPIO_NUM_22
#define FILTER_SAMPLE_SIZE  10


int16_t mean(int16_t *data_arr, size_t data_arr_len) {
    int32_t filtered_data = 0;
    for (int i = 0; i < data_arr_len; ++i) {
        filtered_data += data_arr[i];
    }
    return (int16_t)(filtered_data / (int32_t)data_arr_len);
}


void app_main(void){
    // Open i2c channel for communication with mpu6050
    i2c_master_bus_handle_t mst_bus_handle;
    i2c_master_dev_handle_t dev_handle;

    i2c_master_bus_config_t mst_bus_config = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&mst_bus_config, &mst_bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,  // Sets address length of the slave device
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = I2C_SCL_CLK_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(mst_bus_handle, &dev_config, &dev_handle));

    mpu_init(dev_handle);

    int16_t raw_data[3];
    char *tag[] = {"X", "Y", "Z"};
    int16_t *x_arr = calloc(FILTER_SAMPLE_SIZE, sizeof(int16_t));
    int16_t *y_arr = calloc(FILTER_SAMPLE_SIZE, sizeof(int16_t));
    int16_t filtered_data[2];
    int count = 0;
    while (1) {
        if (mpu_read_data(MPU_ACCEL_DATA, dev_handle, raw_data, (sizeof(raw_data) / sizeof(int16_t)))) {
            ESP_LOGE(MPU_TAG, "%s", "Failed to read MPU data!");
        }
        // Filter large values and small fluctuation in the raw data by averaging it out over a few readings
        if (count < FILTER_SAMPLE_SIZE) {
            x_arr[count] = raw_data[0];
            y_arr[count] = raw_data[1];
            ++count;
        }
        else {
            /* Get the mean value of each raw data arr */
            filtered_data[0] = mean(x_arr, FILTER_SAMPLE_SIZE);
            filtered_data[1] = mean(y_arr, FILTER_SAMPLE_SIZE);
            /* Remove the first element of each raw data arr and shift left to make space for a new value */
            memmove(x_arr, x_arr + 1, (FILTER_SAMPLE_SIZE - 1) * sizeof(int16_t));
            x_arr[FILTER_SAMPLE_SIZE - 1] = raw_data[0];
            memmove(y_arr, y_arr + 1, (FILTER_SAMPLE_SIZE - 1) * sizeof(int16_t));
            y_arr[FILTER_SAMPLE_SIZE - 1] = raw_data[1];

            // for (int i = 0; i < 3; ++i) {
            //     ESP_LOGI(MPU_TAG, "%s: %d", tag[i], data[i]);
            // }

            ESP_LOGI(MPU_TAG, "%s: %d", tag[1], filtered_data[1]);
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
    }
}