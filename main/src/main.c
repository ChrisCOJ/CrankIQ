#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../include/mpu_i2c.h"
#include "../include/ble_server.h"


#define I2C_SDA_IO              GPIO_NUM_22
#define I2C_SCL_IO              GPIO_NUM_23
#define FILTER_SAMPLE_SIZE      10

#define DEAD_ZONE               0.8f
#define DETECTION_RANGE         0.1f

#define MAX_MPU_ACCEL_RANGE     16
#define MAX_MPU_GYRO_RANGE      2000

enum axis {
    X_AXIS,
    Y_AXIS,
    Z_AXIS
};

enum error_codes {
    MPU_DATA_READ_FAIL = 50000,
    PARAMTER_OUT_OF_RANGE = 50001,
};


int mean(int16_t *data_arr, size_t data_arr_len) {
    int total = 0;
    for (int i = 0; i < data_arr_len; ++i) {
        total += data_arr[i];
    }
    return (total / (int)data_arr_len);
}


int read_initial_pos(int16_t *y_accel, size_t y_accel_size, i2c_master_dev_handle_t dev_handle) {
    int16_t raw_data[3];
    // Filter large values and small fluctuation in the raw data by averaging it out over a few readings
    for (int count = 0; count < y_accel_size; ++count) {
        if (mpu_read_data(MPU_ACCEL_DATA, dev_handle, raw_data, (sizeof(raw_data) / sizeof(int16_t)) ) ) {
            ESP_LOGE(MPU_TAG, "%s", "Failed to read MPU data!");
        }
        y_accel[count] = raw_data[1];
    }
    // On sensor wake up, read the first average y accelerometer value
    return mean(y_accel, y_accel_size);
}


int filter_arr(unsigned data_type, int16_t *arr, size_t arr_size, unsigned axis, i2c_master_dev_handle_t dev_handle) {
    int16_t raw_data[3];

    if (mpu_read_data(data_type, dev_handle, raw_data, (sizeof(raw_data) / sizeof(int16_t)) ) ) {
        ESP_LOGE(MPU_TAG, "%s", "Failed to read MPU data!");
        return MPU_DATA_READ_FAIL;
    }
    if (axis > 2) {
        ESP_LOGE(MPU_TAG, "axis parameter out of range");
        return PARAMTER_OUT_OF_RANGE;
    }
    if (data_type > 1) {
        ESP_LOGE(MPU_TAG, "data_type parameter out of range");
        return PARAMTER_OUT_OF_RANGE;
    }
    /* Remove the first element of the raw data arr and shift left to make space for a new value */
    memmove(arr, arr + 1, (arr_size - 1) * sizeof(int16_t));
    arr[arr_size - 1] = raw_data[axis];

    /* Get the mean value of the raw data arr */
    return mean(arr, arr_size);
}


float get_gyro_drift(i2c_master_dev_handle_t dev_handle, int iterations) {
    /* On start, keep the IMU sensor still, read 100 gyro values and average them to get the drift */
    int16_t arr[3];
    size_t arr_size = 3;
    float total = 0;
    for (int i = 0; i < iterations; ++i) {
        mpu_read_data(MPU_GYRO_DATA, dev_handle, arr, arr_size);
        total += arr[2];  // Z axis
    }
    total /= MPU_2000_DEG_DIV;  // Convert to deg/sec

    return total / iterations;
}


void app_main(void){
    /* --- Memory allocations --- */
    int16_t *z_gyro = calloc(FILTER_SAMPLE_SIZE, sizeof(int16_t));
    int16_t *y_accel = calloc(FILTER_SAMPLE_SIZE, sizeof(int16_t));

    
    /* --- Initialize bluetooth --- */
    if (bt_init() != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s Failed to initialize bluetooth", __func__);
        return;
    }

    /* --- Open i2c channel for communication with mpu6050 --- */
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
    /* -------------------------------------------------------- */

    /* --- Initialize mpu with appropriate accelerometer and gyro sensitivities --- */
    esp_err_t err = mpu_init(dev_handle, MPU6050_ACCEL_2G, MPU6050_GYRO_2000_DEG);
    if (err) return;

    /* --- Runtime Variables --- */
    int filtered_gyro_z = 0;
    double dt = 0;  // Elapsed time between gyro readings used to calculate angle
    int64_t start = 0;
    int64_t end = 0;
    float angle = 0;
    float filtered_accel_y = 0;
    bool allow_count = false;  // Flag to stop duplicate cadence readings when the sensor is stationary
    int rotations = 0;

    /* Calculate gyro drift on start and subtract from further readings */
    float gyro_z_offset = get_gyro_drift(dev_handle, 100);
    /* rotation_reference = the accelerometer y value at which a rotation should be counted */
    float rotation_reference = read_initial_pos(y_accel, FILTER_SAMPLE_SIZE, dev_handle);
    rotation_reference /= MPU_2G_DIV;  // Convert raw value to Gs

    while (1) {
        /* --- Filter accel y values by averaging over a few readings --- */
        filtered_accel_y = filter_arr(MPU_ACCEL_DATA, y_accel, FILTER_SAMPLE_SIZE, Y_AXIS, dev_handle);
        if ((int)filtered_accel_y > MAX_MPU_RAW_VALUE) {   // Out of range return values correspond to error values.
            int err = filtered_accel_y;
            ESP_LOGE(MPU_TAG, "filter_arr() failed with error no %d", err);
            // Cleanup
            free(y_accel);
            free(z_gyro);
            return;
        }
        filtered_accel_y /= MPU_2G_DIV;  // Convert raw value to Gs

        end = esp_timer_get_time();
        if (start) {
            dt = (double)(end - start) / 1e6;
            angle += filtered_gyro_z * dt;
            ESP_LOGI(MPU_TAG, "Angle: %.2f", angle);
            rotations = abs((int)(angle / 360.f));
        }
         /* --- Filter gyro z values by averaging over a few readings --- */
        filtered_gyro_z = filter_arr(MPU_GYRO_DATA, z_gyro, FILTER_SAMPLE_SIZE, Z_AXIS, dev_handle);
        if (filtered_gyro_z > MAX_MPU_RAW_VALUE) {
            int err = filtered_gyro_z;
            ESP_LOGE(MPU_TAG, "filter_arr() failed with error no %d", err);
            // Cleanup
            free(y_accel);
            free(z_gyro);
            return;
        }
        filtered_gyro_z /= MPU_2000_DEG_DIV;  // Convert raw value to degrees/sec
        filtered_gyro_z -= gyro_z_offset;
        start = esp_timer_get_time();


        /* --- Dead zone to prevent multiple premature readings of crank rotations --- */
        if ((filtered_accel_y > (rotation_reference + DEAD_ZONE)) || (filtered_accel_y < (rotation_reference - DEAD_ZONE))) {
            allow_count = true;
        }
        /* --- Count rotations when the y accel value reaches the initial value (range) again --- */
        if (allow_count) {
            if ((filtered_accel_y >= rotation_reference) && (filtered_accel_y < (rotation_reference + DETECTION_RANGE))) {
                ESP_LOGI(MPU_TAG, "%d", rotations);
                allow_count = false;
            }
        }

        // ESP_LOGI(MPU_TAG, "Z: %d", filtered_gyro_z);
        // ESP_LOGI(MPU_TAG, "ACCEL Y: %.2f", filtered_accel_y);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}