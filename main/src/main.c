#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../include/mpu_i2c.h"
#include "../include/ble_server.h"


#define I2C_SDA_IO                      GPIO_NUM_22
#define I2C_SCL_IO                      GPIO_NUM_23
#define FILTER_SAMPLE_SIZE              10
#define VARIANCE_SAMPLE_SIZE            500
#define DRIFT_SAMPLE_SIZE               100

#define DEAD_ZONE                       0.8f
#define DETECTION_RANGE                 0.1f
#define ACCEPTABLE_GYRO_VALUE           0.1f

#define MAX_MPU_ACCEL_RANGE             16
#define MAX_MPU_GYRO_RANGE              2000

#define CADENCE_ROLLING_AVERAGE_SIZE    60

#define GEAR_RATIO                      1
#define WHEEL_CIRCUMFERENCE             1   // in meteres

enum axis {
    X_AXIS,
    Y_AXIS,
    Z_AXIS
};

enum error_codes {
    MPU_DATA_READ_FAIL = 50000,
    PARAMTER_OUT_OF_RANGE = 50001,
};


// float mean(int16_t *data_arr, size_t data_arr_len) {
//     float total = 0;
//     for (int i = 0; i < data_arr_len; ++i) {
//         total += data_arr[i];
//     }
//     return (total / (float)data_arr_len);
// }


// float filter_arr(unsigned data_type, int16_t *arr, size_t arr_size, unsigned axis, i2c_master_dev_handle_t dev_handle) {
//     int16_t raw_data[3];

//     if (mpu_read_data(data_type, dev_handle, raw_data, (sizeof(raw_data) / sizeof(int16_t)) ) ) {
//         ESP_LOGE(MPU_TAG, "%s", "Failed to read MPU data!");
//         return MPU_DATA_READ_FAIL;
//     }
//     if (axis > 2) {
//         ESP_LOGE(MPU_TAG, "axis parameter out of range");
//         return PARAMTER_OUT_OF_RANGE;
//     }
//     if (data_type > 1) {
//         ESP_LOGE(MPU_TAG, "data_type parameter out of range");
//         return PARAMTER_OUT_OF_RANGE;
//     }
//     /* Remove the first element of the raw data arr and shift left to make space for a new value */
//     memmove(arr, arr + 1, (arr_size - 1) * sizeof(int16_t));
//     arr[arr_size - 1] = raw_data[axis];

//     /* Get the mean value of the raw data arr */
//     return mean(arr, arr_size);
// }


float calc_gyro_z_drift(i2c_master_dev_handle_t dev_handle, int iterations) {
    /**
     * @brief Returns the stationary drift of the gyro z axis (in deg/sec)
     * The sensor MUST be held still during start-up to get a reliable drift estimate
     */
    int16_t arr[3];
    float mean = 0;
    for (int i = 0; i < iterations; ++i) {
        mpu_read_data(MPU_GYRO_DATA, dev_handle, arr, sizeof(arr));
        mean += arr[2];  // Z axis
    }
    mean /= iterations;

    return mean / MPU_2000_DEG_DIV;  // Convert to deg/sec
}


float calc_gyro_z_variance(i2c_master_dev_handle_t dev_handle, int iterations, float drift) {
    /**
     * @brief Returns the calcualted variance of the gyro z axis over the specified iteration steps (in deg/sec)
     */

    int16_t arr[3];
    float variance = 0;
    for (int i = 0; i < iterations; ++i) {
        mpu_read_data(MPU_GYRO_DATA, dev_handle, arr, sizeof(arr));
        float z = arr[2] - drift;  // Account for initial gyro drift
        variance += z * z;
    }
    variance /= iterations;

    return variance / MPU_2000_DEG_DIV;  // Convert to deg/sec
}


void notify_cadence(bt_conn_properties *bt_conn, uint8_t cadence) {
    /* --- Check if the client subscribed to cadence notifications --- */
    if (!get_cadence_cccd()) {
        return;
    }
    /* --- Send cadence data notification to the BLE client --- */
    esp_ble_gatts_send_indicate(bt_conn->gatts_if, bt_conn->conn_id, 
                                bt_conn->handle_table[IDX_CADENCE_CHAR_VAL],
                                sizeof(cadence), &cadence, false);
}

void notify_speed(bt_conn_properties *bt_conn, uint8_t speed) {
    /* --- Check if the client subscribed to speed notifications --- */
    if (!get_speed_cccd()) {
        return;
    }
    /* --- Send speed data notification to the BLE client --- */
    esp_ble_gatts_send_indicate(bt_conn->gatts_if, bt_conn->conn_id, 
                                bt_conn->handle_table[IDX_SPEED_KMH_CHAR_VAL],
                                sizeof(speed), &speed, false);
}


void app_main(void){    
    /* --- Initialize bluetooth --- */
    bt_conn_properties bt_conn = bt_init();
    if (bt_conn.ret != ESP_OK) {
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
    vTaskDelay(200 / portTICK_PERIOD_MS);

    /* --- Runtime Variables --- */
    int16_t gyro_arr[3];
    float gyro_z = 0;
    double dt = 0;  // Elapsed time between gyro readings used to calculate angle

    double rotation_dt = 0;  // Elapsed time between full crank rotations
    int64_t start = 0;
    int64_t end = 0;
    float angle = 0;
    bool full_rotation = false;

    /* --- Kalman filter variables --- */
    float estimated_gyro_z = 0.f;
    float estimate_variance = 1.f;
    float gyro_z_variance;
    float system_variance = 0.1f;  // Predicted change in cadence after each reading (tunable)

    /* Calculate stationary gyro drift on start and subtract from further readings */
    float gyro_z_offset = calc_gyro_z_drift(dev_handle, DRIFT_SAMPLE_SIZE);

    /* Calculate the variance of gyro readings */
    gyro_z_variance = calc_gyro_z_variance(dev_handle, VARIANCE_SAMPLE_SIZE, gyro_z_offset);

    while (1) {
        /* --- Read raw gyro data, convert to deg/sec, and use the time taken to integrate angle --- */
        end = esp_timer_get_time();
        if (start) {
            dt = (double)(end - start) / 1e6;
            angle += estimated_gyro_z * dt;
            rotation_dt += dt;
            /* Reset angle on completed rotation */
            if (angle >= 360.0f) {
                full_rotation = true;
                angle -= 360.0f;
            }
            if (angle <= -360.0f) {
                full_rotation = true;
                angle += 360.0f;
            }
        }

        int err = mpu_read_data(MPU_GYRO_DATA, dev_handle, gyro_arr, sizeof(gyro_arr));
        if (err) {
            ESP_LOGE(MPU_TAG, "mpu_read_data() failed with err code %d", err);
            continue;
        }
        gyro_z = (gyro_arr[2] / MPU_2000_DEG_DIV) - gyro_z_offset;  // Convert raw value to degrees/sec

        /* Add threshold that ignores very small fluctuations */
        if (fabsf(gyro_z) < ACCEPTABLE_GYRO_VALUE) {
            gyro_z = 0;
        }

        /* --- Apply kalman filter to smooth out data and reduce the impact of noise (e.g. vibrations) --- */
        /* Predict estimate uncertainty */
        estimate_variance += system_variance;

        /* Update estimate */
        float kalman_gain = estimate_variance / (estimate_variance + gyro_z_variance);
        estimated_gyro_z = estimated_gyro_z + kalman_gain * (gyro_z - estimated_gyro_z);
        estimate_variance = (1 - kalman_gain) * estimate_variance;

        start = esp_timer_get_time();
        /*------------------------------------------------------------------------------------------*/

        int cadence = fabsf((estimated_gyro_z / 360.f) * 60.f);
        notify_cadence(&bt_conn, cadence);

        int speed = (cadence * GEAR_RATIO * WHEEL_CIRCUMFERENCE) * 60 / 1000;  // (... * 60 / 1000) to convert from meters/minute to kmh
        notify_speed(&bt_conn, speed);
        // ESP_LOGI(MPU_TAG, "CADENCE = %d", cadence);

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}