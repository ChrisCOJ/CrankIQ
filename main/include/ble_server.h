#ifndef ble_server_h
#define ble_server_h


#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"


#define GATT_CHAR_DECL_SIZE                 (sizeof(uint8_t))
#define GATTS_TAG                           "GATTS"
#define CCCD_ENABLE                         1


enum service_numbers {
    CADENCE_SENSOR_SERVICE_INST,
    BATTERY_SERVICE_INST    
};            


/* --- Cadence sensor BLE table enums --- */
enum crankiq_profile {
    /* --- Cadence Sensor custom service --- */
    IDX_CADENCE_SENSOR_SERVICE,

    IDX_CADENCE_CHAR_DECL,
    IDX_CADENCE_CHAR_VAL,
    IDX_CADENCE_CCCD_DESC,

    /* --- Battery Service --- */
    IDX_BATTERY_SERVICE,

    IDX_BATTERY_LEVEL_CHAR_DECL,
    IDX_BATTERY_LEVEL_CHAR_VAL,

    PROFILE_LEN
};


typedef struct {
    uint16_t conn_id;
    esp_gatt_if_t gatts_if;
    uint16_t handle_table[PROFILE_LEN];
    size_t handle_table_size;
    esp_err_t ret;
} bt_conn_properties;


bt_conn_properties bt_init();



#endif