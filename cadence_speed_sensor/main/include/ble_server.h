#ifndef ble_server_h
#define ble_server_h


#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"


#define GATT_CHAR_DECL_SIZE                 (sizeof(uint8_t))
#define GATTS_TAG                           "GATTS"
#define CCCD_ENABLE                         1


enum service_numbers {
    CRANKIQ_SENSOR_SERVICE_INST,
    BATTERY_SERVICE_INST    
};            


/* --- Cadence sensor BLE table enums --- */
enum crankiq_profile {
    /* --- CrankIQ Sensor custom service --- */
    IDX_CRANKIQ_SENSOR_SERVICE,

    IDX_CADENCE_CHAR_DECL,
    IDX_CADENCE_CHAR_VAL,
    IDX_CADENCE_CCCD_DESC,

    IDX_SPEED_KMH_CHAR_DECL,
    IDX_SPEED_KMH_CHAR_VAL,
    IDX_SPEED_CCCD_DESC,

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

bool get_cadence_cccd();
bool get_speed_cccd();



#endif