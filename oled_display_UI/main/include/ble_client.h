#ifndef ble_client_h
#define ble_client_h

#include "esp_gattc_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"


#define GATTC_TAG       "SH1106"


typedef struct {
    esp_gatt_if_t gattc_if;
    esp_bd_addr_t remote_bda;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handles[2];
} gattc_conn_parameters;

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
void bt_init();

#endif