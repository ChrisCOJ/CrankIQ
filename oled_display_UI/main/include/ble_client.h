#ifndef ble_client_h
#define ble_client_h

#include "esp_gattc_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"


#define GATTC_TAG       "SH1106"


/**
* @name gattc_conn_parameters
*
* @brief Structure that stores connection parameters and characteristic handles for a BLE GATT client.
*
* @param gattc_if             GATT client interface handle associated with this connection.
* @param remote_bda           Bluetooth device address of the connected server.
* @param conn_id              Connection ID assigned by the BLE stack.
* @param service_start_handle Start handle of the GATT service of interest.
* @param service_end_handle   End handle of the GATT service of interest.
* @param notifiable_char_handles Array storing handles of notifiable characteristics (e.g., speed, cadence).
*/
typedef struct {
    esp_gatt_if_t gattc_if;
    esp_bd_addr_t remote_bda;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t notifiable_char_handles[2];
} gattc_conn_parameters;


void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);


/**
* @name bt_init()
*
* @brief Initializes Bluetooth by configuring the BLE controller, enabling the host stack, and starting GAP scanning.
*
* @return void.
*/
void bt_init();


/**
* @name get_speed()
*
* @brief Retrieves the most recently received speed value from the BLE characteristic.
*
* @return The speed value as an unsigned 8-bit integer.
*/
uint8_t get_speed();


/**
* @name get_cadence()
*
* @brief Retrieves the most recently received cadence value from the BLE characteristic.
*
* @return The cadence value as an unsigned 8-bit integer.
*/
uint8_t get_cadence();


bool connection_established();


#endif