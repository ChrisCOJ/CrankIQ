#include <string.h>

#include "esp_system.h"
#include "esp_log.h"

#include "../include/ble_server.h"

#include "nvs_flash.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"


#define DEVICE_NAME             "CrankIQ"
#define CRANKIQ_APP_ID          1


static const uint16_t GATT_SERVICE_DECL_UUID = 0x2800;
static const uint16_t GATT_CHAR_DECL_UUID = 0x2803;
static const uint16_t GATT_CCCD_UUID = 0x2902;

uint16_t crankiq_handle_table[PROFILE_LEN];

/* LSB ------------------------------> MSB */
uint8_t CADENCE_SERVICE_UUID[16] = {
    0x34, 0x90, 0x15, 0x6b, 0xbb, 0x3e, 0x4d, 0x96, 
    0x96, 0xd3, 0x11, 0x42, 0x6d, 0xe5, 0xc5, 0x20
};

uint8_t CADENCE_SENSOR_CHAR_UUID[16] = {
    0x34, 0x91, 0x15, 0x6b, 0xbb, 0x3e, 0x4d, 0x96, 
    0x96, 0xd3, 0x11, 0x42, 0x6d, 0xe5, 0xc5, 0x20
};

uint8_t cadence = 0;
uint16_t cccd_value = 0;  // Default init. The client will later write to this to enable notifications

/* --- Properties --- */
esp_gatt_char_prop_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/* --- Connection variables */
uint16_t conn_id;
uint16_t gatts_if;


/* --- GATTS Table --- */
const esp_gatts_attr_db_t crankiq_gatts_db[PROFILE_LEN] = {
    [IDX_CADENCE_SENSOR_SERVICE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&GATT_SERVICE_DECL_UUID, ESP_GATT_PERM_READ, 
         sizeof(CADENCE_SERVICE_UUID), sizeof(CADENCE_SERVICE_UUID), CADENCE_SERVICE_UUID}
    },

    [IDX_CADENCE_CHAR_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&GATT_CHAR_DECL_UUID, ESP_GATT_PERM_READ,
         GATT_CHAR_DECL_SIZE, GATT_CHAR_DECL_SIZE, (uint8_t *)&char_prop_notify}
    },
    [IDX_CADENCE_CHAR_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, CADENCE_SENSOR_CHAR_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(cadence), sizeof(cadence), &cadence}
    },
    [IDX_CADENCE_CCCD_DESC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&GATT_CCCD_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)&cccd_value}
    },
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = false,                // Name set to false not to exceed the max advertising packet size of 31 bytes
    .include_txpower = true,
    .min_interval = 0x0006,                 // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010,                 // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x0340,                   // HID Generic Mouse,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(CADENCE_SERVICE_UUID),
    .p_service_uuid = CADENCE_SERVICE_UUID,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = false,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = 0
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    esp_err_t ret;

    switch(event) {
        /* 
        * Set a device name and configure advertising data following successfull application registration.
        * Also create an attribute table for the HID Service.
        */
        case ESP_GATTS_REG_EVT:
            ret = esp_ble_gap_set_device_name(DEVICE_NAME);
             if (ret != ESP_OK){
                ESP_LOGE(GATTS_TAG, "%s -> Error on line %d. Set device name failed! Error code: %d", __func__, __LINE__, ret);
                return;
             }
             ESP_LOGI(GATTS_TAG, "%s -> Set device name successful! LINE %d", __func__, __LINE__);

            ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret != ESP_OK){
                ESP_LOGE(GATTS_TAG, "%s -> Error on line %d. Config advertising data failed! Error code: %d", __func__, __LINE__, ret);
                return;
             }
             ESP_LOGI(GATTS_TAG, "%s -> Config advertising data successful! LINE %d", __func__, __LINE__);

            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret != ESP_OK){
                ESP_LOGE(GATTS_TAG, "%s -> Error on line %d. Config scan response data failed! Error code: %d", __func__, __LINE__, ret);
                return;
            }
            ESP_LOGI(GATTS_TAG, "%s -> Config scan response data successful! LINE %d", __func__, __LINE__);

            ret = esp_ble_gatts_create_attr_tab(crankiq_gatts_db, gatts_if, PROFILE_LEN, CADENCE_SENSOR_SERVICE_INST);
            if (ret != ESP_OK) {
                ESP_LOGE(GATTS_TAG, "%s -> Error on line %d. Creating gatt atribute table failed! Error code: %d", __func__, __LINE__, ret);
                return;
            }
            ESP_LOGI(GATTS_TAG, "%s -> Created gatt attribute table successfully! LINE %d", __func__, __LINE__);
            break;
            
        /* 
        * Check whether the attribute table has been created correctly.
        * Start the hid gatt service.
        */
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            ESP_LOGI(GATTS_TAG, "The total number of handles = %d", param->add_attr_tab.num_handle);
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
                return;
            }
            else if (param->add_attr_tab.num_handle != PROFILE_LEN) {
                ESP_LOGE(GATTS_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to PROFILE_LEN(%d)", 
                         param->add_attr_tab.num_handle, PROFILE_LEN);
                return;
            }
            else {
                ESP_LOGI(GATTS_TAG, "create attribute table successfully, the number of handles = %d", param->add_attr_tab.num_handle);

                memcpy(crankiq_handle_table, param->add_attr_tab.handles, sizeof(crankiq_handle_table));
                ESP_LOGI(GATTS_TAG, "CrankIQ handle table: ");
                for (int i = 0; i < param->add_attr_tab.num_handle; ++i) {
                    ESP_LOGI(GATTS_TAG, "%d", crankiq_handle_table[i]);
                }
                esp_ble_gatts_start_service(crankiq_handle_table[IDX_CADENCE_SENSOR_SERVICE]);
            }
            break;

        case ESP_GATTS_WRITE_EVT: {
            if (param->write.handle == crankiq_handle_table[IDX_CADENCE_CCCD_DESC] && param->write.len == 2) {
                // Read the written cccd value to check what messages the client has subscribed for.
                uint16_t read_cccd_val = param->write.value[0] | param->write.value[1] << 8;
                if (read_cccd_val != CCCD_ENABLE) {
                    ESP_LOGW(GATTS_TAG, "Disabled cadence notifications!");
                    break;
                }
                ESP_LOGI(GATTS_TAG, "Subscribed to cadence notifications!");
            }
            break;
        }

        case ESP_GATTS_CONNECT_EVT: {
            // Store connection id
            conn_id = param->connect.conn_id;

            // Update connection parameters
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            esp_ble_gap_update_conn_params(&conn_params);

            // Initiates encryption with the remote device on connect
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT); 
            break;
        }

        // case ESP_GATTS_CONF_EVT:
        //     ESP_LOGI(GATTS_TAG, "Notification status = %d", param->conf.status);
        //     ESP_LOGI(GATTS_TAG, "Notification value length = %d", param->conf.len);

        //     for (int i = 0; i < param->conf.len; ++i) {
        //         ESP_LOGI(GATTS_TAG, "Byte%d = %u", i, *(param->conf.value + i));
        //     }
        //     break;

        case ESP_GATTS_DISCONNECT_EVT:
            ret = esp_ble_gap_start_advertising(&adv_params);
            if (ret != ESP_OK) {
                ESP_LOGE(GATTS_TAG, "Device disconnected, Restarting advertising failed!");
            }
            ESP_LOGI(GATTS_TAG, "%s -> Device disconnected! Restarting advertising! LINE %d", __func__, __LINE__);
            break;

        default:
            break;
    }
}


void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    esp_err_t ret;

    switch (event) {
        // * Start advertising following successfull configuration of the advertising data.
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ret = esp_ble_gap_start_advertising(&adv_params);
            if (ret == ESP_OK) {
                ESP_LOGI(GATTS_TAG, "%s -> Started advertising successfully! LINE %d", __func__, __LINE__);
            }
            break;

        // * Check whether the advertising was successfull.
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising start failure!");
            }
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TAG, "update connection params status = %d, conn_int = %d, latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
                
        case ESP_GAP_BLE_SEC_REQ_EVT:
            ESP_LOGI("GAP", "ESP_GAP_BLE_SEC_REQ_EVT");
            // Accept the security request
            ret = esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            if (ret != ESP_OK) {
                ESP_LOGE(GATTS_TAG, "Failed to accept security request!");
            }
            break;

        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            ESP_LOGI("GAP", "ESP_GAP_BLE_AUTH_CMPL_EVT");

            ESP_LOGI(GATTS_TAG, "address type = %d",   
                param->ble_security.auth_cmpl.addr_type);  
            ESP_LOGI(GATTS_TAG, "pair status = %s",  
                param->ble_security.auth_cmpl.success ? "success" : "fail");
            break;
        
        default:
            break;
    }
}


esp_err_t bt_init() {
    esp_err_t ret;

    // Initialize non-volatile storage to store non-volataile bluetooth data
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_config);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to initialize bluetooth controller", __func__);
        return ret;
    }

    esp_bt_controller_disable();
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to enable bluetooth controller", __func__);
        return ret;
    }

    // Initialize and enable bluedroid bluetooth stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to initialize bluedroid stack", __func__);
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(GATTS_TAG, "%s failed to enable bluedroid stack", __func__);
        return ret;
    }

    // Register callback functions (functions that perform logic in response to bluetooth events)
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return ret;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret); 
        return ret;
    }

    // Register gatts profile
    ret = esp_ble_gatts_app_register(CRANKIQ_APP_ID);
    if (ret != ESP_OK){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return ret;
    }

    // ****************** Enable security ******************
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;  // set the IO capability to No Input No Output. No user involvement ('Just Works')
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;  // bonding with peer device after authentication
    uint8_t key_size = 16;
    uint32_t passkey = 123456;  // to be hashed
    // Key distribution mask for the initiator: distributes encryption (LTK) and identity (IRK) keys.
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    // Key distribution mask for the responder: distributes encryption (LTK) and identity (IRK) keys.
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
    // ******************************************************

    return ESP_OK;
}

