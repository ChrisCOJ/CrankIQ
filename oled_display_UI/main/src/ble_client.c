#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "../include/ble_client.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_device.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"


#define DISPLAY_APP_ID          1

// We are interested to retrieve these value from the ble peripheral
static uint8_t cadence = 0;
static uint8_t speed = 0;

static char *remote_device_name = "CrankIQ";
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

static bool found_service = false;
static bool connect = false;

uint8_t CRANKIQ_SERVICE_UUID[16] = {
    0x34, 0x90, 0x15, 0x6b, 0xbb, 0x3e, 0x4d, 0x96, 
    0x96, 0xd3, 0x11, 0x42, 0x6d, 0xe5, 0xc5, 0x20
};

uint8_t CADENCE_SENSOR_CHAR_UUID[16] = {
    0x34, 0x91, 0x15, 0x6b, 0xbb, 0x3e, 0x4d, 0x96, 
    0x96, 0xd3, 0x11, 0x42, 0x6d, 0xe5, 0xc5, 0x20
};

uint8_t SPEED_KMH_CHAR_UUID[16] = {
    0x34, 0x92, 0x15, 0x6b, 0xbb, 0x3e, 0x4d, 0x96, 
    0x96, 0xd3, 0x11, 0x42, 0x6d, 0xe5, 0xc5, 0x20
};

static gattc_conn_parameters conn_parameters;

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};



uint8_t get_speed() {
    return speed;
}

uint8_t get_cadence() {
    return cadence;
}


void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch(event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            //the unit of the duration is second
            uint32_t duration = 30;
            esp_ble_gap_start_scanning(duration);
            break;
        }

        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            switch (scan_result->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                    ESP_LOG_BUFFER_HEX(GATTC_TAG, scan_result->scan_rst.bda, 6);
                    ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
                    adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
                    ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
                    ESP_LOG_BUFFER_CHAR(GATTC_TAG, adv_name, adv_name_len);
                    ESP_LOGI(GATTC_TAG, " ");
                    if (adv_name != NULL) {
                        if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                            // Note: If there are multiple devices with the same device name, the device may connect to an unintended one.
                            // It is recommended to change the default device name to ensure it is unique.
                            ESP_LOGI(GATTC_TAG, "searched device %s", remote_device_name);
                            if (connect == false) {
                                connect = true;
                                ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                                esp_ble_gap_stop_scanning();
                                esp_ble_gatt_creat_conn_params_t create_conn_params = {0};
                                memcpy(&create_conn_params.remote_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                                create_conn_params.remote_addr_type = scan_result->scan_rst.ble_addr_type;
                                create_conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
                                create_conn_params.is_direct = true;
                                create_conn_params.is_aux = false;
                                create_conn_params.phy_mask = 0x0;
                                esp_ble_gattc_enh_open(conn_parameters.gattc_if,
                                                       &create_conn_params);
                            }
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        }
        default:
            break;
    }
}


void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    switch(event) {
    case ESP_GATTC_REG_EVT:
        conn_parameters.gattc_if = gattc_if;
        /* Set scan parameters on app register */
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret) {
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;

    case ESP_GATTC_CONNECT_EVT: {
        /* Store connection parameters for later use */
        ESP_LOGI(GATTC_TAG, "Connected, conn_id %d, remote "ESP_BD_ADDR_STR"", param->connect.conn_id,
                    ESP_BD_ADDR_HEX(param->connect.remote_bda));
        conn_parameters.conn_id = param->connect.conn_id;
        memcpy(conn_parameters.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, param->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "Config MTU error, error code = %x", mtu_ret);
        }
        break;
    }

    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Service discover failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Service discover complete, conn_id %d", param->dis_srvc_cmpl.conn_id);
        /* Filter ble device discovery to those with specific services */
        esp_bt_uuid_t filter_service_uuid = {
            .len = ESP_UUID_LEN_128,
        };
        memcpy(filter_service_uuid.uuid.uuid128, CRANKIQ_SERVICE_UUID, 16);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &filter_service_uuid);
        break;

    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "Service search result, conn_id = %x, is primary service %d", param->search_res.conn_id, param->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d, end handle %d, current handle value %d", param->search_res.start_handle, param->search_res.end_handle, param->search_res.srvc_id.inst_id);

        /* Compare discovered service uuid with actual uuid and fetch characteristic handles if identical */
        int found_service_uuid = memcmp(param->search_res.srvc_id.uuid.uuid.uuid128, CRANKIQ_SERVICE_UUID, ESP_UUID_LEN_128);
        if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128 && found_service_uuid == 0) {
            ESP_LOGI(GATTC_TAG, "Service found");
            found_service = true;
            conn_parameters.service_start_handle = param->search_res.start_handle;
            conn_parameters.service_end_handle = param->search_res.end_handle;
        }
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT:
        /* Declare characteristic uuids we are interested in discovering and subscribing to */
        // esp_bt_uuid_t filter_cadence_char_uuid = {
        //     .len = ESP_UUID_LEN_128,
        // };
        // memcpy(filter_cadence_char_uuid.uuid.uuid128, CADENCE_SENSOR_CHAR_UUID, sizeof(CADENCE_SENSOR_CHAR_UUID));

        // esp_bt_uuid_t filter_speed_char_uuid = {
        //     .len = ESP_UUID_LEN_128,
        // };
        // memcpy(filter_speed_char_uuid.uuid.uuid128, SPEED_KMH_CHAR_UUID, sizeof(SPEED_KMH_CHAR_UUID));
        /* -------------------------------------------------------------------------------- */

        if (param->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "Service search failed, status %x", param->search_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Service search complete");
        if (found_service) {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                    param->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    conn_parameters.service_start_handle,
                                                                    conn_parameters.service_end_handle,
                                                                    0,
                                                                    &count);
            if (status != ESP_GATT_OK) {
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }

            if (count > 0) {
                ESP_LOGI(GATTC_TAG, "The number of discovered characteristics = %d", count);
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                    break;
                } else {
                    status = esp_ble_gattc_get_all_char(gattc_if,
                                                        param->search_cmpl.conn_id,
                                                        conn_parameters.service_start_handle,
                                                        conn_parameters.service_end_handle,
                                                        char_elem_result,
                                                        &count,
                                                        0);
                    if (status != ESP_GATT_OK) {
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                        free(char_elem_result);
                        char_elem_result = NULL;
                        break;
                    }

                    /* --- Find and store all characteristics that have the notification property --- */
                    int notifiable_char_count = 0;
                    for (int i = 0; i < count; ++i) {
                        if (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
                            ++notifiable_char_count;
                            conn_parameters.notifiable_char_handles[i] = char_elem_result[i].char_handle;
                        }
                    }
                    /* --- Register for notify to all characteristics that have notifications enabled --- */
                    for (int i = 0; i < notifiable_char_count; ++i) {
                        esp_ble_gattc_register_for_notify(gattc_if, conn_parameters.remote_bda, conn_parameters.notifiable_char_handles[i]);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            } else {
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        } else {
            ESP_LOGW(GATTC_TAG, "Desired service not found!");
        }
    break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        esp_bt_uuid_t notify_descr_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}
        };

        if (param->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", param->reg_for_notify.status);
        } else {
            uint16_t count = 0;
            uint16_t notify_enable = 1;

            for (int i = 0; i < sizeof(conn_parameters.notifiable_char_handles) / sizeof(uint16_t); ++i) {
                esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if, conn_parameters.conn_id,
                                                                            ESP_GATT_DB_DESCRIPTOR,
                                                                            conn_parameters.service_start_handle,
                                                                            conn_parameters.service_end_handle,
                                                                            conn_parameters.notifiable_char_handles[i], &count);
                if (ret_status != ESP_GATT_OK) {
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                }
                if (count > 0) {
                    descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                    if (!descr_elem_result) {
                        ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                    } else {
                        ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                            conn_parameters.conn_id,
                                                                            param->reg_for_notify.handle,
                                                                            notify_descr_uuid,
                                                                            descr_elem_result,
                                                                            &count);

                        if (ret_status != ESP_GATT_OK) {
                            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                        }

                        /* Every char has only one descriptor */
                        if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && 
                            descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {

                            ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                        conn_parameters.conn_id,
                                                                        descr_elem_result[0].handle,
                                                                        sizeof(notify_enable),
                                                                        (uint8_t *)&notify_enable,
                                                                        ESP_GATT_WRITE_TYPE_NO_RSP,
                                                                        ESP_GATT_AUTH_REQ_NONE);
                        }

                        if (ret_status != ESP_GATT_OK) {
                            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                        }

                        /* free descr_elem_result */
                        free(descr_elem_result);
                    }
                }
                else {
                    ESP_LOGE(GATTC_TAG, "decsr not found");
                }
            }
        }
        break;

    case ESP_GATTC_NOTIFY_EVT:
        if (param->notify.value_len != 1) {
            // ESP_LOGE(GATTC_TAG, "Invalid value sent");
            break;
        }

        if (param->notify.handle == conn_parameters.notifiable_char_handles[0]) {
            cadence = param->notify.value[0];
            // ESP_LOGI("Cadence (rpm)", "%d", cadence);
        } else {
            speed = param->notify.value[0];
            // ESP_LOGI("Speed (kmh)", "%d", speed);
        }
        break;

    default:
        break;
    }    
}


void bt_init() {
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed, error code = %x", __func__, ret);
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(DISPLAY_APP_ID);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x", __func__, ret);
    }
}