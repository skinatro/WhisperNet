// bridge_gatts.c — minimal notify-only GATT to mirror mesh payloads
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt.h"
#include "esp_log.h"

#define BRIDGE_TAG "BRIDGE"
#define GATT_IF_NONE 0xFF

static uint16_t bridge_service_handle = 0;
static uint16_t bridge_char_handle = 0;
static esp_gatt_if_t bridge_gatt_if = GATT_IF_NONE;
static uint16_t bridge_conn_id = 0xFFFF;
static bool bridge_is_connected = false;

static const uint16_t BRIDGE_SERVICE_UUID = 0xFFF0;
static const uint16_t BRIDGE_CHAR_UUID    = 0xFFF1;

static const esp_gatts_attr_db_t gatt_db[] = {
    // Primary Service
    [0] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&(uint16_t){ESP_GATT_UUID_PRI_SERVICE}, ESP_GATT_PERM_READ,
         sizeof(uint16_t), sizeof(uint16_t), (uint8_t*)&BRIDGE_SERVICE_UUID}
    },
    // Characteristic Declaration
    [1] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE}, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t*)&(uint8_t){ESP_GATT_CHAR_PROP_BIT_NOTIFY}}
    },
    // Characteristic Value (Notify)
    [2] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&BRIDGE_CHAR_UUID, ESP_GATT_PERM_READ,
         20, 0, NULL}
    },
    // Client Characteristic Configuration Descriptor (CCCD)
    [3] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&(uint16_t){ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), sizeof(uint16_t), (uint8_t*)&(uint16_t){0x0000}}
    },
};

static void gap_event_handler(esp_gap_ble_cb_event_t e, esp_ble_gap_cb_param_t *p) {
    // no-op; mesh handles advertising. We don't start our own adverts.
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        bridge_gatt_if = gatts_if;
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, sizeof(gatt_db)/sizeof(gatt_db[0]), 0);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK) {
            bridge_service_handle = param->add_attr_tab.handles[0];
            bridge_char_handle    = param->add_attr_tab.handles[2];
            esp_ble_gatts_start_service(bridge_service_handle);
            ESP_LOGI(BRIDGE_TAG, "Bridge GATT up: svc=0x%04x char=0x%04x",
                     bridge_service_handle, bridge_char_handle);
        } else {
            ESP_LOGE(BRIDGE_TAG, "Attr table create failed 0x%x", param->add_attr_tab.status);
        }
        break;
    case ESP_GATTS_CONNECT_EVT:
        bridge_conn_id = param->connect.conn_id;
        bridge_is_connected = true;
        ESP_LOGI(BRIDGE_TAG, "Central connected, conn_id=%u", bridge_conn_id);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        bridge_is_connected = false;
        bridge_conn_id = 0xFFFF;
        ESP_LOGI(BRIDGE_TAG, "Central disconnected");
        break;
    default:
        break;
    }
}

void bridge_gatts_init(void) {
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(0x42); // arbitrary APP ID
}

// Call this to push a small payload (<= 20 bytes) to the PC
void bridge_notify(const uint8_t *data, uint16_t len) {
    if (!bridge_is_connected || bridge_gatt_if == GATT_IF_NONE || bridge_char_handle == 0)
        return;
    esp_ble_gatts_send_indicate(bridge_gatt_if, bridge_conn_id, bridge_char_handle, len, (uint8_t*)data, false);
}
