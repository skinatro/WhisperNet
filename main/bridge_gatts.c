#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt.h"
#include "esp_log.h"
#include <string.h>
#include "bridge_gatts.h"

#define BRIDGE_TAG "BRIDGE"

static uint16_t bridge_service_handle = 0;
static uint16_t bridge_char_notify_handle = 0; // 0xFFF1 (Notify)
static uint16_t bridge_char_write_handle  = 0; // 0xFFF2 (Write)
static esp_gatt_if_t bridge_gatt_if = 0xFF;
static uint16_t bridge_conn_id = 0xFFFF;
static bool bridge_is_connected = false;
static bridge_rx_cb_t rx_cb = NULL;

// UUIDs
static const uint16_t BRIDGE_SERVICE_UUID = 0xFFF0;
static const uint16_t BRIDGE_CHAR_NOTIFY  = 0xFFF1;
static const uint16_t BRIDGE_CHAR_WRITE   = 0xFFF2;

// Increase value length to allow MTU>23 later
#define BRIDGE_VAL_MAX 100

// GATT database
enum {
    IDX_SVC,

    IDX_CHAR_NOTIFY_DECL,
    IDX_CHAR_NOTIFY_VAL,
    IDX_CHAR_NOTIFY_CCCD,

    IDX_CHAR_WRITE_DECL,
    IDX_CHAR_WRITE_VAL,

    IDX_NB
};

static const esp_gatts_attr_db_t gatt_db[IDX_NB] = {
    // Primary Service
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&(uint16_t){ESP_GATT_UUID_PRI_SERVICE}, ESP_GATT_PERM_READ,
         sizeof(uint16_t), sizeof(uint16_t), (uint8_t*)&BRIDGE_SERVICE_UUID}
    },

    // 0xFFF1 Notify
    [IDX_CHAR_NOTIFY_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE}, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t*)&(uint8_t){ESP_GATT_CHAR_PROP_BIT_NOTIFY}}
    },
    [IDX_CHAR_NOTIFY_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&BRIDGE_CHAR_NOTIFY, ESP_GATT_PERM_READ,
         BRIDGE_VAL_MAX, 0, NULL}
    },
    [IDX_CHAR_NOTIFY_CCCD] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&(uint16_t){ESP_GATT_UUID_CHAR_CLIENT_CONFIG},
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), sizeof(uint16_t), (uint8_t*)&(uint16_t){0x0000}}
    },

    // 0xFFF2 Write (no response required; both perms okay)
    [IDX_CHAR_WRITE_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&(uint16_t){ESP_GATT_UUID_CHAR_DECLARE}, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t), (uint8_t*)&(uint8_t){ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR}}
    },
    [IDX_CHAR_WRITE_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&BRIDGE_CHAR_WRITE, ESP_GATT_PERM_WRITE,
         BRIDGE_VAL_MAX, 0, NULL}
    },
};

static void gap_event_handler(esp_gap_ble_cb_event_t e, esp_ble_gap_cb_param_t *p) {
    (void)e; (void)p; // Mesh handles adverts; no custom GAP here.
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        bridge_gatt_if = gatts_if;
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_NB, 0);
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK) {
            const uint16_t *h = param->add_attr_tab.handles;
            bridge_service_handle   = h[IDX_SVC];
            bridge_char_notify_handle = h[IDX_CHAR_NOTIFY_VAL];
            bridge_char_write_handle  = h[IDX_CHAR_WRITE_VAL];
            esp_ble_gatts_start_service(bridge_service_handle);
            ESP_LOGI(BRIDGE_TAG, "GATT up: svc=0x%04x notify=0x%04x write=0x%04x",
                     bridge_service_handle, bridge_char_notify_handle, bridge_char_write_handle);
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

    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == bridge_char_write_handle && rx_cb) {
            // Only unprepared/short writes considered here
            rx_cb(param->write.value, param->write.len);
        }
        break;

    default:
        break;
    }
}

void bridge_gatts_init(void) {
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(0x42); // arbitrary
}

void bridge_notify(const uint8_t *data, uint16_t len) {
    if (!bridge_is_connected || bridge_gatt_if == 0xFF || bridge_char_notify_handle == 0)
        return;
    if (len > BRIDGE_VAL_MAX) len = BRIDGE_VAL_MAX;
    esp_ble_gatts_send_indicate(bridge_gatt_if, bridge_conn_id, bridge_char_notify_handle, len, (uint8_t*)data, false);
}

void bridge_set_rx_cb(bridge_rx_cb_t cb) {
    rx_cb = cb;
}
