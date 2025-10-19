/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"

// static void example_ble_mesh_client_cb(esp_ble_mesh_client_common_cb_event_t ev,
//                                        esp_ble_mesh_client_common_cb_param_t *p);

static esp_err_t send_vendor_msg(uint16_t dst, const uint8_t *data, uint16_t len);


#define TAG "EXAMPLE"

#define CID_ESP     0x02E5

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND      ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

#define VND_CLI_TIMEOUT_MS  4000

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

static uint16_t g_net_idx = 0x0000;
static uint16_t g_app_idx = 0x0000;
static uint16_t g_elem_addr = 0x0000;


static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 30),
    .relay = ESP_BLE_MESH_RELAY_ENABLED,    
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 30),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 5,
};

static esp_ble_mesh_model_t sig_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

// 2) Vendor op table (what SERVER can receive)
static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 1),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_op_t vnd_cli_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_STATUS, 1),  // min len 1 is fine
    ESP_BLE_MESH_MODEL_OP_END,
};

// 3) Vendor client op-pair
static const esp_ble_mesh_client_op_pair_t vnd_cli_op_pair[] = {
    { ESP_BLE_MESH_VND_MODEL_OP_SEND, ESP_BLE_MESH_VND_MODEL_OP_STATUS },
};



static esp_ble_mesh_client_t vnd_client = {
    .op_pair_size = ARRAY_SIZE(vnd_cli_op_pair),
    .op_pair      = vnd_cli_op_pair,
};


// 4) Client publish context
ESP_BLE_MESH_MODEL_PUB_DEFINE(vnd_cli_pub, 400, ROLE_NODE);

// 5) Vendor models array (client + server)
static esp_ble_mesh_model_t vnd_models[] = {
    // Vendor CLIENT (receives STATUS)
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_CLIENT,
                              vnd_cli_op, &vnd_cli_pub, &vnd_client),

    // Vendor SERVER (receives SEND)
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
                              vnd_op, NULL, NULL),
};


// static esp_ble_mesh_model_t root_models[] = {
//     ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),

//     // Vendor CLIENT (ID 0x0000) – can publish messages
//     ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_CLIENT,
//                               NULL, &vnd_cli_pub, &vnd_client),

//     // Vendor SERVER (ID 0x0001) – handles incoming OP_SEND
//     ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
//                               vnd_op, NULL, NULL),
// };



static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, sig_models, vnd_models),
};


static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    g_elem_addr = addr; 
    g_net_idx = net_idx;
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08" PRIx32, flags, iv_index);
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            g_net_idx = param->value.state_change.appkey_add.net_idx;
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND: {
    uint16_t mid = param->value.state_change.mod_app_bind.model_id;
    if (mid == ESP_BLE_MESH_VND_MODEL_ID_CLIENT || mid == ESP_BLE_MESH_VND_MODEL_ID_SERVER) {
        g_app_idx = param->value.state_change.mod_app_bind.app_idx;
        // Optional test:
        // uint8_t ping[] = {0x01,0x02,0x03};
        // send_vendor_msg(0x0003, ping, sizeof(ping));
    }
} break;

        default:
            break;
        }
    }
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT: {
        uint32_t op = param->model_operation.opcode;
        uint16_t len = param->model_operation.length;
        const uint8_t *p = param->model_operation.msg;
        const esp_ble_mesh_model_t *m = param->model_operation.model;
        uint16_t src = param->model_operation.ctx->addr;

        if (op == ESP_BLE_MESH_VND_MODEL_OP_SEND) {
            // Server side: got a SEND → reply with STATUS
            uint16_t echo_tid = 0;
            if (len >= 2)        echo_tid = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
            else if (len == 1)   echo_tid = p[0];

            ESP_LOGI(TAG, "[S] RECV SEND from 0x%04x len=%u", src, len);

            esp_err_t err = esp_ble_mesh_server_model_send_msg(
                (esp_ble_mesh_model_t *)m,
                param->model_operation.ctx,
                ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                sizeof(echo_tid), (uint8_t *)&echo_tid);
            if (err) ESP_LOGE(TAG, "STATUS send failed (%d)", err);
            return;
        }

        if (op == ESP_BLE_MESH_VND_MODEL_OP_STATUS) {
            // Client side: peer replied STATUS
            uint16_t val = 0;
            if (len >= 2)        val = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
            else if (len == 1)   val = p[0];

            ESP_LOGI(TAG, "[C] RECV STATUS from 0x%04x len=%u val=0x%04x", src, len, val);
            return;
        }
        break;
    }

    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            ESP_LOGE(TAG, "TX fail op 0x%06" PRIx32, param->model_send_comp.opcode);
        } else {
            ESP_LOGI(TAG, "TX ok   op 0x%06" PRIx32, param->model_send_comp.opcode);
        }
        break;

    default:
        break;
    }
}


static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    // // Register client callback BEFORE esp_ble_mesh_init()
    // esp_ble_mesh_register_client_callback(example_ble_mesh_client_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }

    board_led_operation(LED_G, LED_ON);
    ESP_LOGI(TAG, "BLE Mesh Node initialized");
    return ESP_OK;
}


void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    board_init();

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}

// static void example_ble_mesh_client_cb(esp_ble_mesh_client_common_cb_event_t ev,
//                                        esp_ble_mesh_client_common_cb_param_t *p) {
//     if (ev == ESP_BLE_MESH_CLIENT_MODEL_SEND_TIMEOUT_EVT) {
//         ESP_LOGW(TAG, "Client send timeout op 0x%06" PRIx32 " dst 0x%04x",
//                  p->model_send_timeout.opcode, p->model_send_timeout.ctx.addr);
//     }
// }

__attribute__((unused))
static esp_err_t send_vendor_msg(uint16_t dst, const uint8_t *data, uint16_t len) {
    esp_ble_mesh_msg_ctx_t ctx = {
        .net_idx = g_net_idx, .app_idx = g_app_idx, .addr = dst,
        .send_ttl = 5, .send_rel = true,
    };
    return esp_ble_mesh_client_model_send_msg(&vnd_models[0], &ctx,
           ESP_BLE_MESH_VND_MODEL_OP_SEND, len, (uint8_t*)data, 4000, true, false);
}


