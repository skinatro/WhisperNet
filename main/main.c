/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "lcd/lcd_i2c.h"

#include "ble_mesh_example_init.h"
#include "board.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "bridge_gatts.h"

#define TAG "EXAMPLE"

#define CID_ESP 0x02E5
#define GROUP_ADDR 0xC000

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT 0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER 0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

#define VND_CLI_TIMEOUT_MS 4000

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = {0x32, 0x10};

static uint16_t g_net_idx = 0x0000;
static uint16_t g_app_idx = ESP_BLE_MESH_KEY_UNUSED;
static uint16_t g_elem_addr = 0x0000;

static esp_ble_mesh_model_t *g_vnd_cli_model = NULL;
static esp_ble_mesh_model_t *g_vnd_srv_model = NULL;


void bridge_gatts_init(void);
void bridge_notify(const uint8_t *data, uint16_t len);


static void bridge_rx_to_mesh(const uint8_t *data, uint16_t len) {
    if (!g_vnd_srv_model) {
        ESP_LOGW(TAG, "Server model not found yet");
        return;
    }
    if (g_app_idx == ESP_BLE_MESH_KEY_UNUSED) {
        ESP_LOGW(TAG, "No AppKey index yet; receive a mesh msg once or bind AppKey to this model.");
        return;
    }
    if (!len) return;
    if (len > 60) len = 60;

    esp_ble_mesh_msg_ctx_t ctx = {0};
    ctx.net_idx  = g_net_idx;   // set in prov_complete or from last RX
    ctx.app_idx  = g_app_idx;   // from last RX/bind
    ctx.addr     = GROUP_ADDR;  // 0xC000
    ctx.send_ttl = 4;

    esp_err_t err = esp_ble_mesh_server_model_send_msg(
        g_vnd_srv_model, &ctx,
        ESP_BLE_MESH_VND_MODEL_OP_SEND,
        (uint16_t)len, (uint8_t*)data
    );
    if (err) ESP_LOGE(TAG, "server_model_send_msg failed (%d)", err);
    else     ESP_LOGI(TAG, "TX (SERVER) → 0x%04x len=%u", ctx.addr, len);
}



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
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                          1),  // min len 1 is fine
    ESP_BLE_MESH_MODEL_OP_END,
};

// 3) Vendor client op-pair
static const esp_ble_mesh_client_op_pair_t vnd_cli_op_pair[] = {
    {ESP_BLE_MESH_VND_MODEL_OP_SEND, ESP_BLE_MESH_VND_MODEL_OP_STATUS},
};

static esp_ble_mesh_client_t vnd_client = {
    .op_pair_size = ARRAY_SIZE(vnd_cli_op_pair),
    .op_pair = vnd_cli_op_pair,
};

// 4) Client publish context
ESP_BLE_MESH_MODEL_PUB_DEFINE(vnd_cli_pub, 400, ROLE_NODE);

static esp_ble_mesh_model_t vnd_models[] = {
    // Vendor CLIENT
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP,
                              ESP_BLE_MESH_VND_MODEL_ID_CLIENT,
                              vnd_cli_op,
                              &vnd_cli_pub,
                              &vnd_client),

    // Vendor SERVER
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP,
                              ESP_BLE_MESH_VND_MODEL_ID_SERVER,
                              vnd_op,
                              NULL,
                              NULL),
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

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags,
                          uint32_t iv_index) {
  g_elem_addr = addr;
  g_net_idx = net_idx;
  ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
  ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08" PRIx32, flags, iv_index);
}

static void example_ble_mesh_provisioning_cb(
    esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t* param) {
  switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
      ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d",
               param->prov_register_comp.err_code);
      break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
      ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d",
               param->node_prov_enable_comp.err_code);
      break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
      ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
               param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV
                   ? "PB-ADV"
                   : "PB-GATT");
      break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
      ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
               param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV
                   ? "PB-ADV"
                   : "PB-GATT");
      break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
      ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
      prov_complete(
          param->node_prov_complete.net_idx, param->node_prov_complete.addr,
          param->node_prov_complete.flags, param->node_prov_complete.iv_index);
      break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
      ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
      break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
      ESP_LOGI(TAG,
               "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d",
               param->node_set_unprov_dev_name_comp.err_code);
      break;
    default:
      ESP_LOGW(TAG, "Provisioning callback unhandled event: 0x%02x", event);
      break;
  }
}

static void example_ble_mesh_config_server_cb(
    esp_ble_mesh_cfg_server_cb_event_t event,
    esp_ble_mesh_cfg_server_cb_param_t* param) {
  if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
    switch (param->ctx.recv_op) {
      case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        g_net_idx = param->value.state_change.appkey_add.net_idx;
        ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
        ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                 param->value.state_change.appkey_add.net_idx,
                 param->value.state_change.appkey_add.app_idx);
        ESP_LOG_BUFFER_HEX("AppKey",
                           param->value.state_change.appkey_add.app_key, 16);
        break;

case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND: {
    if (param->value.state_change.mod_app_bind.company_id == CID_ESP) {
        uint16_t mid = param->value.state_change.mod_app_bind.model_id;
        if (mid == ESP_BLE_MESH_VND_MODEL_ID_SERVER) {
            g_app_idx = param->value.state_change.mod_app_bind.app_idx;
            ESP_LOGI(TAG, "Vendor SERVER bound to AppIdx 0x%04x", g_app_idx);
        }
    }
} break;
      default:
        ESP_LOGW(TAG, "Config server unhandled state change op: 0x%06x",
                 param->ctx.recv_op);
        break;
    }
  } else {
    ESP_LOGW(TAG, "Config server unhandled event: 0x%02x", event);
  }
}

static void example_ble_mesh_custom_model_cb(
    esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t* param) {

  switch (event) {

    case ESP_BLE_MESH_MODEL_OPERATION_EVT: {
      uint32_t op  = param->model_operation.opcode;
      uint16_t len = param->model_operation.length;
      const uint8_t *p = param->model_operation.msg;
      const esp_ble_mesh_model_t *m = param->model_operation.model;
      const esp_ble_mesh_msg_ctx_t *c = param->model_operation.ctx;

      if (op == ESP_BLE_MESH_VND_MODEL_OP_SEND) {
        // Cache AppKey idx so GUI-initiated TX can use it later
        g_app_idx = c->app_idx;

        // Drop self-loopback so GUI doesn't see its own message twice
        if (c->addr != g_elem_addr) {
          uint16_t mirror_len = (len > 90) ? 90 : len;
          bridge_notify(p, mirror_len);
        }

        uint16_t echo_tid = 0;
        if (len >= 2)      echo_tid = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
        else if (len == 1) echo_tid = p[0];

        esp_err_t err = esp_ble_mesh_server_model_send_msg(
            (esp_ble_mesh_model_t *)m, c,
            ESP_BLE_MESH_VND_MODEL_OP_STATUS, sizeof(echo_tid),
            (uint8_t *)&echo_tid);
        if (err) {
          ESP_LOGE(TAG, "STATUS send failed (%d)", err);
        }
        return;
      }

      if (op == ESP_BLE_MESH_VND_MODEL_OP_STATUS) {
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
      ESP_LOGW(TAG, "Custom model unhandled event: %d", event);
      break;
  }
}


static esp_err_t ble_mesh_init(void) {
  esp_err_t err;

  esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
  esp_ble_mesh_register_config_server_callback(
      example_ble_mesh_config_server_cb);
  esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

  err = esp_ble_mesh_init(&provision, &composition);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize mesh stack");
    return err;
  }

  g_vnd_srv_model = esp_ble_mesh_find_vendor_model(&elements[0], CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER);
if (!g_vnd_srv_model) {
    ESP_LOGE(TAG, "Could not find vendor SERVER model");
    return ESP_FAIL;
}

  err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV |
                                      ESP_BLE_MESH_PROV_GATT);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable mesh node");
    return err;
  }

  board_led_operation(LED_G, LED_ON);
  ESP_LOGI(TAG, "BLE Mesh Node initialized");
  return ESP_OK;
}

void app_main(void) {
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

  bridge_gatts_init();
  bridge_set_rx_cb(bridge_rx_to_mesh);
  ble_mesh_get_dev_uuid(dev_uuid);

lcd_i2c_init(I2C_NUM_0, (gpio_num_t)21, (gpio_num_t)22, 400000, 0x27);
lcd_clear();
lcd_printf("Mesh booting…");

  /* Initialize the Bluetooth Mesh Subsystem */
  err = ble_mesh_init();
  if (err) {
    ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
  }
}

