#pragma once
#include <stdint.h>

// Init/notify
void bridge_gatts_init(void);
void bridge_notify(const uint8_t *data, uint16_t len);

// Register a callback to receive bytes written by the PC to 0xFFF2
typedef void (*bridge_rx_cb_t)(const uint8_t *data, uint16_t len);
void bridge_set_rx_cb(bridge_rx_cb_t cb);
