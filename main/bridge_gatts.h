#pragma once
#include <stdint.h>
void bridge_gatts_init(void);
void bridge_notify(const uint8_t *data, uint16_t len);
