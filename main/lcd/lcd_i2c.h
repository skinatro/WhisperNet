#pragma once
#include "driver/i2c.h"
#include "rom/ets_sys.h"

esp_err_t lcd_i2c_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_hz, uint8_t addr);
void      lcd_clear(void);
void      lcd_home(void);
void      lcd_set_cursor(uint8_t col, uint8_t row);
void      lcd_print(const char *s);
void      lcd_printf(const char *fmt, ...) __attribute__((format(printf,1,2)));
void      lcd_log(const char *fmt, ...)    __attribute__((format(printf,1,2)));
