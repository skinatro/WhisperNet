// lcd_i2c.c  (minimal PCF8574 + HD44780 4-bit mode)
#include "lcd_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include "rom/ets_sys.h"
#include "driver/i2c.h"

static i2c_port_t s_port;
static uint8_t    s_addr;       // 7-bit I2C
static uint8_t    s_backlight = 0x08; // P3 often controls backlight
static uint8_t    s_cols = 16, s_rows = 2;
static uint8_t    s_row_next = 0;     // for lcd_log()

// PCF8574 pins (common backpacks):
// P0=D4, P1=D5, P2=D6, P3=BL, P4=EN, P5=RW, P6=RS, P7=D7
#define RS  (1<<6)
#define RW  (1<<5)
#define EN  (1<<4)
static esp_err_t pcf8574_write(uint8_t v){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr<<1)|I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, v | s_backlight, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return err;
}
static void pulse_en(uint8_t data){
    pcf8574_write(data | EN);   // EN high
    esp_rom_delay_us(1);
    pcf8574_write(data & ~EN);  // EN low
    esp_rom_delay_us(50);
}
static void write4(uint8_t nib, uint8_t rs){
    uint8_t b = 0;
    // map nibble to P0..P2,P7 = D4..D7
    b |= (nib & 0x01) ? (1<<0) : 0;
    b |= (nib & 0x02) ? (1<<1) : 0;
    b |= (nib & 0x04) ? (1<<2) : 0;
    b |= (nib & 0x08) ? (1<<7) : 0;
    if (rs) b |= RS;
    pcf8574_write(b);
    pulse_en(b);
}
static void send(uint8_t val, uint8_t rs){
    write4(val >> 4, rs);
    write4(val & 0x0F, rs);
}
static void cmd(uint8_t c){ send(c, 0); if (c==0x01||c==0x02) vTaskDelay(pdMS_TO_TICKS(2)); }
static void data(uint8_t d){ send(d, 1); }

esp_err_t lcd_i2c_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_hz, uint8_t addr){
    s_port = port;
    s_addr = addr; // e.g. 0x27
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda, .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl, .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_hz,
    };
    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0));

    vTaskDelay(pdMS_TO_TICKS(50)); // power-on wait
    // 4-bit init sequence
    write4(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));
    write4(0x03, 0); esp_rom_delay_us(150);
    write4(0x03, 0); esp_rom_delay_us(150);
    write4(0x02, 0); // 4-bit
    // Function set: 2 lines, 5x8
    cmd(0x28);
    // Display on, cursor off, blink off
    cmd(0x0C);
    // Clear
    cmd(0x01); vTaskDelay(pdMS_TO_TICKS(2));
    // Entry mode
    cmd(0x06);
    return ESP_OK;
}
void lcd_clear(void){ cmd(0x01); vTaskDelay(pdMS_TO_TICKS(2)); }
void lcd_home(void){ cmd(0x02); vTaskDelay(pdMS_TO_TICKS(2)); }
void lcd_set_cursor(uint8_t col, uint8_t row){
    static const uint8_t row_off[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= s_rows) row = s_rows-1;
    cmd(0x80 | (row_off[row] + col));
}
void lcd_print(const char *s){ while(*s) data((uint8_t)*s++); }

static void vprintf_to_lcd(uint8_t row, const char *fmt, va_list ap){
    char buf[33]; // fits 16 cols safely; truncate
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (n < 0) return;
    buf[ (n > 16) ? 16 : n ] = 0;
    lcd_set_cursor(0, row);
    // clear line
    for (int i=0;i<16;i++) data(' ');
    lcd_set_cursor(0, row);
    lcd_print(buf);
}
void lcd_printf(const char *fmt, ...){
    va_list ap; va_start(ap, fmt);
    vprintf_to_lcd(0, fmt, ap);
    va_end(ap);
}
void lcd_log(const char *fmt, ...){
    // simple 2-line “ticker”: line0 newest, line1 previous
    static char last[17] = {0};
    char curr[33];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(curr, sizeof(curr), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    curr[(n>16)?16:n] = 0;

    // shift last -> line1, curr -> line0
    lcd_set_cursor(0,1);
    for(int i=0;i<16;i++) data(' ');
    lcd_set_cursor(0,1); lcd_print(last);

    memset(last, 0, sizeof(last));
    strncpy(last, curr, 16);

    lcd_set_cursor(0,0);
    for(int i=0;i<16;i++) data(' ');
    lcd_set_cursor(0,0); lcd_print(curr);
}
