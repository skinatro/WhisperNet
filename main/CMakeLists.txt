set(srcs
    "main.c"
    "board.c"
    "lcd/lcd_i2c.c"
    "bridge_gatts.c"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "." "lcd"
    REQUIRES driver nvs_flash
)
