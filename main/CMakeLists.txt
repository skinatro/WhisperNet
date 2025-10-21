set(srcs "main.c"
         "board.c"
         "lcd/lcd_i2c.c")   
idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "." "lcd"
    REQUIRES driver nvs_flash
)
