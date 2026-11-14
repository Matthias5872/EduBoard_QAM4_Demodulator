idf_component_register(
    SRCS
        ./eduboardADC/src/eduboard2_adc_esp32_s3.c

        ./eduboardLCD/src/eduboard2_lcd_esp32_s3.c
        ./eduboardLCD/src/lcdDriver.c
        ./eduboardLCD/src/st7789.c
        ./eduboardLCD/src/ili9488.c
        ./eduboardLCD/src/decode_png.c
        ./eduboardLCD/src/pngle.c
        ./eduboardLCD/src/decode_jpeg.c

        ./eduboardSpiffs/src/fontx.c
        ./eduboardSpiffs/src/eduboard2_spiffs.c

        ./eduboardInit/src/eduboard2_init.c

    INCLUDE_DIRS
        .
        eduboardADC
        eduboardLCD
        eduboardSpiffs
        eduboardInit

    REQUIRES
        driver
        esp_adc
        gpspi
        vfs
        spiffs
)