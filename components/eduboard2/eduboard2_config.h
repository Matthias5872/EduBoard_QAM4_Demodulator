#pragma once
#define EDUBOARD_CPU_BOARD_ESP32_S3

/* ADC: Empfangssignal auf AN0 / GPIO9 */
#define CONFIG_ENABLE_ADC_STREAMING
#define CONFIG_ENABLE_AN0
#ifdef CONFIG_ENABLE_ADC_STREAMING
    #define CONFIG_ADC_STREAMING_BUFFERSIZE 256
    #define ADC_STREAM_SAMPLERATE_US 50
#endif

/* Display */
#define CONFIG_ENABLE_LCD
#ifdef CONFIG_ENABLE_LCD
    #define CONFIG_LCD_ILI9488
    #define CONFIG_LCD_RESOLUTION_320x480
    #define CONFIG_USE_DIFFUPDATE
#endif

/* Fonts fuer die Display-Ausgabe */
#define CONFIG_ENABLE_SPIFFS