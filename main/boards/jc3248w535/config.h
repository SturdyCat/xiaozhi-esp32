#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>
#include <driver/spi_master.h>

#define BOOT_BUTTON_GPIO                GPIO_NUM_0

#define AUDIO_INPUT_SAMPLE_RATE         24000
#define AUDIO_OUTPUT_SAMPLE_RATE        24000
#define AUDIO_DEFAULT_OUTPUT_VOLUME     80

#define AUDIO_I2S_SPK_GPIO_MCLK         (GPIO_NUM_NC)
#define AUDIO_I2S_SPK_GPIO_BCLK         (GPIO_NUM_42)
#define AUDIO_I2S_SPK_GPIO_DOUT         (GPIO_NUM_41)
#define AUDIO_I2S_SPK_GPIO_WS           (GPIO_NUM_2)

#define AUDIO_I2S_MIC_GPIO_WS           (GPIO_NUM_5)
#define AUDIO_I2S_MIC_GPIO_SCK          (GPIO_NUM_6)
#define AUDIO_I2S_MIC_GPIO_DIN          (GPIO_NUM_7)

#define DISPLAY_WIDTH                   320
#define DISPLAY_HEIGHT                  480
#define DISPLAY_MIRROR_X                false
#define DISPLAY_MIRROR_Y                false
#define DISPLAY_SWAP_XY                 false

#define QSPI_LCD_H_RES                  (DISPLAY_WIDTH)
#define QSPI_LCD_V_RES                  (DISPLAY_HEIGHT)
#define QSPI_LCD_BIT_PER_PIXEL          (16)

#define QSPI_LCD_HOST                   (SPI2_HOST)
#define QSPI_PIN_NUM_LCD_CS             (GPIO_NUM_45)
#define QSPI_PIN_NUM_LCD_SCLK           (GPIO_NUM_47)
#define QSPI_PIN_NUM_LCD_DATA0          (GPIO_NUM_21)
#define QSPI_PIN_NUM_LCD_DATA1          (GPIO_NUM_48)
#define QSPI_PIN_NUM_LCD_DATA2          (GPIO_NUM_40)
#define QSPI_PIN_NUM_LCD_DATA3          (GPIO_NUM_39)
#define QSPI_PIN_NUM_LCD_RST            (GPIO_NUM_NC)
#define QSPI_PIN_NUM_LCD_BL             (GPIO_NUM_1)
#define QSPI_PIN_NUM_LCD_TE             (GPIO_NUM_38)

#define DISPLAY_OFFSET_X                0
#define DISPLAY_OFFSET_Y                0

#define TP_PORT                         (I2C_NUM_1)
#define TP_PIN_NUM_TP_SDA               (GPIO_NUM_4)
#define TP_PIN_NUM_TP_SCL               (GPIO_NUM_8)
#define TP_PIN_NUM_TP_RST               (GPIO_NUM_NC)
#define TP_PIN_NUM_TP_INT               (GPIO_NUM_3)
#define TP_I2C_CLK_SPEED_HZ             (400 * 1000)

#define DISPLAY_BACKLIGHT_PIN           QSPI_PIN_NUM_LCD_BL
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

// #define SD_MMC_D0 13
// #define SD_MMC_CLK 12
// #define SD_MMC_CMD 11
// #define BAT_ADC_PIN 5

#endif // _BOARD_CONFIG_H_
