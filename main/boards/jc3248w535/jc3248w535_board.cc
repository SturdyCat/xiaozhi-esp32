
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_check.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_spd2010.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <iot_button.h>
#include <wifi_station.h>
#include <esp_lvgl_port.h>

#include "application.h"
#include "button.h"
#include "config.h"
#include "display/lcd_display.h"
#include "font_awesome_symbols.h"
#include "esp_lcd_axs15231b.h"
#include "iot/thing_manager.h"
#include "power_save_timer.h"
#include "wifi_board.h"
#include "audio_codecs/no_audio_codec.h"

#define TAG "jc3248w535_board"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

typedef struct
{
    SemaphoreHandle_t tp_intr_event; /*!< Semaphore for tp interrupt */
    lv_display_rotation_t rotate;    /*!< Rotation configuration for the display */
} touch_int_t;

class Jc3248w535Board: public WifiBoard
{
private:
    i2c_master_bus_handle_t i2c_bus_;
    LcdDisplay *display_;
    Button boot_button_;
    PowerSaveTimer *power_save_timer_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    esp_lcd_touch_handle_t touch_ = nullptr;

    void InitializePowerSaveTimer()
    {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode(
            [this]()
            {
                ESP_LOGI(TAG, "Enabling sleep mode");
                auto display = GetDisplay();
                display->SetChatMessage("system", "");
                display->SetEmotion("sleepy");
                GetBacklight()->SetBrightness(10);
            });
        power_save_timer_->OnExitSleepMode(
            [this]()
            {
                auto display = GetDisplay();
                display->SetChatMessage("system", "");
                display->SetEmotion("neutral");
                GetBacklight()->RestoreBrightness();
            });
        power_save_timer_->OnShutdownRequest(
            [this]()
            {
                ESP_LOGI(TAG, "Shutting down");
                // rtc_gpio_set_level(GPIO_NUM_21, 0);
                // // 启用保持功能，确保睡眠期间电平不变
                // rtc_gpio_hold_en(GPIO_NUM_21);
                esp_lcd_panel_disp_on_off(panel_, false); // 关闭显示
                esp_deep_sleep_start();
            });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c()
    {
        ESP_LOGI(TAG, "Initialize I2C bus");
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = TP_PORT,
            .sda_io_num = TP_PIN_NUM_TP_SDA,
            .scl_io_num = TP_PIN_NUM_TP_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags =
                {
                    .enable_internal_pullup = 1,
                },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
        i2c_master_dev_handle_t i2c_device_ = NULL;
        i2c_device_config_t i2c_device_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = ESP_LCD_TOUCH_IO_I2C_AXS15231B_ADDRESS,
            .scl_speed_hz = TP_I2C_CLK_SPEED_HZ,

        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_, &i2c_device_cfg, &i2c_device_));
    }

    static void touch_process_points_cb(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength,
                                        uint8_t *point_num, uint8_t max_point_num)
    {
        uint16_t tmp;
        touch_int_t *touch_handle = (touch_int_t *)tp->config.user_data;

        for ( int i = 0; i < *point_num; i++ )
        {
            if ( LV_DISPLAY_ROTATION_270 == touch_handle->rotate )
            {
                tmp = x[i];
                x[i] = tp->config.y_max - y[i];
                y[i] = tmp;
            }
            else if ( LV_DISPLAY_ROTATION_180 == touch_handle->rotate )
            {
                tmp = x[i];
                x[i] = tp->config.x_max - x[i];
                y[i] = tp->config.y_max - y[i];
            }
            else if ( LV_DISPLAY_ROTATION_90 == touch_handle->rotate )
            {
                tmp = x[i];
                x[i] = y[i];
                y[i] = tp->config.x_max - tmp;
            }
        }
        ESP_LOGI(TAG, "touch x:%d y:%d", *x, *y);
    }

    static void touch_interrupt_cb(esp_lcd_touch_handle_t tp)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        touch_int_t *touch_handle = (touch_int_t *)tp->config.user_data;

        xSemaphoreGiveFromISR(touch_handle->tp_intr_event, &xHigherPriorityTaskWoken);

        if ( xHigherPriorityTaskWoken )
        {
            portYIELD_FROM_ISR();
        }
    }

    static bool touch_sync_cb(void *arg)
    {
        assert(arg);
        bool touch_interrupt = false;
        touch_int_t *touch_handle = (touch_int_t *)arg;

        if ( touch_handle && touch_handle->tp_intr_event )
        {
            if ( xSemaphoreTake(touch_handle->tp_intr_event, 0) == pdTRUE )
            {
                touch_interrupt = true;
            }
        }
        else
        {
            touch_interrupt = true;
        }

        return touch_interrupt;
    }

    void InitializeAxs15231bTouchpad()
    {

        ESP_LOGI(TAG, "Initialize axs15231b touchpad");
        SemaphoreHandle_t tp_intr_event = NULL;
        touch_int_t *touch_ctx = NULL;

        /* Initialize touch */
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH,
            .y_max = DISPLAY_HEIGHT,
            .rst_gpio_num = TP_PIN_NUM_TP_RST, // Shared with LCD reset
            .int_gpio_num = TP_PIN_NUM_TP_INT,
            .levels =
                {
                    .reset = 0,
                    .interrupt = 0,
                },
            .flags =
                {
                    .swap_xy = 0,
                    .mirror_x = 0,
                    .mirror_y = 0,
                },
            .process_coordinates = touch_process_points_cb,
        };

        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        const esp_lcd_panel_io_i2c_config_t tp_io_config = {
            .dev_addr = ESP_LCD_TOUCH_IO_I2C_AXS15231B_ADDRESS,
            .control_phase_bytes = 1,
            .dc_bit_offset = 0,
            .lcd_cmd_bits = 8,
            .flags =
                {
                    .disable_control_phase = 1,
                },
            .scl_speed_hz = TP_I2C_CLK_SPEED_HZ,
        };
        esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle);
        esp_lcd_touch_new_i2c_axs15231b(tp_io_handle, &tp_cfg, &touch_);
    }

    void InitializeButtons()
    {
        boot_button_.OnClick(
            [this]()
            {
                ESP_LOGI(TAG, "boot button Click");
                auto &app = Application::GetInstance();
                if ( app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected() )
                {
                    ResetWifiConfiguration();
                }
                // gpio_set_level(BUILTIN_LED_GPIO, 1);
                app.ToggleChatState();
            });
    }

    void InitializeSpi()
    {
        ESP_LOGI(TAG, "Initialize QSPI bus");

        const spi_bus_config_t qspi_cfg = AXS15231B_PANEL_BUS_QSPI_CONFIG(
            QSPI_PIN_NUM_LCD_SCLK, QSPI_PIN_NUM_LCD_DATA0, QSPI_PIN_NUM_LCD_DATA1, QSPI_PIN_NUM_LCD_DATA2,
            QSPI_PIN_NUM_LCD_DATA3, DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t));
        ESP_ERROR_CHECK(spi_bus_initialize(QSPI_LCD_HOST, &qspi_cfg, SPI_DMA_CH_AUTO));
    }

    void InitializeAxs15231bDisplay()
    {
        ESP_LOGI(TAG, "Install panel IO");
        const esp_lcd_panel_io_spi_config_t io_config = AXS15231B_PANEL_IO_QSPI_CONFIG(QSPI_PIN_NUM_LCD_CS, NULL, NULL);
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)QSPI_LCD_HOST, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install axs15231b panel driver");
        const axs15231b_vendor_config_t vendor_config = {
            .h_res = DISPLAY_WIDTH,
            .v_res = DISPLAY_HEIGHT,
            .bb_size = DISPLAY_WIDTH * 10,
            .flags =
                {
                    .use_qspi_interface = 1,
                },
        };
        const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = QSPI_PIN_NUM_LCD_RST,
            .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB, // Implemented by LCD command `36h`
            .bits_per_pixel = QSPI_LCD_BIT_PER_PIXEL,   // Implemented by LCD command `3Ah` (16/18)
            .vendor_config = (void *)&vendor_config,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_axs15231b(panel_io_, &panel_config, &panel_));

        esp_lcd_panel_reset(panel_);
        esp_lcd_panel_init(panel_);
        esp_lcd_panel_swap_xy(panel_, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel_, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new SpiLcdDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X,
                                     DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                     {
                                         .text_font = &font_puhui_20_4,
                                         .icon_font = &font_awesome_20_4,
                                         .emoji_font = font_emoji_64_init(),
                                     });

        lv_display_t *lv_display = display_->GetLvDisplay();

        // lv_display_set_rotation(lv_display, LV_DISPLAY_ROTATION_90);

        /* Add touch input (for selected screen) */
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display,
            .handle = touch_,
        };

        lvgl_port_add_touch(&touch_cfg);
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot()
    {
        auto &thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Backlight"));
    }

public:
    Jc3248w535Board()
        : boot_button_(BOOT_BUTTON_GPIO)
    {
        ESP_LOGI(TAG, "Initialize jc3248w535 board");
        InitializeI2c();
        InitializeAxs15231bTouchpad();
        InitializeSpi();
        InitializeAxs15231bDisplay();
        InitializePowerSaveTimer();
        InitializeButtons();
        InitializeIot();
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec *GetAudioCodec() override
    {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                               AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_WS, AUDIO_I2S_SPK_GPIO_DOUT,
                                               AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display *GetDisplay() override
    {
        return display_;
    }

    virtual Backlight *GetBacklight() override
    {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual void SetPowerSaveMode(bool enabled) override
    {
        if ( !enabled )
        {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveMode(enabled);
    }
};

DECLARE_BOARD(Jc3248w535Board);
