#include "application.h"
#include "assets/lang_config.h"
#include "audio_codecs/no_audio_codec.h"
#include "button.h"
#include "config.h"
#include "display/oled_display.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"
#include "power_save_timer.h"
#include "system_reset.h"
#include "wifi_board.h"

#include <driver/i2c_master.h>
#include <driver/rtc_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_sleep.h>
#include <wifi_station.h>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "CompactWifiBoard"

LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_14_1);

class CompactWifiBoard : public WifiBoard {
  private:
  i2c_master_bus_handle_t display_i2c_bus_;
  esp_lcd_panel_io_handle_t panel_io_ = nullptr;
  esp_lcd_panel_handle_t panel_ = nullptr;
  Display* display_ = nullptr;
  Button boot_button_;
  Button touch_button_;
  //   Button volume_up_button_;
  //   Button volume_down_button_;
  bool press_to_talk_enabled_ = false;
  PowerSaveTimer* power_save_timer_;

  void InitializePowerSaveTimer() {

    power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
    power_save_timer_->OnEnterSleepMode([this]() {
      ESP_LOGI(TAG, "Enabling sleep mode");
      auto display = GetDisplay();
      display->SetChatMessage("system", "");
      display->SetEmotion("sleepy");

      auto codec = GetAudioCodec();
      codec->EnableInput(false);
    });
    power_save_timer_->OnExitSleepMode([this]() {
      auto codec = GetAudioCodec();
      codec->EnableInput(true);

      auto display = GetDisplay();
      display->SetChatMessage("system", "");
      display->SetEmotion("neutral");
    });
    power_save_timer_->OnShutdownRequest([this]() {
      ESP_LOGI(TAG, "Shutting down");
      const gpio_num_t ext_wakeup_pin = GPIO_NUM_6;
      esp_sleep_enable_ext0_wakeup(ext_wakeup_pin, 0);
      rtc_gpio_pullup_en(ext_wakeup_pin);
      rtc_gpio_pulldown_dis(ext_wakeup_pin);

      esp_lcd_panel_disp_on_off(panel_, false); // 关闭显示

      esp_deep_sleep_start();
    });
    power_save_timer_->SetEnabled(true);
  }

  void InitializeDisplayI2c() {
    i2c_master_bus_config_t bus_config = {
      .i2c_port = (i2c_port_t)0,
      .sda_io_num = DISPLAY_SDA_PIN,
      .scl_io_num = DISPLAY_SCL_PIN,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags =
        {
                .enable_internal_pullup = 1,
                },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
  }

  void InitializeSsd1306Display() {
    // SSD1306 config
    esp_lcd_panel_io_i2c_config_t io_config = {
      .dev_addr = 0x3C,
      .on_color_trans_done = nullptr,
      .user_ctx = nullptr,
      .control_phase_bytes = 1,
      .dc_bit_offset = 6,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
      .flags =
        {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
                },
      .scl_speed_hz = 400 * 1000,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_, &io_config, &panel_io_));

    ESP_LOGI(TAG, "Install SSD1306 driver");
    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = -1;
    panel_config.bits_per_pixel = 1;

    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
      .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
    };
    panel_config.vendor_config = &ssd1306_config;

#ifdef SH1106
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
#else
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
#endif
    ESP_LOGI(TAG, "SSD1306 driver installed");

    // Reset the display
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
    if (esp_lcd_panel_init(panel_) != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize display");
      display_ = new NoDisplay();
      return;
    }

    // Set the display to on
    ESP_LOGI(TAG, "Turning display on");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

    display_ = new OledDisplay(panel_io_,
                               panel_,
                               DISPLAY_WIDTH,
                               DISPLAY_HEIGHT,
                               DISPLAY_MIRROR_X,
                               DISPLAY_MIRROR_Y,
                               {&font_puhui_14_1, &font_awesome_14_1});
  }

  void InitializeButtons() {
    boot_button_.OnClick([this]() {
      auto& app = Application::GetInstance();
      if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
        ResetWifiConfiguration();
      }
      app.ToggleChatState();
    });

    touch_button_.OnPressDown([this]() {
      power_save_timer_->WakeUp();
      Application::GetInstance().StartListening();
    });
    touch_button_.OnPressUp([this]() {
      Application::GetInstance().StopListening();
    });
  }

  // 物联网初始化，添加对 AI 可见设备
  void InitializeIot() {
    auto& thing_manager = iot::ThingManager::GetInstance();
    thing_manager.AddThing(iot::CreateThing("Speaker"));
    thing_manager.AddThing(iot::CreateThing("Lamp"));
  }

  public:
  CompactWifiBoard()
    : boot_button_(BOOT_BUTTON_GPIO)
    , touch_button_(TOUCH_BUTTON_GPIO)
  // , volume_up_button_(VOLUME_UP_BUTTON_GPIO)
  // , volume_down_button_(VOLUME_DOWN_BUTTON_GPIO)
  {
    InitializeDisplayI2c();
    InitializeSsd1306Display();
    InitializeButtons();
    InitializeIot();
    InitializePowerSaveTimer();
  }

  virtual Led* GetLed() override {
    static SingleLed led(BUILTIN_LED_GPIO);
    return &led;
  }

  virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
    static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE,
                                           AUDIO_OUTPUT_SAMPLE_RATE,
                                           AUDIO_I2S_SPK_GPIO_BCLK,
                                           AUDIO_I2S_SPK_GPIO_LRCK,
                                           AUDIO_I2S_SPK_GPIO_DOUT,
                                           AUDIO_I2S_MIC_GPIO_SCK,
                                           AUDIO_I2S_MIC_GPIO_WS,
                                           AUDIO_I2S_MIC_GPIO_DIN);
#else
    static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE,
                                          AUDIO_OUTPUT_SAMPLE_RATE,
                                          AUDIO_I2S_GPIO_BCLK,
                                          AUDIO_I2S_GPIO_WS,
                                          AUDIO_I2S_GPIO_DOUT,
                                          AUDIO_I2S_GPIO_DIN);
#endif
    return &audio_codec;
  }

  virtual Display* GetDisplay() override { return display_; }

  virtual void SetPowerSaveMode(bool enabled) override {
    if (!enabled) {
      power_save_timer_->WakeUp();
    }
    WifiBoard::SetPowerSaveMode(enabled);
  }
};

DECLARE_BOARD(CompactWifiBoard);
