#include "led.h"

#include <stdint.h>

#include "raw_stream.h"
#include "driver/ledc.h"

#include "logging.h"

constexpr ledc_timer_t kLedcTimer= LEDC_TIMER_0;
constexpr ledc_mode_t kLedcSpeedMode = LEDC_HIGH_SPEED_MODE;
constexpr ledc_channel_t kLedcChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_bit_t kLedcDutyResolution = LEDC_TIMER_13_BIT;
constexpr uint32_t kLedcFrequency = 5000; // 5 kHz

Led::Led(audio_element_handle_t raw_stream)
  : raw_stream_(raw_stream) {
  ESP_LOGI(TAG, "Configuring led GPIO");
  static const gpio_config_t led_pins = {
    .pin_bit_mask = (
        (1ULL << GPIO_NUM_13) |
        (1ULL << GPIO_NUM_14) |
        (1ULL << GPIO_NUM_15) |
        (1ULL << GPIO_NUM_25) |
        (1ULL << GPIO_NUM_26) |
        (1ULL << GPIO_NUM_27)
        ),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&led_pins));

  // Prepare and then apply the LEDC PWM timer configuration
  ledc_timer_config_t ledc_timer = {
      .speed_mode       = kLedcSpeedMode,
      .duty_resolution  = kLedcDutyResolution,
      .timer_num        = kLedcTimer,
      .freq_hz          = kLedcFrequency,  // Set output frequency at 5 kHz
      .clk_cfg          = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Prepare and then apply the LEDC PWM channel configuration
  ledc_channel_config_t ledc_channel = {
      .gpio_num       = GPIO_NUM_13,  // Pick one.
      .speed_mode     = kLedcSpeedMode,
      .channel        = kLedcChannel,
      .intr_type      = LEDC_INTR_DISABLE,
      .timer_sel      = kLedcTimer,
      .duty           = 0, // Set duty to 0%
      .hpoint         = 0,
      .flags          = {}
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  ledc_channel.gpio_num = GPIO_NUM_14;
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  ledc_channel.gpio_num = GPIO_NUM_15;
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  ledc_channel.gpio_num = GPIO_NUM_25;
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  ledc_channel.gpio_num = GPIO_NUM_26;
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  ledc_channel.gpio_num = GPIO_NUM_27;
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void Led::led_task() {
  static constexpr int kNumFrames = 10;
  static constexpr int kNumChannels = 2;
  static constexpr int kFrameSize = sizeof(int16_t) * kNumChannels; // 2 samples for stereo.
  int16_t samples[kNumFrames * kNumChannels];
  char* start = reinterpret_cast<char*>(&samples[0]);
  char* cur = reinterpret_cast<char*>(&samples[0]);
  char* end = cur + sizeof(samples);
  int num_frames = 0;
  int32_t brightness = 0;
  static constexpr int32_t kDecay = 10;
  static constexpr int32_t kMaxBrightness = 65535;
  static constexpr int kLedUpdatePerNSample = 200;
  int last_update_sample = 0;
  while (1) {
    int bytes_read = raw_stream_read(raw_stream_, cur, end - cur);
    if (bytes_read < 0) {
      ESP_LOGE(TAG, "Failed to read raw stream %d", bytes_read);
      continue;
    }
    cur += bytes_read;
    num_frames += (cur - start) / kFrameSize;
    for (int i = 0; i < num_frames; ++i) {
      brightness -= kDecay;
      if (brightness < 0) {
        brightness = 0;
      }
      // Do some math on the samples to produce a decaying magnitude.
      int32_t value = samples[i];
      value += samples[i + 1];
      value /= 2;
      brightness += value;
      if (brightness > kMaxBrightness) {
       brightness = kMaxBrightness;
      }
      last_update_sample++;
      if (last_update_sample > kLedUpdatePerNSample) {
        // TODO: Do update.
        last_update_sample = 0;
      }
    }
  }
}


