#include "led.h"

#include <stdint.h>

#include "raw_stream.h"

#include "logging.h"

namespace {

constexpr ledc_timer_t kLedcTimer= LEDC_TIMER_1;
constexpr ledc_mode_t kLedcSpeedMode = LEDC_HIGH_SPEED_MODE;
constexpr ledc_channel_t kLedcChannel = LEDC_CHANNEL_0;
constexpr ledc_timer_bit_t kLedcDutyResolution = LEDC_TIMER_10_BIT;
constexpr uint32_t kLedcFrequency = 25000; // 5 kHz  Try to get to 25 kHz

ledc_channel_config_t default_channel_config() {
  ledc_channel_config_t channel_config = {};
  channel_config.speed_mode = kLedcSpeedMode;
  channel_config.intr_type = LEDC_INTR_DISABLE;
  channel_config.timer_sel = kLedcTimer;
  channel_config.duty = 0; // Set duty to 0%
  channel_config.hpoint = 0;
  return channel_config;
}

}  // namespace

const std::array<gpio_num_t, kNumColors> Led::led_gpio_list_{
  GPIO_NUM_25,
  GPIO_NUM_26,
  GPIO_NUM_27,
  GPIO_NUM_14,
  GPIO_NUM_13,
  GPIO_NUM_15,
};

const std::array<ledc_channel_t, kNumColors + 1> Led::channel_list_{
  LEDC_CHANNEL_0,
  LEDC_CHANNEL_1,
  LEDC_CHANNEL_2,
  LEDC_CHANNEL_3,
  LEDC_CHANNEL_4,
  LEDC_CHANNEL_5,
  LEDC_CHANNEL_6,
};

Led::Led() {
  ESP_LOGI(TAG, "Configuring led GPIO");
  uint64_t pin_bit_mask = 0;
  for (int i = 0; i < led_gpio_list_.size(); i++) {
    pin_bit_mask = pin_bit_mask | (1ULL << led_gpio_list_[i]);
  }
  static const gpio_config_t led_pins = {
    .pin_bit_mask = pin_bit_mask,
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
      .clk_cfg          = LEDC_USE_APB_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Install the hardware fade setup.
  ledc_fade_func_install(0);
}

void Led::flare(SongColor color) {
  ledc_channel_config_t channel_config = default_channel_config();
  int color_id = static_cast<int>(color);
  channel_config.channel = channel_list_[color_id];
  channel_config.gpio_num = led_gpio_list_[color_id];

  ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

  ledc_set_fade_step_and_start(
      kLedcSpeedMode,
      channel_config.channel,
      8191,  // target
      1000, // scale
      1, // cycle_num,
      LEDC_FADE_WAIT_DONE);

  ledc_set_fade_step_and_start(
      kLedcSpeedMode,
      channel_config.channel,
      500,  // target
      10, // scale
      1, // cycle_num,
      LEDC_FADE_NO_WAIT);
}

void Led::flare_all_and_follow() {
  ledc_channel_config_t channel_config = default_channel_config();
  channel_config.channel = channel_list_.back();

  for (gpio_num_t pin : led_gpio_list_) {
    channel_config.gpio_num = pin;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
  }

  flare_channel(channel_config.channel);
}

void Led::set_to_follow(SongColor color) {
  ledc_channel_config_t channel_config = default_channel_config();
  channel_config.channel = channel_list_.back();
  channel_config.gpio_num = led_gpio_list_[static_cast<int>(color)];
  ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

void Led::start_following(ringbuf_handle_t buf) {
  // Begin a 1khz timer that reads data off the ringbuf.
  follow_ringbuf_ = buf;
}

bool Led::on_led_intr_(const ledc_cb_param_t *param, void *user_arg) {
  Led* led = static_cast<Led*>(user_arg);
  if (param->event == LEDC_FADE_END_EVT) {
    xQueueSendFromISR(led->led_queue_, param, NULL);
  }
  return false;
}

void Led::flare_channel(ledc_channel_t channel) {
  ledc_set_fade_step_and_start(
      kLedcSpeedMode,
      channel,
      8191,  // target
      1000, // scale
      1, // cycle_num,
      LEDC_FADE_NO_WAIT);
}

void Led::led_task() {
  while(1) {
    LedCommand command;
    xQueueReceive(led_queue_, &command, portMAX_DELAY);

    ledc_set_fade_step_and_start(
        kLedcSpeedMode,
        command.channel,
        command.current_duty * 0.8,  // Taper to 80% of flare.
        1,   // scale
        100,    // cycle_num,
        LEDC_FADE_NO_WAIT);
  }
}
