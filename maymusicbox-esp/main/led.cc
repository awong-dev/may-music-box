#include "led.h"

#include <stdint.h>

#include "esp_log.h"
#include "raw_stream.h"
#include "driver/timer.h"

static const char *TAG = "led";

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

const std::array<gpio_num_t, kNumColors> Led::led_gpio_list_;
const std::array<ledc_channel_t, kNumColors + 1> Led::channel_list_;

Led::Led() {
  ESP_LOGI(TAG, "Configuring led GPIO");
  uint64_t pin_bit_mask = 0;
  for (gpio_num_t pin : led_gpio_list_) {
    pin_bit_mask = pin_bit_mask | (1ULL << pin);
  }
  static const gpio_config_t led_pins = {
    .pin_bit_mask = pin_bit_mask,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&led_pins));

  // Set up the Timer for all the PWMs.
  ledc_timer_config_t ledc_timer = {
      .speed_mode       = kLedcSpeedMode,
      .duty_resolution  = kLedcDutyResolution,
      .timer_num        = kLedcTimer,
      .freq_hz          = kLedcFrequency,  // Set output frequency at 5 kHz
      .clk_cfg          = LEDC_USE_APB_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Install the hardware fade setup.
//  ledc_fade_func_install(0);

  ledc_cbs_t cbs;
  cbs.fade_cb = &Led::on_led_intr_;
  for (ledc_channel_t ch : channel_list_) {
    ledc_cb_register(kLedcSpeedMode, ch, &cbs, this);
  }
//  xTaskCreate(&Led::led_task_thunk, "led_task", 4096, this, configMAX_PRIORITIES -1, NULL);
}

void Led::flare(SongColor color) {
  ledc_channel_config_t channel_config = default_channel_config();
  int color_id = static_cast<int>(color);
  channel_config.channel = channel_list_[color_id];
  channel_config.gpio_num = led_gpio_list_[color_id];

  ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
  flare_channel(channel_config.channel);
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

void Led::start_following() {
  is_following_ = true;

  // Begin a 1khz timer that reads data off the ringbuf.
  timer_config_t timer_config = {
      .alarm_en = TIMER_ALARM_EN,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_EN,
      .divider = 10,
  };
  ESP_ERROR_CHECK(timer_init(TIMER_GROUP_1, TIMER_1, &timer_config));

  timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0);

  /* Configure the alarm value and the interrupt on alarm. */
  static DRAM_ATTR constexpr int kTimerDivider = 10;
  timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, TIMER_BASE_CLK / kTimerDivider / kFollowRateHz);

  timer_enable_intr(TIMER_GROUP_1, TIMER_1);

  timer_isr_callback_add(TIMER_GROUP_1, TIMER_1, &Led::on_follow_intr_, this, 0);

//  timer_start(TIMER_GROUP_1, TIMER_1);
}

bool IRAM_ATTR Led::on_follow_intr_(void *param) {
  Led* led = static_cast<Led*>(param);
  LedCommand c(Action::SampleRingbuf, channel_list_.back(), 0);
  xQueueSendFromISR(led->led_queue_, &c, 0);
  return true;
}

bool Led::on_led_intr_(const ledc_cb_param_t *param, void *user_arg) {
  Led* led = static_cast<Led*>(user_arg);
  if (param->event == LEDC_FADE_END_EVT) {
    Action last_action =
      led->channel_current_action_[param->channel].exchange(Action::Nothing);
    if (last_action == Action::FlareToMax) {
      LedCommand c(Action::DimTo80, static_cast<ledc_channel_t>(param->channel), 0);
      xQueueSendFromISR(led->led_queue_, &c, 0);
//      return true;
    }
  }
  return true;
}

void Led::flare_channel(ledc_channel_t channel) {
  LedCommand c(Action::FlareToMax, channel, 0);
  xQueueSend(led_queue_, &c, portMAX_DELAY);
  taskYIELD();
}

void Led::led_task() {
  while(1) {
//    ESP_LOGI(TAG, "led Waiting for command");
    LedCommand command;
    xQueueReceive(led_queue_, &command, portMAX_DELAY);
    if (channel_current_action_[command.channel] != Action::Nothing) {
//      ESP_LOGI(TAG, "Channel %d busy. Skipping command %d",
//          command.channel, static_cast<int>(command.action));
      continue;
    }
    channel_current_action_[command.channel] = command.action;
//    ESP_LOGI(TAG, "Channel %d open. Sending command %d",
//        command.channel, static_cast<int>(command.action));
    switch (command.action) {
      case Action::FlareToMax:
        ESP_ERROR_CHECK(ledc_set_fade_step_and_start(
            kLedcSpeedMode,
            command.channel,
            1024,  // target
            500, // scale
            3, // cycle_num,
            LEDC_FADE_NO_WAIT));
      break;

      case Action::DimTo80:
        ESP_ERROR_CHECK(ledc_set_fade_step_and_start(
            kLedcSpeedMode,
            command.channel,
            command.current_duty * 0.8,  // Taper to 80% of flare.
            1,
            100,
            LEDC_FADE_NO_WAIT));
      break;

      case Action::SampleRingbuf:
        if (is_following_) {
          int16_t val;
          if (rb_bytes_available(follow_ringbuf_) > sizeof(val)) {
            rb_read(follow_ringbuf_, reinterpret_cast<char*>(&val), sizeof(val), 0);
            int32_t power = val;
            power *= power;
            float percent = power;
            // TODO: Fix math.
            percent /= std::numeric_limits<int32_t>::max();
            static constexpr int kMaxDuty = 1024;
            ESP_ERROR_CHECK(ledc_set_fade_step_and_start(
                kLedcSpeedMode,
                command.channel,
                kMaxDuty * percent,
                1000,  // TODO: Spread it over 1khz.
                1,
                LEDC_FADE_NO_WAIT));
          }
        }
      break;

      case Action::Nothing:
      break;
    }

  }
}
