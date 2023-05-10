#include "led.h"

#include <stdint.h>
#include <math.h>

#include "esp_log.h"
#include "raw_stream.h"
#include "driver/timer.h"

static const char *TAG = "led";

namespace {

constexpr ledc_timer_t kLedcTimer= LEDC_TIMER_1;
constexpr ledc_mode_t kLedcSpeedMode = LEDC_HIGH_SPEED_MODE;
constexpr ledc_timer_bit_t kLedcDutyResolution = LEDC_TIMER_11_BIT;
constexpr int32_t kLedcFrequencyHz = 30000; // Flicker free per Waveform lighting is > 25khz. Use 30khz..
constexpr int32_t kMaxDuty = ((1UL << kLedcDutyResolution) - 1);

// How much of MaxDuty should a flare be?
constexpr float kFlarePercentage = 0.3;
constexpr int kFlareScale = 80;
constexpr float kGlowPercentage = 0.02;

ledc_channel_config_t default_channel_config() {
  ledc_channel_config_t channel_config = {};
  channel_config.speed_mode = kLedcSpeedMode;
  channel_config.intr_type = LEDC_INTR_DISABLE;
  channel_config.timer_sel = kLedcTimer;
  channel_config.duty = 0; // Set duty to 0%
  channel_config.hpoint = 0;
  return channel_config;
}

struct TsVal {
  int32_t ts;
  Led::FollowSample s;
};

constexpr int kLogEntries = 4000;
std::array<TsVal, kLogEntries> f_led_values;
int f_num_led_values = 0;

}  // namespace

const std::array<gpio_num_t, kNumColors> Led::led_gpio_list_;
const std::array<ledc_channel_t, kNumColors> Led::channel_list_;

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
      .freq_hz          = kLedcFrequencyHz,  // Set output frequency at 5 kHz
      .clk_cfg          = LEDC_USE_APB_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Setup all Led channels.
  for (int i = 0; i < kNumColors; i++) {
    ledc_channel_config_t channel_config = default_channel_config();
    channel_config.channel = channel_list_[i];
    channel_config.gpio_num = led_gpio_list_[i];
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
  }

  // Create 1kHz timer to sample ringbuf.
  config_follow_timer();

  // Install the hardware fade setup.
  ledc_fade_func_install(0);

  ledc_cbs_t cbs;
  cbs.fade_cb = &Led::on_led_intr;
  for (ledc_channel_t ch : channel_list_) {
    ledc_cb_register(kLedcSpeedMode, ch, &cbs, this);
  }
  // i2s_stream defaults to priority 23.
  xTaskCreate(&Led::led_task_thunk, "led_task", 4096, this, 20, NULL);
}

void Led::flare(SongColor color) {
  // TODO: Move to 0 first.
  LedCommand c(Action::FlareToMax, channel_list_[static_cast<int>(color)], 0);
  xQueueSend(led_queue_, &c, portMAX_DELAY);
}

void Led::dim_to_glow(SongColor color) {
  LedCommand c(Action::DimToGlow, channel_list_[static_cast<int>(color)], 0);
  xQueueSend(led_queue_, &c, portMAX_DELAY);
}

void Led::flare_all_and_follow() {
  for (auto&& channel : channel_list_) {
    LedCommand c(Action::FlareToMaxThenDim, channel, 0);
    xQueueSend(led_queue_, &c, portMAX_DELAY);
  }
}

void Led::config_follow_timer() {
  esp_timer_create_args_t timer_args = {};
  timer_args.callback = &Led::on_follow_intr;
  timer_args.arg = this;
  timer_args.name = "samplerb";
  timer_args.dispatch_method = ESP_TIMER_ISR;
  timer_args.skip_unhandled_events = true;

  esp_timer_create(&timer_args, &follow_timer_);
  int64_t rate_us = (1ULL * 10000000)/kFollowRateHz;
  ESP_LOGI(TAG, "Following every %lldus", rate_us);
  esp_timer_start_periodic(follow_timer_, (1ULL * 1000000)/kFollowRateHz);
}

void IRAM_ATTR Led::on_follow_intr(void *param) {
  Led* led = static_cast<Led*>(param);
  // Channel is ignored for SampleRingbuf. All channels are populated.
  LedCommand c(Action::SampleRingbuf, channel_list_.front(), 0);
  xQueueSendFromISR(led->led_queue_, &c, 0);
}

bool Led::on_led_intr(const ledc_cb_param_t *param, void *user_arg) {
  Led* led = static_cast<Led*>(user_arg);
  if (param->event == LEDC_FADE_END_EVT) {
    Action last_action =
      led->channel_current_action_[param->channel].exchange(Action::Nothing);
    if (last_action == Action::FlareToMaxThenDim) {
      LedCommand c(Action::DimToGlow, static_cast<ledc_channel_t>(param->channel), param->duty);
      xQueueSendFromISR(led->led_queue_, &c, 0);
      return true;
    }
  }
  return false;
}

void Led::flare_channel(ledc_channel_t channel) {
  LedCommand c(Action::FlareToMax, channel, 0);
  xQueueSend(led_queue_, &c, portMAX_DELAY);
}

void Led::led_task() {
  while(1) {
    LedCommand command;
    xQueueReceive(led_queue_, &command, portMAX_DELAY);
    if (command.action != Action::SampleRingbuf &&
        channel_current_action_[command.channel] != Action::Nothing) {
      ESP_LOGI(TAG, "Channel %d busy. Skipping command %d",
          command.channel, static_cast<int>(command.action));
      continue;
    }

    channel_current_action_[command.channel] = command.action;

    switch (command.action) {
      case Action::FlareToMax:
      case Action::FlareToMaxThenDim:
        ESP_ERROR_CHECK(ledc_set_fade_step_and_start(
            kLedcSpeedMode,
            command.channel,
            kMaxDuty * kFlarePercentage,  // target
            kFlareScale,
            1,
            LEDC_FADE_NO_WAIT));
      break;

      case Action::DimToGlow:
        ESP_ERROR_CHECK(ledc_set_fade_step_and_start(
            kLedcSpeedMode,
            command.channel,
            kMaxDuty * kGlowPercentage,
            1,
            15,
            LEDC_FADE_NO_WAIT));
      break;

      case Action::SampleRingbuf: {
        FollowSample s;

        // Degrade it.
        static int degrade_count = 0;
        static constexpr int kDegradeSlope = 10;
        int target_duty;
        if (degrade_count++ % kDegradeSlope == 0) {
          target_duty = std::max(0, cur_duty_ - 1);
          degrade_count = 0;
        } else {
          target_duty = cur_duty_;
        }
        
        if (rb_bytes_available(follow_ringbuf_) >= sizeof(s)) {
          rb_read(follow_ringbuf_, reinterpret_cast<char*>(&s), sizeof(s), 1 / portTICK_PERIOD_MS);
          if (f_num_led_values < f_led_values.size()) {
            f_led_values[f_num_led_values].ts = static_cast<int32_t>(esp_timer_get_time() / 1000);
            f_led_values[f_num_led_values].s = s;
            f_num_led_values++;
          }
          float percent = s.volume;
          assert(percent >= 0);

          // TODO: Fix math.
          percent = sqrt(percent);
          // Compress volume range to make LED brigher.
          percent /= std::numeric_limits<int16_t>::max()/8;
          percent = std::min(percent, 1.0f);

          int new_duty = kMaxDuty * percent;
          // It's a new peak so update.
          if (new_duty > target_duty) {
            target_duty = new_duty;
          }
        }

        static constexpr int kNumCyclesInSample = (kLedcFrequencyHz / kFollowRateHz);
        int scale = 1 + abs((target_duty - cur_duty_)) / (kNumCyclesInSample);

        if (scale != 1) {
          ESP_LOGI(TAG, "s:%d, d:%d, t:%d c:%d r:%d", scale, target_duty - cur_duty_, target_duty, cur_duty_, (kLedcFrequencyHz / kFollowRateHz));
        }
        for (ledc_channel_t ch : channel_list_) {
          ESP_ERROR_CHECK(ledc_set_fade_step_and_start(
                kLedcSpeedMode,
                ch,
                target_duty,
                scale,
                1,
                LEDC_FADE_NO_WAIT));
        }
        cur_duty_ = target_duty;
      }
      break;

      case Action::Nothing:
      break;
    }
  }
}

void Led::print_led_times() {
  float delta = 0;
  int n_d = 0;
  for (int i = 0; i < f_num_led_values; ++i) {
    const auto& x = f_led_values[i];
    int d = 0;
    if (i > 0) {
      d = x.ts - f_led_values[i-1].ts;
      delta += d;
      n_d++;
    }
  }
  float sum = delta / n_d;
  ESP_LOGI(TAG, "L: a.ts:%f, a.hz:%f", sum, 1000/sum);
}
