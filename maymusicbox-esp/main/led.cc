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
constexpr float kFlarePercentage = 0.9;
constexpr int kFlareScale = 80;
constexpr float kGlowPercentage = 0.7;

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

const std::array<Led::LedInfo, kNumColors> Led::led_info_;

Led::Led() {
  ESP_LOGI(TAG, "Configuring led GPIO");
  uint64_t pin_bit_mask = 0;
  for (auto& info : led_info_) {
    pin_bit_mask = pin_bit_mask | (1ULL << info.pin);
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
    auto& info = led_info_[i];
    ESP_LOGI(TAG, "Channel i:%d n:%d", i, info.channel);
    channel_config.channel = info.channel;
    channel_config.gpio_num = info.pin;
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
  }

  // Create 1kHz timer to sample ringbuf.
  config_follow_timer();

  // Install the hardware fade setup.
  ledc_fade_func_install(0);

  ledc_cbs_t cbs;
  cbs.fade_cb = &Led::on_led_intr;
  for (auto& info : led_info_) {
    ledc_cb_register(kLedcSpeedMode, info.channel, &cbs, this);
  }
  // i2s_stream defaults to priority 23.
  xTaskCreatePinnedToCore(&Led::led_task_thunk, "led_task", 4096, this, 20, NULL, 0);
}

void Led::flare_and_hold(SongColor color) {
  LedCommand c(Action::FlareToMax, color_to_channel(color), 0);
  xQueueSend(command_queue_, &c, portMAX_DELAY);
}

void Led::dim_to_glow(SongColor color) {
  LedCommand c(Action::DimToGlow, color_to_channel(color), 0);
  xQueueSend(command_queue_, &c, portMAX_DELAY);
}

void Led::off_and_follow(SongColor color) {
  LedCommand c(Action::OffAndFollow, color_to_channel(color), 0);
  xQueueSend(command_queue_, &c, portMAX_DELAY);
}

void Led::flare_all_and_follow() {
  for (auto&& info : led_info_) {
    LedCommand c(Action::FlareToMaxThenOffAndFollow, info.channel, 0);
    xQueueSend(command_queue_, &c, portMAX_DELAY);
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
  LedCommand c(Action::SampleRingbuf, LEDC_CHANNEL_MAX, 0);
  xQueueSendFromISR(led->command_queue_, &c, 0);
}

bool Led::on_led_intr(const ledc_cb_param_t *param, void *user_arg) {
  Led* led = static_cast<Led*>(user_arg);
  if (param->event == LEDC_FADE_END_EVT) {
    LedCommand c(Action::FadeEnd, static_cast<ledc_channel_t>(param->channel), param->duty);
    xQueueSendFromISR(led->command_queue_, &c, 0);
    return true;
  }
  return false;
}

void Led::led_task() {
  while(1) {
    LedCommand command;
    xQueueReceive(command_queue_, &command, portMAX_DELAY);
    handle_command(command);
  }
}

void Led::handle_command(const LedCommand& command) {
  auto& state = channel_state_[command.channel];

  // Handle the FadeEnd event first because it changes the
  // busy state for the channel.
  if (command.action == Action::FadeEnd) {
    state.current_duty = command.current_duty;
    if (state.next_action != Action::Nothing) {
       send_command(state.next_action, command.channel, command.current_duty);
    } else {
       state.is_busy = false;
    }
    return;
  }

  // Reenqueue command if target channel is busy.
  if (command.channel == LEDC_CHANNEL_MAX) {
    handle_broadcast_command(command);
    return;
  }

  bool should_skip_fade = state.is_busy;

  state.next_action = Action::Nothing;
  state.is_busy = true;
  state.follow_ringbuf = false;
  switch (command.action) {
    case Action::FollowRingbuf:
      // Preserve prior is_busy state.
      state.is_busy = should_skip_fade;
      state.follow_ringbuf = true;
      break;

    case Action::FlareToMaxThenOffAndFollow:
      state.next_action = Action::DimToOffAndFollow;
      // Fall through.

    case Action::FlareToMax:
      ESP_LOGE(TAG, " flare: %d, f:%d b:%d", command.channel, state.follow_ringbuf, state.is_busy);
      ESP_ERROR_CHECK(ledc_set_fade_step_and_start(
          kLedcSpeedMode,
          command.channel,
          kMaxDuty * kFlarePercentage,  // target
          kFlareScale,
          10,
          LEDC_FADE_NO_WAIT));
      break;

    case Action::DimToGlow:
      ESP_ERROR_CHECK(ledc_set_fade_step_and_start(
          kLedcSpeedMode,
          command.channel,
          kMaxDuty * kGlowPercentage,
          1,
          10,
          LEDC_FADE_NO_WAIT));
      break;

    case Action::OffAndFollow:
      state.next_action = Action::FollowRingbuf;
      ESP_ERROR_CHECK(ledc_set_fade_step_and_start(
          kLedcSpeedMode,
          command.channel,
          0,
          80,
          1,
          LEDC_FADE_NO_WAIT));
      break;

    case Action::DimToOffAndFollow:
      state.next_action = Action::FollowRingbuf;
      ESP_ERROR_CHECK(ledc_set_fade_step_and_start(
          kLedcSpeedMode,
          command.channel,
          0,
          1,
          100,
          LEDC_FADE_NO_WAIT));
      break;

    default:
      break;
  }
}

void Led::handle_broadcast_command(const LedCommand& command) {
  if (command.action == Action::SampleRingbuf) {
    FollowSample s;

    int new_duty = -1;
    if (rb_bytes_available(follow_ringbuf_) >= sizeof(s)) {
      rb_read(follow_ringbuf_, reinterpret_cast<char*>(&s), sizeof(s), 1 / portTICK_PERIOD_MS);
      if (f_num_led_values < f_led_values.size()) {
        f_led_values[f_num_led_values].ts = static_cast<int32_t>(esp_timer_get_time() / 1000);
        f_led_values[f_num_led_values].s = s;
        f_num_led_values++;
      }
      float percent = s.volume;

#define X_FOLLOW 2
#if X_FOLLOW == 1
      percent = sqrt(percent);
      percent /= sqrt(std::numeric_limits<int16_t>::max());
#elif X_FOLLOW == 2
      percent /= std::numeric_limits<int16_t>::max();
      percent *= 2;
      percent = std::min(percent, 1.0f);
#endif

      new_duty = kMaxDuty * percent;
    }

    // Calculate degraded duty.
    static constexpr int kNumCyclesInSample = (kLedcFrequencyHz / kFollowRateHz);

    static constexpr int kDegradeSlope = 5;  // Smaller is faster.
    static int degrade_count = 0;
    degrade_count++;

    for (int i = 0; i < led_info_.size(); ++i) {
      // TODO: how to come back to the ringbuf level quickly?
      if (channel_state_[i].is_busy || !channel_state_[i].follow_ringbuf) {
        continue;
      }

      // Calculated degraded duty.
      int target_duty = channel_state_[i].current_duty;
      if (degrade_count % kDegradeSlope == 0) {
        if (target_duty > 0) {
          //target_duty--;
          //target_duty *= 0.75;
          // TODO: Max flare is still slow to return.
          target_duty -= sqrt(target_duty) / 8 + 1;
        }
        degrade_count = 0;
      }

      // New peak.
      if (new_duty > target_duty) {
        target_duty = new_duty;
      }

      // Interpolate the scale.
      int scale = 1;
      uint32_t current_duty = channel_state_[i].current_duty;
      if (target_duty > current_duty) {
        scale += (target_duty - current_duty) / kNumCyclesInSample;
      } else {
        scale += (current_duty - target_duty) / kNumCyclesInSample;
      }

      ESP_ERROR_CHECK(ledc_set_fade_step_and_start(
            kLedcSpeedMode,
            led_info_[i].channel,
            target_duty,
            scale,
            1,
            LEDC_FADE_NO_WAIT));
    }
  }
}

void Led::send_command(Action action, ledc_channel_t channel, uint32_t current_duty) {
  LedCommand c(action, channel, current_duty);
  xQueueSend(command_queue_, &c, portMAX_DELAY);
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
