#ifndef LED_H_
#define LED_H_

#include "song.h"

#include <array>
#include <atomic>

#include "driver/ledc.h"
#include "esp_timer.h"
#include "ringbuf.h"

class Led {
 public:
  Led();
  void flare_and_hold(SongColor color);
  void dim_to_glow(SongColor color);
  void flare_all_and_follow();

  void print_led_times();

  // Follow ringbuf info.
  static inline constexpr int kFollowRateHz = 1000; // Window at 1000hz sample.
  struct FollowSample {
    int16_t volume;
  };
  ringbuf_handle_t follow_ringbuf() const { return follow_ringbuf_; }

 private:
  enum class Action : int {
    Nothing = 0,
    FlareToMax,
    FlareToMaxThenOff,
    DimToOff,
    DimToGlow,
    SampleRingbuf,
  };

  struct LedCommand {
    Action action = Action::Nothing;
    ledc_channel_t channel = {};
    uint32_t current_duty = {};

    LedCommand() {}
    LedCommand(Action action, ledc_channel_t channel, uint32_t currnt_duty)
      : action(action), channel(channel), current_duty(currnt_duty) {}
  };

  struct LedInfo {
    gpio_num_t pin;
    ledc_channel_t channel;
  };

  // Order follows SongColor int values.
  static DRAM_ATTR constexpr std::array<LedInfo, kNumColors> led_info_{{
    { GPIO_NUM_25, LEDC_CHANNEL_0 },
    { GPIO_NUM_26, LEDC_CHANNEL_1 },
    { GPIO_NUM_27, LEDC_CHANNEL_2 },
    { GPIO_NUM_14, LEDC_CHANNEL_3 },
    { GPIO_NUM_13, LEDC_CHANNEL_4 },
    { GPIO_NUM_15, LEDC_CHANNEL_5 }
  }};

  ledc_channel_t color_to_channel(SongColor color) {
    return led_info_[static_cast<int>(color)].channel;
  }

  // Command Queue information.
  std::array<std::atomic<Action>, kNumColors> current_action_{{
    Action::Nothing,
    Action::Nothing,
    Action::Nothing,
    Action::Nothing,
    Action::Nothing,
    Action::Nothing
  }};

  QueueHandle_t command_queue_ = xQueueCreate(10, sizeof(LedCommand));

  // Handle up to 96khz power values. Add padding of 3 values just because.
  ringbuf_handle_t follow_ringbuf_ = rb_create(sizeof(FollowSample), (96000 / kFollowRateHz) + 3);
  int cur_duty_ = 0;
  esp_timer_handle_t follow_timer_ = nullptr;

  static void led_task_thunk(void *param) {
    static_cast<Led*>(param)->led_task();
  }

  static bool IRAM_ATTR on_led_intr(const ledc_cb_param_t *param, void *user_arg);
  static void IRAM_ATTR on_follow_intr(void *param);

  void flare_channel(ledc_channel_t channel);

  void config_follow_timer();

  void led_task();
};

#endif /* LED_H_ */
