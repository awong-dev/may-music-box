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
  void off_and_follow(SongColor color);
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
    // Special actions.
    Nothing = 0,
    FadeEnd,

    // Broadcast
    SampleRingbuf,

    // Single color commands.
    FlareToMax,
    FlareToMaxThenOffAndFollow,
    FollowRingbuf,
    DimToOffAndFollow,
    DimToGlow,
    OffAndFollow,
  };

  // Command executed by the main Led Task.  When a command is sent to the Led
  // task, the ledc hardware peripheral will be triggered to do the action. Once
  // the action is complete, the ledc peripheral interrupt will enqueue another
  // command for that action with is_command_ack=true.
  struct LedCommand {
    Action action = Action::Nothing;
    ledc_channel_t channel = {};

    uint32_t current_duty = {};
    bool is_command_ack = false;

    LedCommand() {}
    LedCommand(Action action, ledc_channel_t channel, uint32_t currnt_duty)
      : action(action), channel(channel), current_duty(currnt_duty) {}
    LedCommand(Action action, ledc_channel_t channel, uint32_t currnt_duty, bool is_command_ack)
      : action(action), channel(channel), current_duty(currnt_duty), is_command_ack(is_command_ack) {}
  };

  struct LedInfo {
    gpio_num_t pin;
    ledc_channel_t channel;
  };

  ledc_channel_t color_to_channel(SongColor color) {
    return led_info_[static_cast<int>(color)].channel;
  }

  static bool IRAM_ATTR on_led_intr(const ledc_cb_param_t *param, void *user_arg);
  static void IRAM_ATTR on_follow_intr(void *param);
  void config_follow_timer();
  void handle_command(const LedCommand& command);
  void handle_broadcast_command(const LedCommand& command);
  void send_command(Action action, ledc_channel_t channel, uint32_t current_duty);

  static void led_task_thunk(void *param) {
    static_cast<Led*>(param)->led_task();
  }

  void led_task();

  // Order follows SongColor int values.
  static DRAM_ATTR constexpr std::array<LedInfo, kNumColors> led_info_{{
    { GPIO_NUM_25, LEDC_CHANNEL_0 },
    { GPIO_NUM_26, LEDC_CHANNEL_1 },
    { GPIO_NUM_27, LEDC_CHANNEL_2 },
    { GPIO_NUM_14, LEDC_CHANNEL_3 },
    { GPIO_NUM_13, LEDC_CHANNEL_4 },
    { GPIO_NUM_15, LEDC_CHANNEL_5 }
  }};

  // Channel Information
  struct ChannelState {
    bool is_busy = false;
    bool follow_ringbuf = true;
    uint32_t current_duty = 0;
    Action next_action = Action::Nothing;
  };
  std::array<ChannelState, kNumColors> channel_state_;
  std::array<Action, kNumColors> current_action_{{
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
};

#endif /* LED_H_ */
