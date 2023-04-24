#ifndef LED_H_
#define LED_H_

#include "song.h"

#include <array>
#include <atomic>

#include "driver/ledc.h"
#include "ringbuf.h"

class Led {
 public:
  Led();
  void flare(SongColor color);
  void dim_to_on(SongColor color);
  void flare_all_and_follow();

  void set_to_follow(SongColor color);
  void start_following();

  // Follow ringbuf info.
  static inline constexpr int kFollowRateHz = 100; // Window at 1000hz sample.
  ringbuf_handle_t follow_ringbuf() const { return follow_ringbuf_; }

 private:
  enum class Action : int {
    Nothing = 0,
    FlareToMax,
    FlareToMaxThenDim,
    DimToOn,
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

  // Pin order follows SongColor int values.
  static DRAM_ATTR constexpr std::array<gpio_num_t, kNumColors> led_gpio_list_ = {
    GPIO_NUM_25,
    GPIO_NUM_26,
    GPIO_NUM_27,
    GPIO_NUM_14,
    GPIO_NUM_13,
    GPIO_NUM_15,
  };

  // Pin order follows SongColor int values.
  static DRAM_ATTR constexpr std::array<ledc_channel_t, kNumColors> channel_list_ = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_5,
  };

  QueueHandle_t led_queue_ = xQueueCreate(10, sizeof(LedCommand));
  std::array<std::atomic<Action>, channel_list_.size()> channel_current_action_{
    Action::Nothing,
    Action::Nothing,
    Action::Nothing,
    Action::Nothing,
    Action::Nothing,
    Action::Nothing,
  };

  // Handle up to 96khz samples. Add padding of 3 samples just because.
  ringbuf_handle_t follow_ringbuf_ = rb_create(sizeof(int16_t), (96000 / kFollowRateHz) + 3);
  bool is_following_ = false;

  static void led_task_thunk(void *param) {
    static_cast<Led*>(param)->led_task();
  }

  static bool IRAM_ATTR on_led_intr_(const ledc_cb_param_t *param, void *user_arg);
  static bool IRAM_ATTR on_follow_intr_(void *param);

  void flare_channel(ledc_channel_t channel);

  void config_follow_timer();

  void led_task();
};

#endif /* LED_H_ */
