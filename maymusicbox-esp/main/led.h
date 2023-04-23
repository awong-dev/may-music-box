#ifndef LED_H_
#define LED_H_

#include "song.h"

#include <array>

#include "driver/ledc.h"
#include "ringbuf.h"

class Led {
 public:
  Led();
  void flare(SongColor color);
  void flare_all_and_follow();

  void set_to_follow(SongColor color);
  void start_following(ringbuf_handle_t buf);

  // Follow ringbuf info.
  static inline constexpr int kFollowRateHz = 1000; // Window at 1000hz sample.
  ringbuf_handle_t follow_ringbuf() const { return follow_ringbuf_; }

 private:
  struct LedCommand {
    enum Action {
      FlareToMax,
      DimTo80,
      SampleRingbuf,
    } action = {};
    ledc_channel_t channel = {};
    uint32_t current_duty = {};

    LedCommand() {}
    LedCommand(Action action, ledc_channel_t channel, uint32_t currnt_duty)
      : action(action), channel(channel), current_duty(currnt_duty) {}
  };
  QueueHandle_t led_queue_ = xQueueCreate(10, sizeof(LedCommand));

  // Handle up to 96khz samples. Add padding of 3 samples just because.
  ringbuf_handle_t follow_ringbuf_ = rb_create(sizeof(int16_t), (96000 / kFollowRateHz) + 3);
  bool is_following_ = false;

  // Pin order follows SongColor int values.
  static DRAM_ATTR constexpr std::array<gpio_num_t, kNumColors> led_gpio_list_ = {
    GPIO_NUM_25,
    GPIO_NUM_26,
    GPIO_NUM_27,
    GPIO_NUM_14,
    GPIO_NUM_13,
    GPIO_NUM_15,
  };

  // Pin order follows SongColor int values. Last member is "audio" channel.
  static DRAM_ATTR constexpr std::array<ledc_channel_t, kNumColors + 1> channel_list_ = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_5,
    LEDC_CHANNEL_6,
  };

  static void led_task_thunk(void *param) {
    static_cast<Led*>(param)->led_task();
  }

  static bool IRAM_ATTR on_led_intr_(const ledc_cb_param_t *param, void *user_arg);
  static bool IRAM_ATTR on_follow_intr_(void *param);

  void flare_channel(ledc_channel_t channel);

  void led_task();
};

#endif /* LED_H_ */
