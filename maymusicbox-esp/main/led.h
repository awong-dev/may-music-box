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

 private:
  struct LedCommand {
    enum {
      FlareToMax,
      DimTo80,
      SampleRingbuf,
    } command;
    ledc_channel_t channel;
    uint32_t current_duty;
  };
  QueueHandle_t led_queue_ = xQueueCreate(10, sizeof(LedCommand));
  ringbuf_handle_t follow_ringbuf_;

  // Pin order follows SongColor int values.
  static DRAM_ATTR const std::array<gpio_num_t, kNumColors> led_gpio_list_;

  // Pin order follows SongColor int values. Last member is "audio" channel.
  static DRAM_ATTR const std::array<ledc_channel_t, kNumColors + 1> channel_list_;

  static void led_task_thunk(void *param) {
    static_cast<Led*>(param)->led_task();
  }

  static bool on_led_intr_(const ledc_cb_param_t *param, void *user_arg);

  void flare_channel(ledc_channel_t channel);

  void led_task();
};

#endif /* LED_H_ */
