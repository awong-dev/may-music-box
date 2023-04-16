#ifndef LED_H_
#define LED_H_

#include "audio_element.h"

class Led {
 public:
  explicit Led(audio_element_handle_t raw_stream) : raw_stream_(raw_stream) {}
  void pin_to_max(int id);
  void unpin(int id);

 private:
  audio_element_handle_t raw_stream_;

  static void led_task_thunk(void *param) {
    static_cast<Led*>(param)->led_task();
  }

  void led_task();
};

#endif /* LED_H_ */
