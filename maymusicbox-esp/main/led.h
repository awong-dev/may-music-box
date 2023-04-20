#ifndef LED_H_
#define LED_H_

#include "audio_element.h"

#include "song.h"

class Led {
 public:
  explicit Led(audio_element_handle_t raw_stream);
  void pulseActive(SongColor id) {}
  void followAudio(SongColor id) {}

 private:
  audio_element_handle_t raw_stream_;

  static void led_task_thunk(void *param) {
    static_cast<Led*>(param)->led_task();
  }

  void led_task();
};

#endif /* LED_H_ */
