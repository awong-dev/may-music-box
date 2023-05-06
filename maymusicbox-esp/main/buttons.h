#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "song.h"

#include <array>
#include <atomic>

class AudioPlayer;
class Led;

enum class ButtonEvent : uint32_t {
  Off = 0,
  Down = 1,
  Up = 2,
  On = 3,
};


struct ButtonState {
  bool b0: 1;
  bool b1: 1;
  bool b2: 1;
  bool b3: 1;
  bool b4: 1;
  bool b5: 1;

  bool all_off() const {
    return !(b0 || b1 || b2 || b3 || b4 || b5);
  }

  bool operator!=(const ButtonState& other) const {
    return !(*this == other);
  }

  bool operator==(const ButtonState& other) const {
    return
      b0 == other.b0 &&
      b1 == other.b1 &&
      b2 == other.b2 &&
      b3 == other.b3 &&
      b4 == other.b4 &&
      b5 == other.b5;
  }
};

class Buttons {
 public:

  Buttons(AudioPlayer *player, Led* led);

 private:
  // Pin order follows SongColor int values.
  static const uint64_t kLongPressUs = 30*1000*1000;

  AudioPlayer* player_;
  Led* led_;

  // This just tells the system to wake up.
  QueueHandle_t sample_queue_ = xQueueCreate(10, sizeof(ButtonState));

  // Impelement the Hackaday debounce method in an interrupt.
  static void IRAM_ATTR on_ulp_interrupt(void* param);

  // This reads the button state and sends commands until all buttons are
  // released.
  public:
  void process_buttons();
};

#endif  /* BUTTONS_H_ */
