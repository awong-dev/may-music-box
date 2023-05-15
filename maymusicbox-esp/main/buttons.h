#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "song.h"

#include <memory>

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
  explicit Buttons(Led* led);

  // This reads the button state and sends commands until all buttons are
  // released.
  void process_buttons();

 private:
  // Pin order follows SongColor int values.
  static constexpr inline uint64_t kLongPressUs = 30*1000*1000;

  static void IRAM_ATTR on_ulp_interrupt(void* param);

  static void on_play_done_thunk(void* param) {
    static_cast<Buttons*>(param)->on_play_done();
  }

  void on_play_done();

  std::unique_ptr<AudioPlayer> player_;
  Led* led_ = nullptr;
  SongColor currently_playing_{-1};

  QueueHandle_t button_state_queue_ = xQueueCreate(10, sizeof(ButtonState));
};

#endif  /* BUTTONS_H_ */
