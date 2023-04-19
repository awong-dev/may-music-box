#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <atomic>

class AudioPlayer;
class Led;

enum class ButtonId {
  Red=0,
  Orange,
  Yellow,
  Green,
  Blue,
  Purple,
};

enum class ButtonEvent : uint32_t {
  Off = 0,
  Up = 2,
  On = 1,
  Down = 3,
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
  static constexpr int kNumButtons = 6;

  Buttons(AudioPlayer *player, Led* led);

 private:
  struct IdPinMap {
    ButtonId id;
    gpio_num_t gpio;
  };

  static const IdPinMap pin_id_map_[kNumButtons];
  static const uint64_t kLongPressUs = 30*1000*1000;

  QueueHandle_t wake_queue_ = xQueueCreate(1, sizeof(char));
  AudioPlayer* player_;
  Led* led_;

  // Used by the timer interrupt.
  uint32_t intr_history_[kNumButtons] = {};
  uint64_t intr_num_samples_ = {};
  ButtonState intr_old_button_state_ = {};

  // This just tells the system to wake up.
  QueueHandle_t sample_queue_ = xQueueCreate(3, sizeof(ButtonState));
  static void IRAM_ATTR on_interrupt(void *args); 
  void IRAM_ATTR wake();
  void IRAM_ATTR disable_interrupts() const;
  void enable_interrupts() const; 

  // Impelement the Hackaday debounce method in an interrupt.
  void start_sample_timer();
  void stop_sample_timer();
  static void IRAM_ATTR on_timer_interrupt(void* param);
  bool IRAM_ATTR is_on(ButtonId id);
  void IRAM_ATTR push_history_bit(ButtonId b, bool value);
  void IRAM_ATTR sample_once();


  // This reads the button state and sends commands until all buttons are
  // released.
  public:
  void process_buttons();
};

#endif  /* BUTTONS_H_ */
