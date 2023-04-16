#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

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

enum class ButtonEvent {
  Up,
  Down,
  On,
  Off
};

// TODO: Incorrect static.

class Buttons {
 public:
  static constexpr int kNumButtons = 6;

  Buttons(AudioPlayer *player, Led* led);

  // Takes 32 samples of the button state, 1ms apart.
  void blocking_sample();

  bool all_off() const {
     return (history_[0] | history_[1] | history_[2] |
             history_[3] | history_[4] | history_[5]) == 0;
  }

  // Impelement the Hackaday debounce method.
  ButtonEvent get_event(ButtonId id);

 private:
  struct IdPinMap {
    ButtonId id;
    gpio_num_t gpio;
  };

  static const IdPinMap pin_id_map_[kNumButtons];
  static const uint64_t kLongPressUs = 30*1000*1000;

  uint32_t history_[kNumButtons] = {};
  QueueHandle_t wake_queue_ = xQueueCreate(1, sizeof(char));
  AudioPlayer* player_;
  Led* led_;

  // This just tells the system to wake up.
  static void IRAM_ATTR on_interrupt(void *args); 
  void IRAM_ATTR wake();
  void IRAM_ATTR disable_interrupts() const;
  void enable_interrupts() const; 

  void push_history_bit(ButtonId b, uint32_t value);
  void sample_once();

  // This reads the button state and sends commands until all buttons are
  // released.
  void process_buttons();
};

#endif  /* BUTTONS_H_ */
