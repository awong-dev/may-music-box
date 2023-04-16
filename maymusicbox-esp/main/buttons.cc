#include "buttons.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio_player.h"
#include "led.h"
#include "logging.h"

Buttons::Buttons(AudioPlayer *player, Led* led) : player_(player), led_(led) {
  ESP_LOGI(TAG, "Configuring button GPIO");
  static const gpio_config_t button_pullup_pins = {
    .pin_bit_mask = (
        (1ULL << GPIO_NUM_32) |
        (1ULL << GPIO_NUM_33)
        ),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_LOW_LEVEL
  };
  ESP_ERROR_CHECK(gpio_config(&button_pullup_pins));

  static const gpio_config_t button_pins = {
    .pin_bit_mask = (
        (1ULL << GPIO_NUM_34) |
        (1ULL << GPIO_NUM_35) |
        (1ULL << GPIO_NUM_36) |
        (1ULL << GPIO_NUM_39)
        ),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_LOW_LEVEL
  };
  ESP_ERROR_CHECK(gpio_config(&button_pins));

  gpio_isr_register(
      &Buttons::on_interrupt,
      this,
      ESP_INTR_FLAG_LEVEL1 |
      ESP_INTR_FLAG_SHARED |
      ESP_INTR_FLAG_EDGE |
      ESP_INTR_FLAG_IRAM,
      NULL);

  ESP_LOGI(TAG, "Enabling button interrupts");
  enable_interrupts();
}

// Takes 32 samples of the button state, 1ms apart.
void Buttons::blocking_sample() {
   for (int i = 0; i < 32; i++) {
     sample_once();

     static constexpr TickType_t kSamplePeriod = 1 / portTICK_PERIOD_MS;
     vTaskDelay(kSamplePeriod);
   }
}

// Impelement the Hackaday debounce method.
ButtonEvent Buttons::get_event(ButtonId id) {
  static constexpr uint32_t kDebounceMask = 0xFC00003F;
  static constexpr uint32_t kDebounceOn = (0xFFFFFFFF) & kDebounceMask;
  static constexpr uint32_t kDebounceUp = (0xFFFFFFFF << 31) & kDebounceMask;
  static constexpr uint32_t kDebounceDown = (0xFFFFFFFF >> 1) & kDebounceMask;

  uint32_t& history = history_[static_cast<int>(id)];
  if ((history & kDebounceMask) == kDebounceOn) {
    return ButtonEvent::On;
  } else if ((history & kDebounceMask) == kDebounceDown) {
    // Setting to 0xFFFFFFFFUL avoids suprious "Down" signals.
    history = 0xFFFFFFFF;
    return ButtonEvent::Down;
  } else if ((history & kDebounceMask) == kDebounceUp) {
    return ButtonEvent::Up;
  }

  return ButtonEvent::Off;
}

const Buttons::IdPinMap Buttons::pin_id_map_[] = {
  { ButtonId::Red, GPIO_NUM_32 },
  { ButtonId::Orange, GPIO_NUM_33 },
  { ButtonId::Yellow, GPIO_NUM_34 },
  { ButtonId::Green, GPIO_NUM_35 },
  { ButtonId::Blue, GPIO_NUM_36 },
  { ButtonId::Purple, GPIO_NUM_39 },
};

const uint64_t Buttons::kLongPressUs;

// This just tells the system to wake up.
void IRAM_ATTR Buttons::on_interrupt(void *args) {
    Buttons* buttons = static_cast<Buttons*>(args);
    buttons->disable_interrupts();
    buttons->wake();
}

void Buttons::enable_interrupts() const {
  for (int i = 0; i < kNumButtons; i++) {
    gpio_intr_enable(pin_id_map_[i].gpio);
  }
}

void IRAM_ATTR Buttons::disable_interrupts() const {
  for (int i = 0; i < kNumButtons; i++) {
    gpio_intr_disable(pin_id_map_[i].gpio);
  }
}

void IRAM_ATTR Buttons::wake() {
  char dummy = 0;
  xQueueSendFromISR(wake_queue_, &dummy, NULL);
}

void Buttons::push_history_bit(ButtonId b, uint32_t value) {
  int id = static_cast<int>(b);
  history_[id] = (history_[id] << 1) | (value & 0x1);
}

void Buttons::sample_once() {
  for (int i = 0; i < kNumButtons; i++) {
    push_history_bit(pin_id_map_[i].id, gpio_get_level(pin_id_map_[i].gpio));
  }
}

// This reads the button state and sends commands until all buttons are
// released.
void Buttons::process_buttons() {
  char dummy;
  while (1) {
    xQueueReceive(wake_queue_, &dummy, portMAX_DELAY);

    do {
      // Just woke... time to read button state and turn into a command.
      blocking_sample();

      int64_t button_down_times[kNumButtons] = {};

      for (int i = 0; i < kNumButtons; i++) {
        switch (get_event(static_cast<ButtonId>(i))) {
          case ButtonEvent::Up:
            button_down_times[i] = 0;
            if ((button_down_times[0] | button_down_times[1] |
                 button_down_times[2] | button_down_times[3] |
                 button_down_times[4] | button_down_times[5]) == 0) {
              player_->start_playing(i);
            } else {
              // TODO: pinning needs to be a semaphore counter.
              led_->unpin(i);
            }
            break;

          case ButtonEvent::Down:
            // TODO: Make LED Max bright.
            led_->pin_to_max(i);
            button_down_times[i] = esp_timer_get_time();
            break;

          case ButtonEvent::On:
            if ((esp_timer_get_time() - button_down_times[i]) > kLongPressUs) {
              // TODO: Flag a long press.
            }
            break;

          default:
            // Don't care.
            break;
        }
        // If multiple down for > 30 seconds, do special commands.

        // 60-second hold of all 6 buttons = "wifi" mode.
        // 60-second hold of all 3 buttons = "bluetooth pair" mode.
        // 30-second hold of all 3 buttons = "bluetooth on" mode.
      }

    } while (!all_off());
    
    // Play most recent up.

    enable_interrupts();
  }
}


