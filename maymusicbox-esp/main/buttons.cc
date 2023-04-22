#include "buttons.h"

#include <array>
#include <limits>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "audio_player.h"
#include "led.h"
#include "logging.h"

static DRAM_ATTR constexpr int kTimerDivider = 16;
static DRAM_ATTR constexpr int kTimerScale = TIMER_BASE_CLK / kTimerDivider;
static DRAM_ATTR constexpr float kTimerInterval = (0.05/32); // 5ms total debounce

bool IRAM_ATTR Buttons::on_timer_interrupt(void* param) {
  Buttons* buttons = static_cast<Buttons*>(param);
  buttons->sample_once();
  buttons->intr_num_samples_++;

  ButtonState bs = {};
  bs.b0 = buttons->is_on(SongColor::Red);
  bs.b1 = buttons->is_on(SongColor::Orange);
  bs.b2 = buttons->is_on(SongColor::Yellow);
  bs.b3 = buttons->is_on(SongColor::Green);
  bs.b4 = buttons->is_on(SongColor::Blue);
  bs.b5 = buttons->is_on(SongColor::Purple);

  if (buttons->intr_num_samples_ >= 32 && bs != buttons->intr_old_button_state_) {
    buttons->intr_old_button_state_ = bs;
    xQueueSendFromISR(buttons->sample_queue_, &bs, NULL);
    return true;
  }

  return false;
}

void Buttons::start_sample_timer() {
  timer_config_t config = {
    .alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = kTimerDivider,
  }; // default clock source is APB
  timer_init(TIMER_GROUP_1, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, kTimerInterval * kTimerScale);
  timer_enable_intr(TIMER_GROUP_1, TIMER_0);
  timer_isr_callback_add(TIMER_GROUP_1, TIMER_0, &Buttons::on_timer_interrupt, this, 0);

  // Reset sampling state. Safe to do before timer starts.
  intr_history_[0] = 0xFFFFFFFF;
  intr_history_[1] = 0xFFFFFFFF;
  intr_history_[2] = 0xFFFFFFFF;
  intr_history_[3] = 0xFFFFFFFF;
  intr_history_[4] = 0xFFFFFFFF;
  intr_history_[5] = 0xFFFFFFFF;
  intr_num_samples_ = 0;

  timer_start(TIMER_GROUP_1, TIMER_0);
}

void Buttons::stop_sample_timer() {
  timer_pause(TIMER_GROUP_1, TIMER_0);
}

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
      ESP_INTR_FLAG_IRAM,
      NULL);

  ESP_LOGI(TAG, "Enabling button interrupts");
  enable_interrupts();
}

// Impelement the Hackaday debounce method.
bool Buttons::is_on(SongColor color) {
  static constexpr uint32_t kDebounceMask = 0xFF0000FF;
  static constexpr uint32_t kDebounceUp = (0xFFFFFFFF >> 4) & kDebounceMask;
  static constexpr uint32_t kDebounceDown = (0xFFFFFFFF << 28) & kDebounceMask;

  uint32_t& h = intr_history_[static_cast<int>(color)];
  // Do some denoising here. 
  if ((h & kDebounceMask) == kDebounceDown) {
    // Setting to 0xFFFFFFFFUL avoids suprious "Down" signals.
    h = 0;
    return true;
  } else if ((h & kDebounceMask) == kDebounceUp) {
    h = 0xFFFFFFFF;
    return false;
  }

  return h == 0;
}

const gpio_num_t Buttons::button_gpio_list_[] = {
  GPIO_NUM_33,
  GPIO_NUM_32,
  GPIO_NUM_35,
  GPIO_NUM_34,
  GPIO_NUM_39,
  GPIO_NUM_36,
};

const uint64_t Buttons::kLongPressUs;

// This just tells the system to wake up.
void IRAM_ATTR Buttons::on_interrupt(void *args) {
    Buttons* buttons = static_cast<Buttons*>(args);
    buttons->disable_interrupts();
    buttons->wake();
}

void Buttons::enable_interrupts() const {
  for (int i = 0; i < kNumColors; i++) {
    gpio_intr_enable(button_gpio_list_[i]);
  }
}

void IRAM_ATTR Buttons::disable_interrupts() const {
  for (int i = 0; i < kNumColors; i++) {
    gpio_intr_disable(button_gpio_list_[i]);
  }
}

void IRAM_ATTR Buttons::wake() {
  char dummy = 0;
  xQueueSendFromISR(wake_queue_, &dummy, NULL);
}

void IRAM_ATTR Buttons::push_history_bit(SongColor b, bool value) {
  int id = static_cast<int>(b);
  intr_history_[id] = intr_history_[id] << 1;
  if (value) {
    intr_history_[id] |= 1;
  }
}

void Buttons::sample_once() {
  for (int i = 0; i < kNumColors; i++) {
    bool v = gpio_get_level(button_gpio_list_[i]) != 0;
    push_history_bit(static_cast<SongColor>(i), v);
  }
}

static ButtonEvent button_state_to_event(bool prev, bool cur) {
  if (prev && cur) {
    return ButtonEvent::On;
  } else if (prev && !cur) {
    return ButtonEvent::Up;
  } else if (!prev && cur) {
    return ButtonEvent::Down;
  } else {
    return ButtonEvent::Off;
  }
}

static void state_to_events(std::array<ButtonEvent,6>& events, const ButtonState& prev, const ButtonState& cur) {
  events[0] = button_state_to_event(prev.b0, cur.b0);
  events[1] = button_state_to_event(prev.b1, cur.b1);
  events[2] = button_state_to_event(prev.b2, cur.b2);
  events[3] = button_state_to_event(prev.b3, cur.b3);
  events[4] = button_state_to_event(prev.b4, cur.b4);
  events[5] = button_state_to_event(prev.b5, cur.b5);
}

// This reads the button state and sends commands until all buttons are
// released.
void Buttons::process_buttons() {
  int dummy;
  while (1) {
    xQueueReceive(wake_queue_, &dummy, portMAX_DELAY);

      // Just woke... time to read button state and turn into a command.
    start_sample_timer();
    ButtonState prev_bs = {};
    ButtonState bs = {};
    uint64_t button_down_times[kNumColors];
    do {
      if (xQueueReceive(sample_queue_, &bs, 1000 / portTICK_PERIOD_MS) == pdFALSE) {
        // No change in state since we timedout. So just signal a status event.
        bs = prev_bs;
      }
      std::array<ButtonEvent,6> events;
      state_to_events(events, prev_bs, bs);

      ESP_LOGI(TAG, "0:%02x 1:%02x 2:%02x 3:%02x 4:%02x 5:%02x",
          static_cast<uint32_t>(events[0]),
          static_cast<uint32_t>(events[1]),
          static_cast<uint32_t>(events[2]),
          static_cast<uint32_t>(events[3]),
          static_cast<uint32_t>(events[4]),
          static_cast<uint32_t>(events[5]));

      for (int i = 0; i < kNumColors; i++) {
        SongColor color = static_cast<SongColor>(i);
        switch (events[i]) {
          case ButtonEvent::Up:
            ESP_LOGI(TAG, "Button %d up.", i);
            button_down_times[i] = 0;
            led_->set_to_follow(color);
            break;

          case ButtonEvent::Down:
            ESP_LOGI(TAG, "Button %d down.", i);
            led_->flare(color);
            player_->start_playing(color);
            button_down_times[i] = esp_timer_get_time();
            break;

          case ButtonEvent::On:
            if ((esp_timer_get_time() - button_down_times[i]) > kLongPressUs) {
              // TODO: Flag a long press.
              ESP_LOGI(TAG, "Button %d long press.", i);
            }
            break;

          case ButtonEvent::Off:
          default:
            // Don't care.
            break;
        }
        // If multiple down for > 30 seconds, do special commands.

        // 60-second hold of all 6 buttons = "wifi" mode.
        // 60-second hold of all 3 buttons = "bluetooth pair" mode.
        // 30-second hold of all 3 buttons = "bluetooth on" mode.
      }

      prev_bs = bs;
    } while (!bs.all_off());
    stop_sample_timer();
    enable_interrupts();

    ESP_LOGI(TAG, "All off");
  }
}


