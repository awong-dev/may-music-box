#include "buttons.h"

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

void IRAM_ATTR Buttons::on_timer_interrupt(void* param) {
  timer_spinlock_take(TIMER_GROUP_1);
  uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_1);

  if (timer_intr & TIMER_INTR_T0) {
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_0);

    Buttons* buttons = static_cast<Buttons*>(param);
    buttons->sample_once();
    buttons->intr_num_samples_++;

    ButtonState bs = {};
    bs.b0 = buttons->is_on(ButtonId::Red);
    bs.b1 = buttons->is_on(ButtonId::Orange);
    bs.b2 = buttons->is_on(ButtonId::Yellow);
    bs.b3 = buttons->is_on(ButtonId::Green);
    bs.b4 = buttons->is_on(ButtonId::Blue);
    bs.b5 = buttons->is_on(ButtonId::Purple);

    if (buttons->intr_num_samples_ >= 32 && bs != buttons->intr_old_button_state_) {
      buttons->intr_old_button_state_ = bs;
      xQueueSendFromISR(buttons->sample_queue_, &bs, NULL);
    }

    // Reenable timer.
    timer_group_enable_alarm_in_isr(TIMER_GROUP_1, TIMER_0);
  }

  timer_spinlock_give(TIMER_GROUP_1);
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
  timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);
  timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, kTimerInterval * kTimerScale);
  timer_enable_intr(TIMER_GROUP_1, TIMER_0);
  timer_isr_register(TIMER_GROUP_1, TIMER_0, &Buttons::on_timer_interrupt,
      this, 0, NULL);

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
bool Buttons::is_on(ButtonId id) {
  static constexpr uint32_t kDebounceMask = 0xFF0000FF;
  static constexpr uint32_t kDebounceOn = (0x00000000) & kDebounceMask;
  static constexpr uint32_t kDebounceUp = (0xFFFFFFFF >> 4) & kDebounceMask;
  static constexpr uint32_t kDebounceDown = (0xFFFFFFFF << 28) & kDebounceMask;

  uint32_t& h = intr_history_[static_cast<int>(id)];
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

void IRAM_ATTR Buttons::push_history_bit(ButtonId b, bool value) {
  int id = static_cast<int>(b);
  intr_history_[id] = intr_history_[id] << 1;
  if (value) {
    intr_history_[id] |= 1;
  }
}

void Buttons::sample_once() {

  for (int i = 0; i < kNumButtons; i++) {
    bool v = gpio_get_level(pin_id_map_[i].gpio) != 0;
    push_history_bit(pin_id_map_[i].id, v);
  }
}

// This reads the button state and sends commands until all buttons are
// released.
void Buttons::process_buttons() {
  int dummy;
  while (1) {
    xQueueReceive(wake_queue_, &dummy, portMAX_DELAY);

      // Just woke... time to read button state and turn into a command.
    start_sample_timer();
    ButtonState bs = {};
    do {
      xQueueReceive(sample_queue_, &bs, portMAX_DELAY);
      ESP_LOGI(TAG, "0:%02x 1:%02x 2:%02x 3:%02x 4:%02x 5:%02x",
          static_cast<uint32_t>(bs.b0),
          static_cast<uint32_t>(bs.b1),
          static_cast<uint32_t>(bs.b2),
          static_cast<uint32_t>(bs.b3),
          static_cast<uint32_t>(bs.b4),
          static_cast<uint32_t>(bs.b5));
      int64_t button_down_times[kNumButtons] = {};

/*
      for (int i = 0; i < kNumButtons; i++) {
        switch (get_event(static_cast<ButtonId>(i))) {
          case ButtonEvent::Up:
            ESP_LOGI(TAG, "Button up.");
            button_down_times[i] = 0;
            if ((button_down_times[0] | button_down_times[1] |
                 button_down_times[2] | button_down_times[3] |
                 button_down_times[4] | button_down_times[5]) == 0) {
//              player_->start_playing(i);
            } else {
              // TODO: pinning needs to be a semaphore counter.
              led_->unpin(i);
            }
            break;

          case ButtonEvent::Down:
            ESP_LOGI(TAG, "Button down.");
            // TODO: Make LED Max bright.
//            led_->pin_to_max(i);
            button_down_times[i] = esp_timer_get_time();
            break;

          case ButtonEvent::On:
            ESP_LOGI(TAG, "Button on.");
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
      */

    } while (!bs.all_off());

    stop_sample_timer();

    ESP_LOGI(TAG, "All off");
    
    // Play most recent up.

    enable_interrupts();
  }
}


