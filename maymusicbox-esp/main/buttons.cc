#include "buttons.h"

#include <array>
#include <limits>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "esp_log.h"

#include "audio_player.h"
#include "led.h"
#include "ulp_button_wake.h"
#include "wake.h"

static const char *TAG = "buttons";

static DRAM_ATTR constexpr int kTimerDivider = 16;
static DRAM_ATTR constexpr int kTimerScale = TIMER_BASE_CLK / kTimerDivider;
static DRAM_ATTR constexpr float kTimerInterval = (0.005/32); // 5ms total debounce

ButtonState IRAM_ATTR to_bs(uint32_t bs_bits) {
  ButtonState bs;
  bs.b0 = bs_bits & 0x01;
  bs.b1 = bs_bits & 0x02;
  bs.b2 = bs_bits & 0x04;
  bs.b3 = bs_bits & 0x08;
  bs.b4 = bs_bits & 0x10;
  bs.b5 = bs_bits & 0x20;

  return bs;
}

void IRAM_ATTR Buttons::on_ulp_interrupt(void* param) {
  // Make sure to grab a snapshot to avoid race conditions.
  ButtonState bs = to_bs(ulp_button_state);
  xQueueSendFromISR(static_cast<Buttons*>(param)->sample_queue_, &bs, NULL);
}

Buttons::Buttons(AudioPlayer *player, Led* led) : player_(player), led_(led) {
  ESP_ERROR_CHECK(register_button_wake_isr(&Buttons::on_ulp_interrupt, this));
}

const uint64_t Buttons::kLongPressUs;

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
  ButtonState prev_bs = {};
  ButtonState bs = {};
  uint64_t button_down_times[kNumColors] = {};
  while (1) {
    // Just woke... time to read button state and turn into a command.
    if (xQueueReceive(sample_queue_, &bs, 500 / portTICK_PERIOD_MS) == pdFALSE) {
      // No change in state since we timedout. So just signal a status event.

      bs = prev_bs;
    }
    bs = to_bs(ulp_button_state);
    std::array<ButtonEvent,6> events;
    state_to_events(events, prev_bs, bs);

    if (bs != prev_bs) {
      ESP_LOGI(TAG, " bs:%02x pbs:%02x 0:%02x 1:%02x 2:%02x 3:%02x 4:%02x 5:%02x\n"
          "    hs0:%08x hs1:%08x hs2:%08x hs3:%08x hs4:%08x hs5:%08x",
          *reinterpret_cast<char*>(&bs),
          *reinterpret_cast<char*>(&prev_bs),
          static_cast<uint32_t>(events[0]),
          static_cast<uint32_t>(events[1]),
          static_cast<uint32_t>(events[2]),
          static_cast<uint32_t>(events[3]),
          static_cast<uint32_t>(events[4]),
          static_cast<uint32_t>(events[5]),
          (&ulp_button_history0)[1],
          (&ulp_button_history1)[1],
          (&ulp_button_history2)[1],
          (&ulp_button_history3)[1],
          (&ulp_button_history4)[1],
          (&ulp_button_history5)[1]
          );
    }

    for (int i = 0; i < kNumColors; i++) {
      SongColor color = static_cast<SongColor>(i);
      switch (events[i]) {
        case ButtonEvent::Up:
          ESP_LOGI(TAG, "Button %d up.", i);
          button_down_times[i] = 0;
          led_->dim_to_on(color);
          break;

        case ButtonEvent::Down:
          ESP_LOGI(TAG, "Button %d down.", i);
          led_->start_following();
          led_->flare(color);
          player_->start_playing(color);
          button_down_times[i] = esp_timer_get_time();
          break;

        case ButtonEvent::On:
          if ((esp_timer_get_time() - button_down_times[i]) > kLongPressUs) {
            // TODO: Flag a long press.
            ESP_LOGI(TAG, "Button %d long press.", i);
            ESP_LOGI(TAG, " bs:%02x pbs:%02x 0:%02x 1:%02x 2:%02x 3:%02x 4:%02x 5:%02x\n"
                "    hs0:%08x hs1:%08x hs2:%08x hs3:%08x hs4:%08x hs5:%08x",
                *reinterpret_cast<char*>(&bs),
                *reinterpret_cast<char*>(&prev_bs),
                static_cast<uint32_t>(events[0]),
                static_cast<uint32_t>(events[1]),
                static_cast<uint32_t>(events[2]),
                static_cast<uint32_t>(events[3]),
                static_cast<uint32_t>(events[4]),
                static_cast<uint32_t>(events[5]),
                (&ulp_button_history0)[1],
                (&ulp_button_history1)[1],
                (&ulp_button_history2)[1],
                (&ulp_button_history3)[1],
                (&ulp_button_history4)[1],
                (&ulp_button_history5)[1]
                );
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
  }
}


