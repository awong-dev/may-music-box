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

namespace {
const char *TAG = "buttons";

ButtonEvent button_state_to_event(bool prev, bool cur) {
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

void state_to_events(std::array<ButtonEvent,6>& events,
    const ButtonState& prev,
    const ButtonState& cur) {
  events[0] = button_state_to_event(prev.b0, cur.b0);
  events[1] = button_state_to_event(prev.b1, cur.b1);
  events[2] = button_state_to_event(prev.b2, cur.b2);
  events[3] = button_state_to_event(prev.b3, cur.b3);
  events[4] = button_state_to_event(prev.b4, cur.b4);
  events[5] = button_state_to_event(prev.b5, cur.b5);
}

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

}  // namespace

const uint64_t Buttons::kLongPressUs;

Buttons::Buttons(Led* led) : led_(led) {
  // Manually insert first message since interrupt was consumed by waking the
  // entire program. Use the button state captured in the wakeup stub.
  auto wbs = get_wake_button_state();
  push_button_state(get_wake_button_state());

  ESP_LOGI(TAG, "ulp wbs:%08lx lbs:%08lx tbs:%08lx debugr0:%08lx",
  wbs, ulp_last_button_state, ulp_tmp_button_state, ulp_debug_r0);

  // Register the handler for actual WAKE interrupts from the ULP.
  ESP_ERROR_CHECK(register_button_wake_isr(&Buttons::on_ulp_interrupt, this));
}


void IRAM_ATTR Buttons::on_ulp_interrupt(void* param) {
  static_cast<Buttons*>(param)->push_button_state(ulp_last_button_state);
}

void IRAM_ATTR Buttons::push_button_state(uint64_t bs_bits) {
  wake_incr();
  // Make sure to grab a snapshot to avoid race conditions.
  ButtonState bs = to_bs(bs_bits);
  xQueueSendFromISR(button_state_queue_, &bs, NULL);
}

void Buttons::on_play_done() {
  led_->off_and_follow(currently_playing_);
}

// This reads the button state and sends commands until all buttons are
// released.
void Buttons::process_buttons() {
  ButtonState prev_bs = {};
  ButtonState bs = {};
  std::array<uint64_t, kNumColors> button_down_times;
  while (1) {
    // Read button state and turn into a command. May have just woke from deep sleep.
    if (xQueueReceive(button_state_queue_, &bs, 100 / portTICK_PERIOD_MS) == pdFALSE) {
      // Synthetically increment the wake counter to balance out the decrement later.
      wake_incr();
      bs = to_bs(ulp_last_button_state);
    }
    std::array<ButtonEvent,6> events;
    state_to_events(events, prev_bs, bs);

    if (bs != prev_bs) {
      ESP_LOGI(TAG, " bs:%02x pbs:%02x 0:%02lx 1:%02lx 2:%02lx 3:%02lx 4:%02lx 5:%02lx\n"
          "    hs0:%08lx hs1:%08lx hs2:%08lx hs3:%08lx hs4:%08lx hs5:%08lx\n"
          "    all_off:%08lx",
          *reinterpret_cast<char*>(&bs),
          *reinterpret_cast<char*>(&prev_bs),
          static_cast<uint32_t>(events[0]),
          static_cast<uint32_t>(events[1]),
          static_cast<uint32_t>(events[2]),
          static_cast<uint32_t>(events[3]),
          static_cast<uint32_t>(events[4]),
          static_cast<uint32_t>(events[5]),
          ulp_button_history0,
          ulp_button_history1,
          ulp_button_history2,
          ulp_button_history3,
          ulp_button_history4,
          ulp_button_history5,
          ulp_all_off
          );
    }

    SongColor new_song = SongColor::Invalid;
    for (int i = 0; i < kNumColors; i++) {
      SongColor color = static_cast<SongColor>(i);
      switch (events[i]) {
        case ButtonEvent::Up:
          ESP_LOGE(TAG, "Button %d up.", i);
          button_down_times[i] = 0;
          led_->dim_to_glow(color);
          break;

        case ButtonEvent::Down:
          // TODO: We have a race here. If multiple buttons are depressed even spuriously,
          // then we attempt to start_playing multiple times in a row including the
          // wake_incr().  This is almost certainly going to never terminate. Try executing
          // the play at the END of the for loop.
          ESP_LOGE(TAG, "Button %d down.", i);
          new_song = color;
          led_->flare_and_hold(color);
          button_down_times[i] = esp_timer_get_time();
          break;

        case ButtonEvent::On:
          if ((esp_timer_get_time() - button_down_times[i]) > kLongPressUs) {
            // TODO: Flag a long press.
            ESP_LOGI(TAG, "Button %d long press.", i);
            ESP_LOGI(TAG, " bs:%02x pbs:%02x 0:%02lx 1:%02lx 2:%02lx 3:%02lx 4:%02lx 5:%02lx\n"
                "    hs0:%08lx hs1:%08lx hs2:%08lx hs3:%08lx hs4:%08lx hs5:%08lx\n"
                "    all_off:%08lx",
                *reinterpret_cast<char*>(&bs),
                *reinterpret_cast<char*>(&prev_bs),
                static_cast<uint32_t>(events[0]),
                static_cast<uint32_t>(events[1]),
                static_cast<uint32_t>(events[2]),
                static_cast<uint32_t>(events[3]),
                static_cast<uint32_t>(events[4]),
                static_cast<uint32_t>(events[5]),
                ulp_button_history0,
                ulp_button_history1,
                ulp_button_history2,
                ulp_button_history3,
                ulp_button_history4,
                ulp_button_history5,
                ulp_all_off
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

    // Check if there's a new song.
    if (new_song != SongColor::Invalid) {
      // Fix the LEDs if we've stopped playing a song.
      if (currently_playing_ != new_song) {
        led_->off_and_follow(currently_playing_);
      }

      // Stop an exisitng player or create a new one.
      if (player_) {
        player_->stop_playing();
      } else {
        player_ = std::make_unique<AudioPlayer>(led_->follow_ringbuf(), Led::kFollowRateHz);
        player_->set_on_play_done(&Buttons::on_play_done_thunk, this);
      }

      // Set new color and start playing.
      currently_playing_ = new_song;
      player_->start_playing(new_song);
    }

    prev_bs = bs;
  }
}
