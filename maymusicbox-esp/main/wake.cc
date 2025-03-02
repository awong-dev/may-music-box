#include "wake.h"

#include <time.h>

#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp32/ulp.h"

#include "driver/gpio.h"
#include "driver/rtc_cntl.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "ulp_common.h"

#include "logging.h"
#include "ulp_button_wake.h"

namespace {
std::atomic_int32_t g_wake_sleep_at{};
const int ACTION_WAKE_LEASE_MS = 5 * 1000;

RTC_DATA_ATTR uint32_t g_wake_button_state;

int32_t get_time_ms() {
  return static_cast<int>(esp_timer_get_time() / 1000);
}

void config_gpio(gpio_num_t gpio, bool pullup_enable, uint32_t* rtc_pin_out) {
  assert(rtc_gpio_is_valid_gpio(gpio) && "GPIO used for pulse counting must be an RTC IO");

  rtc_gpio_init(gpio);
  rtc_gpio_set_direction(gpio, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(gpio);
  if (pullup_enable) {
    rtc_gpio_pullup_en(gpio);
  } else {
    rtc_gpio_pullup_dis(gpio);
  }
  *rtc_pin_out = rtc_io_number_get(gpio);
}

void sleep_task(void* param) {
  while (1) {
    // TODO: This is dumb. Try to sleep right when g_wake_count hits 0. No need for task.
    vTaskDelay(500 / portTICK_PERIOD_MS);
    if (get_time_ms() > g_wake_sleep_at) {
      enter_sleep();
    }
  }
}

}  // namespace

extern const uint8_t bin_start[] asm("_binary_ulp_button_wake_bin_start");
extern const uint8_t bin_end[]   asm("_binary_ulp_button_wake_bin_end");

// Snapshot the last_button_state right after wake-up since esp-idf boot
// is slow enough that the button state will be lost.
extern "C" {
void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
    esp_default_wake_deep_sleep();
    g_wake_button_state = ulp_last_button_state;
    static RTC_RODATA_ATTR const char fmt_str[] = "[WAKE] wbs%08x\n\n\n\n\n";
    esp_rom_printf(fmt_str, g_wake_button_state);
}
}

void init_ulp() {
  ESP_LOGE(TAG, "Init ULP");

  // Setup the RTC GPIOs.
  ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

  // Needed to keep pull-up and pull-down on as well as allow ULP to use the
  // SLEEP command.
  ESP_ERROR_CHECK(esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON));

  config_gpio(GPIO_NUM_36, false, &ulp_button_history0);
  config_gpio(GPIO_NUM_39, false, &ulp_button_history1);
  config_gpio(GPIO_NUM_34, false, &ulp_button_history2);
  config_gpio(GPIO_NUM_35, false, &ulp_button_history3);
  config_gpio(GPIO_NUM_32, true, &ulp_button_history4);
  config_gpio(GPIO_NUM_33, true, &ulp_button_history5);
  ESP_LOGE(TAG, "rtc io: %ld %ld %ld %ld %ld %ld",
      ulp_button_history0,
      ulp_button_history1,
      ulp_button_history2,
      ulp_button_history3,
      ulp_button_history4,
      ulp_button_history5);

  // Setup the rtc pin here for the i2s shutdown.
  ESP_ERROR_CHECK(rtc_gpio_init(GPIO_NUM_4));

  // suppress boot messages
//  esp_deep_sleep_disable_rom_logging();

  esp_set_deep_sleep_wake_stub(&esp_wake_deep_sleep);

  wake_incr(); // Ensure we don't immediately fall asleep.
  xTaskCreatePinnedToCore(&sleep_task, "sleep", 4096, NULL, 1, NULL, 0);
}

void start_ulp() {
  ESP_LOGE(TAG, "Starting ULP");
  ESP_ERROR_CHECK(ulp_load_binary(
        0, // load address, set to 0 when using default linker scripts
        bin_start,
        (bin_end - bin_start) / sizeof(uint32_t)));

  // slow-poll every 20ms.
  static constexpr int kSlowPollMs = 20 * 1000;
  ESP_ERROR_CHECK(ulp_set_wakeup_period(0, kSlowPollMs));

  // On wake, do 16 samples per half-slow-poll interval.
  ESP_ERROR_CHECK(ulp_set_wakeup_period(1, kSlowPollMs/16/2));

  ESP_ERROR_CHECK(ulp_run(&ulp_entry - RTC_SLOW_MEM));
}


void enter_sleep() {
  ESP_LOGE(TAG, "[ EEE ] SLEEPY.");

  // Place and hold amp in shutdown.
  ESP_ERROR_CHECK(rtc_gpio_hold_dis(GPIO_NUM_4));
  ESP_ERROR_CHECK(rtc_gpio_set_direction(GPIO_NUM_4, RTC_GPIO_MODE_OUTPUT_ONLY));
  ESP_ERROR_CHECK(rtc_gpio_set_direction_in_sleep(GPIO_NUM_4, RTC_GPIO_MODE_OUTPUT_ONLY));
  ESP_ERROR_CHECK(rtc_gpio_set_level(GPIO_NUM_4, 0));
  ESP_ERROR_CHECK(rtc_gpio_hold_en(GPIO_NUM_4));

  esp_deep_sleep_start();
}

esp_err_t register_button_wake_isr(intr_handler_t fn, void*arg) {
  if (!fn) {
    ESP_LOGE(TAG, "register_button_wake_isr has null fn");
    return ESP_ERR_INVALID_ARG;
  }
  REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
  return rtc_isr_register(fn, arg, RTC_CNTL_SAR_INT_ST_M, RTC_INTR_FLAG_IRAM);
}

void wake_incr() {
  g_wake_sleep_at = get_time_ms() + ACTION_WAKE_LEASE_MS;
}

uint32_t get_wake_button_state() {
  return g_wake_button_state;
}
