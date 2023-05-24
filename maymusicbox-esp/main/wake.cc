#include "wake.h"

#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp32/ulp.h"

#include "driver/gpio.h"
#include "driver/rtc_cntl.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "ulp_common.h"

#include "logging.h"
#include "ulp_button_wake.h"

namespace {
std::atomic_int g_wake_count{};

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
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // TODO: This is stupid. There should just be one semaphore count.
    if (g_wake_count == 0) {
      enter_sleep();
    }
  }
}

}  // namespace

extern const uint8_t bin_start[] asm("_binary_ulp_button_wake_bin_start");
extern const uint8_t bin_end[]   asm("_binary_ulp_button_wake_bin_end");

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
  ESP_LOGE(TAG, "rtc io: %d %d %d %d %d %d",
      ulp_button_history0,
      ulp_button_history1,
      ulp_button_history2,
      ulp_button_history3,
      ulp_button_history4,
      ulp_button_history5);

  // Setup the rtc pin here for the i2s shutdown.
  ESP_ERROR_CHECK(rtc_gpio_init(GPIO_NUM_4));

  // suppress boot messages
  esp_deep_sleep_disable_rom_logging();

  xTaskCreate(&sleep_task, "sleep", 4096, NULL, 1, NULL);
}

void start_ulp() {
  ESP_LOGE(TAG, "Starting ULP");
  ESP_ERROR_CHECK(ulp_load_binary(
        0, // load address, set to 0 when using default linker scripts
        bin_start,
        (bin_end - bin_start) / sizeof(uint32_t)));

  // slow-poll every 20ms.
  ESP_ERROR_CHECK(ulp_set_wakeup_period(0, 20 * 1000));

  // Buttons do 16 samples per 10ms.
  ESP_ERROR_CHECK(ulp_set_wakeup_period(1, (5 * 1000)/16));

  ESP_ERROR_CHECK(ulp_run(&ulp_entry - RTC_SLOW_MEM));
}


void enter_sleep() {
  // OTHERWISE HOLD LOCK AND SLEEEEEEEP!!!!
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
  return rtc_isr_register(fn, arg, RTC_CNTL_SAR_INT_ST_M);
}

void IRAM_ATTR wake_incr() {
  g_wake_count.fetch_add(1);
}

void wake_dec() {
  g_wake_count.fetch_sub(1);
}
