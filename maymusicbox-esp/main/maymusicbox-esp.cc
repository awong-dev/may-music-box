#include <stdio.h>

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"

#include <sys/unistd.h>
#include <sys/stat.h>

#include "esp_sleep.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "audio_player.h"
#include "buttons.h"
#include "led.h"
#include "logging.h"
#include "wake.h"

// Look at this as an example of 2 outputs.
// https://github.com/espressif/esp-adf/blob/master/examples/advanced_examples/http_play_and_save_to_file/main/http_play_and_save_to_file.c

// Mounts the SD Card connected to VSPI default pins on /sdcard
sdmmc_card_t* mount_sdcard() {
  ESP_LOGI(TAG, "Configuring SD card GPIO");
  static const gpio_config_t sd_out_pins = {
    .pin_bit_mask = (
        (1ULL << GPIO_NUM_21) |
        (1ULL << GPIO_NUM_22) |
        (1ULL << GPIO_NUM_23)
        ),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&sd_out_pins));

  static const char* kMountPoint = "/sdcard";

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files = 5,
      .allocation_unit_size = 16 * 1024
  };

  ESP_LOGI(TAG, "Initializing SD card");

  // Use settings defined above to initialize SD card and mount FAT filesystem.
  // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
  // Please check its source code and implement error recovery when developing
  // production applications.
  ESP_LOGI(TAG, "Using SPI peripheral");

  // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
  // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
  // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = SPI3_HOST;

  spi_bus_config_t bus_cfg = {};
  bus_cfg.mosi_io_num = GPIO_NUM_22;
  bus_cfg.miso_io_num = GPIO_NUM_19;
  bus_cfg.sclk_io_num = GPIO_NUM_21;
  bus_cfg.quadwp_io_num = -1;
  bus_cfg.quadhd_io_num = -1;
  bus_cfg.max_transfer_sz = 4000;

  ESP_ERROR_CHECK(spi_bus_initialize(static_cast<spi_host_device_t>(host.slot), &bus_cfg, SDSPI_DEFAULT_DMA));

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = GPIO_NUM_23;
  slot_config.host_id = static_cast<spi_host_device_t>(host.slot);

  ESP_LOGI(TAG, "Mounting filesystem");

  sdmmc_card_t* card = NULL;
  ESP_ERROR_CHECK(esp_vfs_fat_sdspi_mount(kMountPoint, &host, &slot_config,
        &mount_config, &card));

  return card;
}

void run_buttons(void* param) {
  Buttons* buttons = static_cast<Buttons*>(param);
  buttons->process_buttons();
}

extern "C" void app_main(void)
{
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  // Unused pin. Set to output.
  // TODO: consider isolating.
  gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);

  Led led;

  init_ulp();
  if (cause != ESP_SLEEP_WAKEUP_ULP) {
    ESP_LOGE(TAG, "FULL Reset: %d", cause);
    start_ulp();
    led.flare_all_and_follow();
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // HACK: Sleep right away after first start of ULP. Otherwise something goes wrong
    // with the button reading.
    ESP_LOGI(TAG, "Initial sleep");
    enter_sleep();
  }

  mount_sdcard();
  Buttons buttons(&led);

  vTaskDelay(500 / portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(&run_buttons, "button_task", 4096, &buttons, 5, NULL, 0);
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}
