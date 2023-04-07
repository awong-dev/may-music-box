#include <stdio.h>

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

static const char *TAG = "maymusicbox";

// Mounts the SD Card connected to VSPI default pins on /sdcard
sdmmc_card_t* mount_sdcard() {
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

  spi_bus_config_t bus_cfg = {
      .mosi_io_num = GPIO_NUM_22,
      .miso_io_num = GPIO_NUM_19,
      .sclk_io_num = GPIO_NUM_21,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 4000,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA));

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = GPIO_NUM_23;
  slot_config.host_id = host.slot;

  ESP_LOGI(TAG, "Mounting filesystem");

  sdmmc_card_t* card = NULL;
  ESP_ERROR_CHECK(esp_vfs_fat_sdspi_mount(kMountPoint, &host, &slot_config,
        &mount_config, &card));

  return card;
}

void configure_gpio(void) {
  static const gpio_config_t amp_pins = {
    .pin_bit_mask = (
        (1ULL << GPIO_NUM_4) |
        (1ULL << GPIO_NUM_5) |
        (1ULL << GPIO_NUM_16) |
        (1ULL << GPIO_NUM_17) |
        (1ULL << GPIO_NUM_18)
        ),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&amp_pins));

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

  static const gpio_config_t sd_in_pins = {
    .pin_bit_mask = (
        (1ULL << GPIO_NUM_32)
        ),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&sd_out_pins));

  static const gpio_config_t led_pins = {
    .pin_bit_mask = (
        (1ULL << GPIO_NUM_13) |
        (1ULL << GPIO_NUM_14) |
        (1ULL << GPIO_NUM_15) |
        (1ULL << GPIO_NUM_25) |
        (1ULL << GPIO_NUM_26) |
        (1ULL << GPIO_NUM_27)
        ),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&led_pins));

  static const gpio_config_t switch_pullup_pins = {
    .pin_bit_mask = (
        (1ULL << GPIO_NUM_32) |
        (1ULL << GPIO_NUM_33)
        ),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_LOW_LEVEL
  };
  ESP_ERROR_CHECK(gpio_config(&switch_pullup_pins));

  static const gpio_config_t switch_pins = {
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
  ESP_ERROR_CHECK(gpio_config(&switch_pins));

  // Unused pin. Set to output.
  gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
}

void configure_i2s() {
}

void configure_buttons() {
}

void configure_rtc_wake() {
  // Set up RTC wake for IO 32,33,34,35,36,39
}

void app_main(void)
{
  configure_gpio();
  configure_i2s();
  configure_buttons();
  mount_sdcard();

  configure_rtc_wake();

  // Enter sleep/play loop.
}
