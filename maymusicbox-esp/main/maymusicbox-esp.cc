#include <stdio.h>

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"


#include <sys/unistd.h>
#include <sys/stat.h>

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "audio_hal.h"
#include "audio_element.h"
#include "fatfs_stream.h"
#include "i2s_stream.h"
#include "raw_stream.h"
#include "esp_audio.h"
#include "esp_decoder.h"

#include "buttons.h"

// Look at this as an example of 2 outputs.
// https://github.com/espressif/esp-adf/blob/master/examples/advanced_examples/http_play_and_save_to_file/main/http_play_and_save_to_file.c

static const char *TAG = "maymusicbox";

class AudioPlayer {
  public:
   void start_playing(int id);
};

class Led {
  public:
   void pin_to_max(int id);
   void unpin(int id);
};

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
  static constexpr uint32_t kDebounceMask = 0xFF0000FF;
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

// TODO: make player not a global.
static esp_audio_handle_t player;

static void esp_audio_state_task (void *param)
{
  QueueHandle_t q = (QueueHandle_t) param;
  esp_audio_state_t esp_state = {};
  while (1) {
    xQueueReceive(q, &esp_state, portMAX_DELAY);
    ESP_LOGI(TAG, "esp_auido status:%x,err:%x\n", esp_state.status, esp_state.err_msg);
    if ((esp_state.status == AUDIO_STATUS_FINISHED)
        || (esp_state.status == AUDIO_STATUS_ERROR)) {
      int time = 0;
      int duration = 0;
      esp_audio_time_get(player, &time);
      esp_audio_duration_get(player, &duration);
      ESP_LOGI(TAG, "[ * ] End of time:%d ms, duration:%d ms", time, duration);
    }
  }
}

static void control_task (void *param) {
  QueueHandle_t q = (QueueHandle_t) param;
  int command;
  while (1) {
    xQueueReceive(q, &command, portMAX_DELAY);
    /*
    switch (command) {
      case PLAY_RED:
      case PLAY_BLUE:
      case PLAY_ORANGE:
      case PLAY_YELLOW:
      case PLAY_GREEN:
      case PLAY_PURPLE:

      case STOP:

      case WIFI_ON:

      default:
    }
    */
  }
}

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

  static const gpio_config_t sd_in_pins = {
    .pin_bit_mask = (
        (1ULL << GPIO_NUM_32)
        ),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&sd_in_pins));

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

void configure_leds(void) {
  ESP_LOGI(TAG, "Configuring led GPIO");
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

}

void configure_i2s(void) {
  ESP_LOGI(TAG, "Configuring I2S GPIO");
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

}

void configure_rtc_wake(void) {
  // Set up RTC wake for IO 32,33,34,35,36,39
}

void setup_player(void) {
  if (player ) {
    return ;
  } 
  esp_audio_cfg_t cfg = DEFAULT_ESP_AUDIO_CONFIG();

/*
  cfg.vol_handle = board_handle->audio_hal;
  cfg.vol_set = (audio_volume_set)audio_hal_set_volume;
  cfg.vol_get = (audio_volume_get)audio_hal_get_volume;
  cfg.resample_rate = 48000;
  */
  static audio_hal_codec_config_t hal_cfg = {
    .adc_input = AUDIO_HAL_ADC_INPUT_LINE1,
    .dac_output = AUDIO_HAL_DAC_OUTPUT_ALL,
    .codec_mode = AUDIO_HAL_CODEC_MODE_DECODE,
    .i2s_iface = {
      .mode = AUDIO_HAL_MODE_MASTER,
      .fmt = AUDIO_HAL_I2S_NORMAL,
      .samples = AUDIO_HAL_48K_SAMPLES,
      .bits = AUDIO_HAL_BIT_LENGTH_16BITS,
    },
  };
  audio_hal_handle_t hal = audio_hal_init(&hal_cfg, NULL);
  cfg.resample_rate = 48000;
  cfg.prefer_type = ESP_AUDIO_PREFER_MEM;
  cfg.evt_que = xQueueCreate(3, sizeof(esp_audio_state_t));
  player = esp_audio_create(&cfg);
  audio_hal_ctrl_codec(hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
  xTaskCreate(esp_audio_state_task, "player_task", 4096, cfg.evt_que, 1, NULL);

  // Create readers and add to esp_audio
  fatfs_stream_cfg_t fs_reader = FATFS_STREAM_CFG_DEFAULT();
  fs_reader.type = AUDIO_STREAM_READER;
  esp_audio_input_stream_add(player, fatfs_stream_init(&fs_reader));

  // Add decoders and encoders to esp_audio. Use autodecoder.
  audio_decoder_t auto_decode[] = {
    DEFAULT_ESP_MP3_DECODER_CONFIG(),
    DEFAULT_ESP_AAC_DECODER_CONFIG(),
    DEFAULT_ESP_OGG_DECODER_CONFIG(),
    DEFAULT_ESP_AMRNB_DECODER_CONFIG(),
    DEFAULT_ESP_AMRWB_DECODER_CONFIG(),
    DEFAULT_ESP_FLAC_DECODER_CONFIG(),
    DEFAULT_ESP_OPUS_DECODER_CONFIG(),
    DEFAULT_ESP_WAV_DECODER_CONFIG(),
    DEFAULT_ESP_M4A_DECODER_CONFIG(),
    DEFAULT_ESP_TS_DECODER_CONFIG(),
  };
  esp_decoder_cfg_t auto_dec_cfg = DEFAULT_ESP_DECODER_CONFIG();
  esp_audio_codec_lib_add(player, AUDIO_CODEC_TYPE_DECODER, esp_decoder_init(&auto_dec_cfg, auto_decode, 10));

  // Create writers and add to esp_audio
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  i2s_stream_cfg_t i2s_writer = I2S_STREAM_CFG_DEFAULT();
#pragma GCC diagnostic pop
  i2s_writer.i2s_config.sample_rate = 48000;
  i2s_writer.i2s_config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
  i2s_writer.type = AUDIO_STREAM_WRITER;

  raw_stream_cfg_t raw_writer = RAW_STREAM_CFG_DEFAULT();
  raw_writer.type = AUDIO_STREAM_WRITER;
  audio_element_handle_t i2s_h =  i2s_stream_init(&i2s_writer);
  audio_element_handle_t raw_stream_h = raw_stream_init(&raw_writer);

  esp_audio_output_stream_add(player, i2s_h);
  esp_audio_output_stream_add(player, raw_stream_h);

  // TODO: take raw_stream_h and use it to drive LED.

  // Set default volume
  esp_audio_vol_set(player, 35);
  ESP_LOGI(TAG, "esp_audio instance is:%p\n", player);
}

extern "C" void app_main(void)
{
  // Unused pin. Set to output.
  gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);

  mount_sdcard();

  AudioPlayer player;
  Led led;
  Buttons buttons(&player, &led);

  configure_rtc_wake();

  // Enter sleep/play loop.
}
