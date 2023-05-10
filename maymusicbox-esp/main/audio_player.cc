#include "audio_player.h"

#include <array>

#include "audio_def.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "driver/rtc_io.h"
#include "esp_decoder.h"
#include "esp_check.h"
#include "mp3_decoder.h"
#include "fatfs_stream.h"
#include "i2s_stream.h"
#include "raw_stream.h"

#include "logging.h"
#include "led_downmix.h"
#include "wake.h"

namespace {
constexpr int kDefaultVolume = -10;
}  // namespace

AudioPlayer::AudioPlayer(ringbuf_handle_t follow_ringbuf, int follow_rate) {
  ESP_LOGI(TAG, "Setup audio pipeline");
  // Create Pipeline.
  audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
  pipeline_ = audio_pipeline_init(&pipeline_cfg);
  mem_assert(pipeline_);

  // Setup i2s stream.
  i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
  i2s_cfg.use_alc = true;
  i2s_cfg.i2s_config.sample_rate = 44100;
  i2s_cfg.type = AUDIO_STREAM_WRITER;
  i2s_stream_writer_ = i2s_stream_init(&i2s_cfg);

  // Setup mp3 decoder.
  // TODO: Do autodecoder.
  mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
  mp3_decoder_ = mp3_decoder_init(&mp3_cfg);

  // Setup downmix.
  led_downmix_cfg_t led_downmix_cfg;
  led_downmix_cfg.follow_ringbuf = follow_ringbuf;
  led_downmix_cfg.follow_rate = follow_rate;
  led_downmix_ = led_downmix_init(&led_downmix_cfg);

  // Setup fatfs
  fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
  fatfs_cfg.type = AUDIO_STREAM_READER;
  fatfs_stream_reader_ = fatfs_stream_init(&fatfs_cfg);

  // Configure the pipeline.
  audio_pipeline_register(pipeline_, fatfs_stream_reader_, "file");
  audio_pipeline_register(pipeline_, mp3_decoder_, "decoder");
  audio_pipeline_register(pipeline_, led_downmix_, "filter");
  audio_pipeline_register(pipeline_, i2s_stream_writer_, "i2s");

  // Link pipeline.
  static std::array link_tag = {"file", "decoder", "filter", "i2s"};
  audio_pipeline_link(pipeline_, &link_tag[0], link_tag.size());

  // Enable chip.
  ESP_LOGI(TAG, "Turning on max98375a");

  // Set to 9db.
  static const gpio_config_t gain_pin = {
    .pin_bit_mask = (
        (1ULL << GPIO_NUM_16)
        ),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK(gpio_config(&gain_pin));
  gpio_set_level(GPIO_NUM_16, 1);

  // Power on chip to L+R/2 mode.
  ESP_ERROR_CHECK(rtc_gpio_hold_dis(GPIO_NUM_4));
  ESP_ERROR_CHECK(rtc_gpio_isolate(GPIO_NUM_4));

  // Set initial volume using I2S ALC.
  i2s_alc_volume_set(i2s_stream_writer_, kDefaultVolume);

  xTaskCreate(&AudioPlayer::pipeline_task_thunk, "pipeline_task", 4096, this, 5, NULL);
}

void AudioPlayer::start_playing(SongColor color) {
  wake_incr();
  const char *url = "/sdcard/test.mp3";
        
  esp_err_t ret = ESP_OK;
  ESP_GOTO_ON_ERROR(
      audio_pipeline_stop(pipeline_),
      skip_playback,
      TAG,
      "Failed starting pipeline stop");
  ESP_GOTO_ON_ERROR(
      audio_pipeline_wait_for_stop_with_ticks(pipeline_, 1000 / portTICK_PERIOD_MS),
      skip_playback,
      TAG,
      "Unable to stop pipeline");

  ESP_GOTO_ON_ERROR(
      audio_element_set_uri(fatfs_stream_reader_, url),
      skip_playback,
      TAG,
      "URL set failed");
  ESP_GOTO_ON_ERROR(
      audio_pipeline_reset_ringbuffer(pipeline_),
      skip_playback,
      TAG,
      "Unable to reset ringbuffer");
  ESP_GOTO_ON_ERROR(
      audio_pipeline_reset_elements(pipeline_),
      skip_playback,
      TAG,
      "Unable to reset elements");
  ESP_GOTO_ON_ERROR(
      audio_pipeline_change_state(pipeline_, AEL_STATE_INIT),
      skip_playback,
      TAG,
      "Unable to move to INIT");

  audio_pipeline_run(pipeline_);
  return;

skip_playback:
    ESP_LOGE(TAG, "Skipping play %d", ret);
    wake_dec();
}

esp_err_t AudioPlayer::set_volume(void *obj, int vol) {
  return i2s_alc_volume_set(static_cast<AudioPlayer*>(obj)->i2s_stream_writer_, vol);
}

esp_err_t AudioPlayer::get_volume(void *obj, int* vol) {
  return i2s_alc_volume_get(static_cast<AudioPlayer*>(obj)->i2s_stream_writer_, vol);
}

void AudioPlayer::pipeline_task() {
  ESP_LOGI(TAG, "[7.0] Set up  event listener");
  audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
  audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

  ESP_LOGI(TAG, "[7.1] Listen for all pipeline events");
  audio_pipeline_set_listener(pipeline_, evt);

  audio_pipeline_run(pipeline_);

  while (1) {
    /* Handle event interface messages from pipeline
       to set music info and to advance to the next song
     */
    audio_event_iface_msg_t msg;
    esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
      continue;
    }
    if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT) {
      // Set music info for a new song to be played
      if (msg.source == (void *) mp3_decoder_
          && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
        audio_element_info_t music_info = {0};
        audio_element_getinfo(mp3_decoder_, &music_info);
        ESP_LOGI(TAG, "[ * ] Received music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
            music_info.sample_rates, music_info.bits, music_info.channels);
        led_downmix_setinfo(led_downmix_, music_info.sample_rates, music_info.bits, music_info.channels);
        audio_element_setinfo(i2s_stream_writer_, &music_info);
        ESP_ERROR_CHECK(i2s_stream_set_clk(i2s_stream_writer_, music_info.sample_rates, music_info.bits, music_info.channels));
        continue;
      }
      // Advance to the next song when previous finishes
      if (msg.source == (void *) i2s_stream_writer_
          && msg.cmd == AEL_MSG_CMD_REPORT_STATUS) {
        audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer_);
        if (el_state == AEL_STATE_FINISHED) {
          ESP_LOGI(TAG, "[ * ] Finished.");
          wake_dec();
        }
        continue;
      }
    }
  }
}
