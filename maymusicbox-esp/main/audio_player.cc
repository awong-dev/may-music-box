#include "audio_player.h"

#include "audio_hal.h"
#include "esp_decoder.h"
#include "fatfs_stream.h"
#include "i2s_stream.h"
#include "raw_stream.h"

#include "logging.h"

AudioPlayer::AudioPlayer() {

/*
  static audio_hal_codec_config_t hal_cfg = {
    .adc_input = AUDIO_HAL_ADC_INPUT_LINE1,  // Doesn't matter.
    .dac_output = AUDIO_HAL_DAC_OUTPUT_ALL,
    .codec_mode = AUDIO_HAL_CODEC_MODE_DECODE,
    .i2s_iface = {
      .mode = AUDIO_HAL_MODE_MASTER,
      .fmt = AUDIO_HAL_I2S_NORMAL,
      .samples = AUDIO_HAL_48K_SAMPLES,
      .bits = AUDIO_HAL_BIT_LENGTH_16BITS,
    },
  };
//  audio_hal_handle_t hal_;
  hal_ = audio_hal_init(&hal_cfg, NULL);

  audio_hal_ctrl_codec(hal_, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
  */

  // Setup the basic player_.
  esp_audio_cfg_t cfg = DEFAULT_ESP_AUDIO_CONFIG();
  cfg.resample_rate = 48000;
  cfg.prefer_type = ESP_AUDIO_PREFER_MEM;
  cfg.evt_que = event_queue_;
  cfg.vol_handle = this;
  cfg.vol_set = (audio_volume_set)i2s_alc_volume_set;
  cfg.vol_get = (audio_volume_get)i2s_alc_volume_get;
  player_ = esp_audio_create(&cfg);

  // Create fatfs reader and add to esp_audio
  fatfs_stream_cfg_t fs_reader = FATFS_STREAM_CFG_DEFAULT();
  fs_reader.type = AUDIO_STREAM_READER;
  esp_audio_input_stream_add(player_, fatfs_stream_init(&fs_reader));

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
  esp_audio_codec_lib_add(player_, AUDIO_CODEC_TYPE_DECODER,
      esp_decoder_init(&auto_dec_cfg, auto_decode, 10));

  // Create writers and add to esp_audio
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  i2s_stream_cfg_t i2s_writer = I2S_STREAM_CFG_DEFAULT();
#pragma GCC diagnostic pop
  i2s_writer.i2s_config.sample_rate = 48000;
  i2s_writer.i2s_config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX);
  i2s_writer.type = AUDIO_STREAM_WRITER;

  raw_stream_cfg_t raw_reader = RAW_STREAM_CFG_DEFAULT();
  raw_reader.type = AUDIO_STREAM_READER;
  i2s_ =  i2s_stream_init(&i2s_writer);
  raw_stream_ = raw_stream_init(&raw_reader);

  esp_audio_output_stream_add(player_, i2s_);
  esp_audio_output_stream_add(player_, raw_stream_);

  // Set default volume
  esp_audio_vol_set(player_, 35);
  ESP_LOGI(TAG, "esp_audio instance is:%p\n", player_);

  xTaskCreate(esp_audio_state_task_thunk, "player_task", 4096, this, 1, NULL);

  // TODO: take raw_stream_h and use it to drive LED.
}

void AudioPlayer::esp_audio_state_task() {
  esp_audio_state_t esp_state = {};
  while (1) {
    xQueueReceive(event_queue_, &esp_state, portMAX_DELAY);
    ESP_LOGI(TAG, "esp_auido status:%x,err:%x\n", esp_state.status, esp_state.err_msg);
    if ((esp_state.status == AUDIO_STATUS_FINISHED)
        || (esp_state.status == AUDIO_STATUS_ERROR)) {
      int time = 0;
      int duration = 0;
      esp_audio_time_get(player_, &time);
      esp_audio_duration_get(player_, &duration);
      ESP_LOGI(TAG, "[ * ] End at time:%d ms, duration:%d ms", time, duration);
    }
  }
}

esp_err_t AudioPlayer::set_volume(void *obj, int vol) {
  return i2s_alc_volume_set(static_cast<AudioPlayer*>(obj)->i2s_, vol);
}

esp_err_t AudioPlayer::get_volume(void *obj, int* vol) {
  return i2s_alc_volume_get(static_cast<AudioPlayer*>(obj)->i2s_, vol);
}
