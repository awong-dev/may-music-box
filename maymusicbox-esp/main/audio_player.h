#ifndef AUDIO_PLAYER_H_
#define AUDIO_PLAYER_H_

#include "freertos/FreeRTOS.h"
//#include "freertos/queue.h"

#include "audio_element.h"
#include "audio_pipeline.h"
//#include "esp_audio.h"

class AudioPlayer {
 public:
  AudioPlayer();
  void start_playing(int id);

 private:
  audio_pipeline_handle_t pipeline_ = nullptr;
  audio_element_handle_t i2s_stream_writer_ = nullptr;
  audio_element_handle_t mp3_decoder_ =  nullptr;
  audio_element_handle_t led_downmix_ = nullptr;
  audio_element_handle_t fatfs_stream_reader_ = nullptr;

  static esp_err_t set_volume(void *obj, int vol);
  static esp_err_t get_volume(void *obj, int* vol);
};

#endif  /*  AUDIO_PLAYER_H_ */
