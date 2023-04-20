#ifndef AUDIO_PLAYER_H_
#define AUDIO_PLAYER_H_

#include "freertos/FreeRTOS.h"

#include "audio_element.h"
#include "audio_pipeline.h"

#include "song.h"

class AudioPlayer {
 public:
  AudioPlayer();
  void start_playing(SongColor color);

 private:
  audio_pipeline_handle_t pipeline_ = nullptr;
  audio_element_handle_t i2s_stream_writer_ = nullptr;
  audio_element_handle_t mp3_decoder_ =  nullptr;
  audio_element_handle_t led_downmix_ = nullptr;
  audio_element_handle_t fatfs_stream_reader_ = nullptr;
  audio_event_iface_handle_t evt_ = nullptr;

  static esp_err_t set_volume(void *obj, int vol);
  static esp_err_t get_volume(void *obj, int* vol);

  static void pipeline_task_thunk(void *param) {
    static_cast<AudioPlayer*>(param)->pipeline_task();
  }

  void pipeline_task();
};

#endif  /*  AUDIO_PLAYER_H_ */
