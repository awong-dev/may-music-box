#ifndef AUDIO_PLAYER_H_
#define AUDIO_PLAYER_H_

#include "freertos/FreeRTOS.h"

#include "audio_element.h"
#include "audio_pipeline.h"

#include "song.h"

class AudioPlayer {
 public:
  AudioPlayer(ringbuf_handle_t follow_ringbuf, int follow_rate);
  void stop_playing();
  void start_playing(SongColor color);

  // Runs on the audio player thread.
  void set_on_play_done(void(*fn)(void* param), void* param) {
    on_play_done_ = fn;
    on_play_done_param_ = param;
  }

 private:
  audio_pipeline_handle_t pipeline_ = nullptr;
  audio_element_handle_t i2s_stream_writer_ = nullptr;
  audio_element_handle_t mp3_decoder_ =  nullptr;
  audio_element_handle_t led_downmix_ = nullptr;
  audio_element_handle_t fatfs_stream_reader_ = nullptr;
  audio_event_iface_handle_t evt_ = nullptr;
  void (*on_play_done_)(void*) = nullptr;
  void* on_play_done_param_ = nullptr;

  static esp_err_t set_volume(void *obj, int vol);
  static esp_err_t get_volume(void *obj, int* vol);

  static void pipeline_task_thunk(void *param) {
    static_cast<AudioPlayer*>(param)->pipeline_task();
  }

  void pipeline_task();
};

#endif  /*  AUDIO_PLAYER_H_ */
