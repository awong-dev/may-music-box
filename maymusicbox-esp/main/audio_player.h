#ifndef AUDIO_PLAYER_H_
#define AUDIO_PLAYER_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "audio_element.h"
#include "esp_audio.h"

class AudioPlayer {
 public:
  AudioPlayer();
  void start_playing(int id);
  audio_element_handle_t raw_stream() {
    return raw_stream_;
  }

 private:
  esp_audio_handle_t player_;
  QueueHandle_t event_queue_ = xQueueCreate(3, sizeof(esp_audio_state_t));
  audio_element_handle_t i2s_ = NULL;
  audio_element_handle_t raw_stream_ = NULL;

  static void esp_audio_state_task_thunk(void *param) {
    static_cast<AudioPlayer*>(param)->esp_audio_state_task();
  }

  void esp_audio_state_task();
  static esp_err_t set_volume(void *obj, int vol);
  static esp_err_t get_volume(void *obj, int* vol);
};

#endif  /*  AUDIO_PLAYER_H_ */
