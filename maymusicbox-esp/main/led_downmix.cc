// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#include <string.h>
#include "esp_log.h"
#include "audio_error.h"
#include "audio_mem.h"
#include "audio_element.h"
#include "led_downmix.h"
#include "audio_type_def.h"
#include "led.h"

static const char *TAG = "LED_DOWNMIX";

static constexpr int kBufSizeBytes = 100;
static_assert(kBufSizeBytes % sizeof(int16_t) == 0);

typedef struct led_downmix {
  int samplerate = 0;
  int bits = 0;
  int channels = 0;
  int16_t *buf = nullptr;
  int at_eof = 0;
  ringbuf_handle_t follow_ringbuf = nullptr;
  int follow_rate = 0;
  uint32_t follow_window_frames = 0;  // num frames to aggregate into one follow_ringbuf entry
  uint32_t follow_window_frames_remainder = 0;  // left-over from samplerate / follow_rate to ensure sync. This is a hack for 44.1 khz.
  uint32_t follow_windows_accumulated = 0;  // Number of windows that have been sent
  uint32_t follow_frames_accumulated = 0;  // Number of frames accumulated.
  uint16_t follow_accumulate = 0;
} led_downmix_t;

static void setup_led_follow_values(led_downmix_t* led_downmix) {
  led_downmix->follow_window_frames = led_downmix->samplerate / led_downmix->follow_rate;
  led_downmix->follow_window_frames_remainder = led_downmix->samplerate % led_downmix->follow_rate;
  led_downmix->follow_windows_accumulated = 0;
  led_downmix->follow_frames_accumulated = 0;
  led_downmix->follow_accumulate = 0;
}

static esp_err_t led_downmix_destroy(audio_element_handle_t self) {
  led_downmix_t *led_downmix = (led_downmix_t *)audio_element_getdata(self);
  audio_free(led_downmix);
  return ESP_OK;
}

static esp_err_t led_downmix_open(audio_element_handle_t self) {
  ESP_LOGI(TAG, "led_downmix_open");
  led_downmix_t *led_downmix = (led_downmix_t *)audio_element_getdata(self);
  led_downmix->buf = static_cast<int16_t*>(audio_calloc(1, kBufSizeBytes));
  if (led_downmix->buf == NULL) {
    ESP_LOGE(TAG, "calloc buffer failed. (line %d)", __LINE__);
    return ESP_ERR_NO_MEM;
  }

  led_downmix->samplerate = 0;
  led_downmix->bits = 0;
  led_downmix->channels = 0;
  led_downmix->at_eof = 0;
//  led_downmix->follow_window_frames = 0;
//  led_downmix->follow_window_frames_remainder = 0;
  led_downmix->follow_windows_accumulated = 0;
  led_downmix->follow_frames_accumulated = 0;
  led_downmix->follow_accumulate = 0;

  return ESP_OK;
}

static esp_err_t led_downmix_close(audio_element_handle_t self)
{
    ESP_LOGI(TAG, "led_downmix_close");
    led_downmix_t *led_downmix = (led_downmix_t *)audio_element_getdata(self);
    if (led_downmix->buf) {
        audio_free(led_downmix->buf);
        led_downmix->buf = NULL;
    }
    return ESP_OK;
}

static audio_element_err_t led_downmix_process(audio_element_handle_t self, char *in_buffer, int in_len)
{
    led_downmix_t *led_downmix = (led_downmix_t *)audio_element_getdata(self);

    audio_element_err_t r_size = AEL_IO_OK;

    r_size = audio_element_input(self, (char *)led_downmix->buf, kBufSizeBytes);
    if (r_size <= 0) {
      return r_size;
    }

    if (r_size != kBufSizeBytes) {
      led_downmix->at_eof = 1;
    }

    // During init, the music_info isn't set. Skip the processing to avoid divide-by-zero issues.
    if (led_downmix->samplerate == 0 || led_downmix->bits == 0 || led_downmix->channels == 0) {
      return audio_element_output(self, (char *)led_downmix->buf, r_size);
    }

    // Attempt to downmix by frames.
    int frame_bytes = led_downmix->channels * (led_downmix->bits / 8);
    assert(r_size % frame_bytes == 0 && "partial frame");
    int num_frames = r_size / frame_bytes;

    // Walk frames and downmix.
    for (int i = 0; i < num_frames; ++i) {
      // Downmix all channels by summing attenuated signals.
      int32_t val = 0;
      for (int ch = 0; ch < led_downmix->channels; ++ch) {
        val += led_downmix->buf[i * led_downmix->channels + ch] / led_downmix->channels;
      }

      // Write downmixed value back into each channel to not change stream format.
      for (int ch = 0; ch < led_downmix->channels; ++ch) {
        led_downmix->buf[i * led_downmix->channels + ch] = static_cast<int16_t>(val);
      }
    }

    audio_element_err_t written_size = audio_element_output(self, (char *)led_downmix->buf, r_size);
    if (written_size < r_size) {
      ESP_LOGE(TAG, "Dropped %d bytes", r_size - written_size);
    }
    int num_frame_written = written_size / frame_bytes;
    for (int i = 0; i < num_frame_written; ++i) {
      // Write values into the follow_ringbuf.
      int frames_to_accumulate = led_downmix->follow_window_frames;
      if (led_downmix->follow_windows_accumulated % 10) {
        frames_to_accumulate += led_downmix->follow_window_frames_remainder;
      }

      // Add attenuated signals.
      int vol = abs(led_downmix->buf[i * led_downmix->channels]);
      led_downmix->follow_accumulate += vol / frames_to_accumulate;
      led_downmix->follow_frames_accumulated++;

      // If a window is complete, write it out and reset.
      if (led_downmix->at_eof || led_downmix->follow_frames_accumulated == frames_to_accumulate) {
        Led::FollowSample s;
        static int s_follow_frame_no = 0;
        s.n = s_follow_frame_no++;
        if (led_downmix->at_eof) {
          s.n = -s.n;
        }
        s.volume = led_downmix->follow_accumulate;
        if (rb_write(led_downmix->follow_ringbuf,
            reinterpret_cast<char*>(&s),
            sizeof(s),
            0) < sizeof(s)) {
          ESP_LOGI(TAG, "! n:%d", s.n);
        }
        led_downmix->follow_windows_accumulated++;
        led_downmix->follow_accumulate = 0;
        led_downmix->follow_frames_accumulated = 0;
      }
    }

    return written_size;
}

esp_err_t led_downmix_setinfo(audio_element_handle_t self, int rate, int bits, int channels) {
  ESP_LOGI(TAG, "led_downmix_setinfo s:%d b:%d c:%d", rate, bits, channels);

  // Only allow 16-bit samples.
  if (bits != 16) {
    return ESP_ERR_INVALID_ARG;
  }

  led_downmix_t *led_downmix = (led_downmix_t *)audio_element_getdata(self);
  led_downmix->samplerate = rate;
  led_downmix->bits = bits;
  led_downmix->channels = channels;
  setup_led_follow_values(led_downmix);
  led_downmix->at_eof = 0;

  return ESP_OK;
}

audio_element_handle_t led_downmix_init(led_downmix_cfg_t *config)
{
    ESP_LOGI(TAG, "led_downmix_init");

    if (config == NULL) {
        ESP_LOGE(TAG, "config is NULL. (line %d)", __LINE__);
        return NULL;
    }
    led_downmix_t *led_downmix = static_cast<led_downmix_t*>(audio_calloc(1, sizeof(led_downmix_t)));
    AUDIO_MEM_CHECK(TAG, led_downmix, return NULL);
    if (led_downmix == NULL) {
        ESP_LOGE(TAG, "audio_calloc failed for led_downmix. (line %d)", __LINE__);
        return NULL;
    }
    audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
    cfg.destroy = led_downmix_destroy;
    cfg.process = led_downmix_process;
    cfg.open = led_downmix_open;
    cfg.close = led_downmix_close;
    cfg.buffer_len = 0;
    cfg.tag = "led_downmix";
    cfg.out_rb_size = config->out_rb_size;

    audio_element_handle_t el = audio_element_init(&cfg);
    AUDIO_MEM_CHECK(TAG, el, {audio_free(led_downmix); return NULL;});

    led_downmix->follow_ringbuf = config->follow_ringbuf;
    led_downmix->follow_rate = config->follow_rate;

    audio_element_setdata(el, led_downmix);
    return el;
}

