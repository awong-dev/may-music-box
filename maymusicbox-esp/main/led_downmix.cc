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

#define BUF_SIZE (100)

typedef struct led_downmix {
    int  samplerate;
    int  channel;
    int16_t *buf;
    int  at_eof;
    ringbuf_handle_t follow_ringbuf;
    int follow_rate;
    uint32_t follow_window_frames;  // num frames to aggregate into one follow_ringbuf entry
    uint32_t follow_window_frames_remainder;  // left-over from samplerate / follow_rate to ensure sync. This is a hack for 44.1 khz.
    uint32_t follow_windows_accumulated;  // Number of windows that have been sent
    uint32_t follow_frames_accumulated;  // Number of frames accumulated.
    uint16_t follow_accumulate;
} led_downmix_t;

static void setup_led_follow_values(led_downmix_t* led_downmix) {
  led_downmix->follow_window_frames = led_downmix->samplerate / led_downmix->follow_rate;
  led_downmix->follow_window_frames_remainder = led_downmix->samplerate % led_downmix->follow_rate;
  led_downmix->follow_windows_accumulated = 0;
  led_downmix->follow_frames_accumulated = 0;
  led_downmix->follow_accumulate = 0;
}

esp_err_t led_downmix_set_info(audio_element_handle_t self, int rate, int ch)
{
    led_downmix_t *led_downmix = (led_downmix_t *)audio_element_getdata(self);
    if (led_downmix->samplerate == rate && led_downmix->channel == ch) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "The reset sample rate and channel of audio stream are %d %d.", rate, ch);
    led_downmix->samplerate = rate;
    led_downmix->channel = ch;
    setup_led_follow_values(led_downmix);
    return ESP_OK;
}

static esp_err_t led_downmix_destroy(audio_element_handle_t self)
{
    led_downmix_t *led_downmix = (led_downmix_t *)audio_element_getdata(self);
    audio_free(led_downmix);
    return ESP_OK;
}

static esp_err_t led_downmix_open(audio_element_handle_t self)
{
    ESP_LOGD(TAG, "led_downmix_open");
    led_downmix_t *led_downmix = (led_downmix_t *)audio_element_getdata(self);
    audio_element_info_t info = {0};
    audio_element_getinfo(self, &info);
    if (info.sample_rates
        && info.channels) {
        led_downmix->samplerate = info.sample_rates;
        led_downmix->channel = info.channels;
    }
    led_downmix->buf = static_cast<int16_t*>(audio_calloc(1, BUF_SIZE));
    if (led_downmix->buf == NULL) {
        ESP_LOGE(TAG, "calloc buffer failed. (line %d)", __LINE__);
        return ESP_ERR_NO_MEM;
    }
    memset(led_downmix->buf, 0, BUF_SIZE);

    return ESP_OK;
}

static esp_err_t led_downmix_close(audio_element_handle_t self)
{
    ESP_LOGD(TAG, "led_downmix_close");
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
    if (led_downmix->at_eof) {
      return AEL_IO_DONE;
    }

    r_size = audio_element_input(self, (char *)led_downmix->buf, BUF_SIZE);
    if (r_size > 0) {
      if (r_size != BUF_SIZE) {
        led_downmix->at_eof = 1;
      }
      audio_element_info_t music_info = {};
      audio_element_getinfo(self, &music_info);

      // During init, the music_info isn't set. Skip the processing to avoid divide-by-zero issues.
      if (music_info.sample_rates == 0 || music_info.bits == 0 || music_info.channels == 0){
        return r_size;
      }

      assert(music_info.bits % 8 == 0);
      int frame_bytes = music_info.channels * (music_info.bits / 8);
      assert(r_size % frame_bytes == 0);
      int num_frames = r_size / frame_bytes;
      static int s_total_frames = 0;
      s_total_frames += num_frames;
//      ESP_LOGI(TAG, "TF:%d", s_total_frames);
      for (int i = 0; i < num_frames; ++i) {
        // Downmix all channels by summing attenuated signals.
        int32_t val = 0;
        for (int ch = 0; ch < music_info.channels; ++ch) {
          val += led_downmix->buf[i * music_info.channels + ch] / music_info.channels;
        }

        // Write downmixed value back into each channel to not change stream format.
        // TODO: Try just setting i2s_stream_writer_'s writer in audio_player.cc to 1 channel.
        for (int ch = 0; ch < music_info.channels; ++ch) {
          led_downmix->buf[i * music_info.channels + ch] = static_cast<int16_t>(val);
        }

        // Write values into the follow_ringbuf.
        int frames_to_accumulate = led_downmix->follow_window_frames;
        if (led_downmix->follow_windows_accumulated % 10) {
          frames_to_accumulate += led_downmix->follow_window_frames_remainder;
        }

        // Add attenuated signals.
        led_downmix->follow_accumulate += abs(val) / frames_to_accumulate;
        led_downmix->follow_frames_accumulated++;

        // If a window is complete, write it out and reset.
        if (led_downmix->at_eof || led_downmix->follow_frames_accumulated == frames_to_accumulate) {
//          ESP_LOGI(TAG, "F %d", led_downmix->follow_accumulate);
          Led::FollowSample s;
          static int s_follow_frame_no = 0;
          s.n = s_follow_frame_no++;
          if (led_downmix->at_eof) {
          ESP_LOGI(TAG, "Finishing");
            s.n = - s.n;
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
      return audio_element_output(self, (char *)led_downmix->buf, BUF_SIZE);
    }
    return r_size;
}

audio_element_handle_t led_downmix_init(led_downmix_cfg_t *config)
{
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
    cfg.task_stack = config->task_stack;
    cfg.task_prio = config->task_prio;
    cfg.task_core = config->task_core;
    cfg.out_rb_size = config->out_rb_size;
    cfg.stack_in_ext = config->stack_in_ext;

    audio_element_handle_t el = audio_element_init(&cfg);
    AUDIO_MEM_CHECK(TAG, el, {audio_free(led_downmix); return NULL;});

    led_downmix->samplerate = config->samplerate;
    led_downmix->channel = config->channel;
    led_downmix->follow_ringbuf = config->follow_ringbuf;
    led_downmix->follow_rate = config->follow_rate;
    setup_led_follow_values(led_downmix);

    audio_element_setdata(el, led_downmix);
    audio_element_info_t info = {0};
    audio_element_setinfo(el, &info);
    ESP_LOGD(TAG, "led_downmix_init");
    return el;
}

