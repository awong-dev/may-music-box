// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#include <string.h>
#include "esp_log.h"
#include "audio_error.h"
#include "audio_mem.h"
#include "audio_element.h"
#include "led_downmix.h"
#include "audio_type_def.h"
static const char *TAG = "LED_DOWNMIX";

#define BUF_SIZE (100)
#define NUMBER_BAND (10)
#define USE_XMMS_ORIGINAL_FREQENT (0)
// #define LED_DOWNMIX_MEMORY_ANALYSIS
// #define DEBUG_LED_DOWNMIX_ENC_ISSUE

typedef struct led_downmix {
    int  samplerate;
    int  channel;
    int16_t *buf;
    int  at_eof;
} led_downmix_t;

static esp_err_t is_valid_led_downmix_channel(int channel)
{
    if ((channel != 1)
        && (channel != 2)) {
        ESP_LOGE(TAG, "The number of channels should be only 1 or 2, here is %d. (line %d)", channel, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

esp_err_t led_downmix_set_info(audio_element_handle_t self, int rate, int ch)
{
    led_downmix_t *led_downmix = (led_downmix_t *)audio_element_getdata(self);
    if (led_downmix->samplerate == rate && led_downmix->channel == ch) {
        return ESP_OK;
    }
    if (is_valid_led_downmix_channel(ch) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    } else {
        ESP_LOGI(TAG, "The reset sample rate and channel of audio stream are %d %d.", rate, ch);
        led_downmix->samplerate = rate;
        led_downmix->channel = ch;
    }
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
#ifdef LED_DOWNMIX_MEMORY_ANALYSIS
    AUDIO_MEM_SHOW(TAG);
#endif
    ESP_LOGD(TAG, "led_downmix_open");
    led_downmix_t *led_downmix = (led_downmix_t *)audio_element_getdata(self);
    audio_element_info_t info = {0};
    audio_element_getinfo(self, &info);
    if (info.sample_rates
        && info.channels) {
        led_downmix->samplerate = info.sample_rates;
        led_downmix->channel = info.channels;
    }
    if (is_valid_led_downmix_channel(led_downmix->channel) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
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

#ifdef LED_DOWNMIX_MEMORY_ANALYSIS
    AUDIO_MEM_SHOW(TAG);
#endif
#ifdef DEBUG_LED_DOWNMIX_ENC_ISSUE
    fclose(infile);
#endif

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
      // TODO: Downmix and throw into LED ringbuffer.
      static constexpr int kNumChannels = 2;
      static constexpr int kSampleBytes = 2;
      static constexpr int kFrameBytes = kNumChannels * kSampleBytes;
      assert(r_size % kFrameBytes == 0);
      int num_frames = r_size / kFrameBytes;
      for (int i = 0; i < num_frames; ++i) {
        // L + R /2 downmix.
        int32_t val = led_downmix->buf[i * kNumChannels];
        val += led_downmix->buf[i * kNumChannels + 1];
        val = val / 2;
        // TODO: Write val into led signal ringbuf.
        led_downmix->buf[i * kNumChannels] = static_cast<int16_t>(val);
        led_downmix->buf[i * kNumChannels + 1] = static_cast<int16_t>(val);
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
    audio_element_setdata(el, led_downmix);
    audio_element_info_t info = {0};
    audio_element_setinfo(el, &info);
    ESP_LOGD(TAG, "led_downmix_init");
    return el;
}

