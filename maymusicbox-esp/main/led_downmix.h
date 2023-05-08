// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#ifndef _LED_DOWNMIX_H_
#define _LED_DOWNMIX_H_

#include "esp_err.h"
#include "audio_element.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief      Equalizer Configuration
 */
typedef struct led_downmix_cfg {
    int out_rb_size = 8 * 1024;    /*!< Size of output ring buffer */
    ringbuf_handle_t follow_ringbuf = nullptr;
    int follow_rate = 0;  /* how frequently the follow_ringbuf is read */
} led_downmix_cfg_t;

/**
 * @brief      Set the audio frame configuration info.
 *
 * @param      self       Audio element handle
 * @param      rate       Audio sample rate
 * @param      bits       Bits per sample
 * @param      channels   Number of channels
 *
 * @return
 *             ESP_OK
 *             ESP_FAIL
 */
esp_err_t led_downmix_setinfo(audio_element_handle_t self, int rate, int bits, int channels);

/**
 * @brief      Create an Audio Element handle that equalizes incoming data.
 *
 * @param      config  The configuration
 *
 * @return     The created audio element handle
 */
audio_element_handle_t led_downmix_init(led_downmix_cfg_t *config);

#ifdef __cplusplus
}
#endif

#endif

