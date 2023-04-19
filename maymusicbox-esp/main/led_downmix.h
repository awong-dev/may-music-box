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
    int samplerate;     /*!< Audio sample rate (in Hz)*/
    int channel;        /*!< Number of audio channels (Mono=1, Dual=2) */
    int out_rb_size;    /*!< Size of output ring buffer */
    int task_stack;     /*!< Task stack size */
    int task_core;      /*!< Task running in core...*/
    int task_prio;      /*!< Task priority*/
    bool stack_in_ext;  /*!< Try to allocate stack in external memory */
} led_downmix_cfg_t;

#define LED_DOWNMIX_TASK_STACK       (4 * 1024)
#define LED_DOWNMIX_TASK_CORE        (0)
#define LED_DOWNMIX_TASK_PRIO        (5)
#define LED_DOWNMIX_RINGBUFFER_SIZE  (8 * 1024)

#define DEFAULT_LED_DOWNMIX_CONFIG() {                   \
        .samplerate     = 48000,                         \
        .channel        = 1,                             \
        .out_rb_size    = LED_DOWNMIX_RINGBUFFER_SIZE,   \
        .task_stack     = LED_DOWNMIX_TASK_STACK,        \
        .task_core      = LED_DOWNMIX_TASK_CORE,         \
        .task_prio      = LED_DOWNMIX_TASK_PRIO,         \
        .stack_in_ext   = true,                          \
    }

/**
 * @brief      Set the audio sample rate and the number of channels to be processed by the equalizer.
 *
 * @param      self       Audio element handle
 * @param      rate       Audio sample rate
 * @param      ch         Audio channel
 *
 * @return
 *             ESP_OK
 *             ESP_FAIL
 */
esp_err_t led_downmix_set_info(audio_element_handle_t self, int rate, int ch);

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

