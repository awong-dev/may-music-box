#ifndef WAKE_H_
#define WAKE_H_

#include "esp_sleep.h"
#include "esp_intr_alloc.h"

void init_ulp();
void start_ulp();
void enter_sleep();

void IRAM_ATTR wake_incr();
void wake_dec();
void wake_set_playing(bool is_playing);

esp_err_t register_button_wake_isr(intr_handler_t fn, void*arg);

#endif  // WAKE_H_
