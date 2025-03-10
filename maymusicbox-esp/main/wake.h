#ifndef WAKE_H_
#define WAKE_H_

#include "esp_sleep.h"
#include "esp_intr_alloc.h"

void init_ulp();
void start_ulp();
void enter_sleep();

uint32_t get_wake_button_state();

void IRAM_ATTR wake_incr();

esp_err_t register_button_wake_isr(intr_handler_t fn, void*arg);

#endif  // WAKE_H_
