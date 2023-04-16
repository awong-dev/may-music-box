#include "led.h"

#include "raw_stream.h"

void Led::led_task() {
  char buf[255];
  while (1) {
    int bytes_read = raw_stream_read(raw_stream_, &buf[0], sizeof(buf));
    for (int i = 0; i < bytes_read; ++i) {
      // Do some math on the samples to produce a decaying magnitude.
    }
  }
}


