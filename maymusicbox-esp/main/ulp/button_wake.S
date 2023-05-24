#include "soc/soc_ulp.h"
#include "soc/rtc_io_reg.h"

// rtc gpio 0 3 4 5 8 9
// 0011 0011 1001
#define BUTTON_RTC_GPIO_MASK 0x0339

#define DEBOUNCE_MASK 0xF00F
#define DEBOUNCE_UP ((0xFFFF >> 2) & DEBOUNCE_MASK)
#define DEBOUNCE_DOWN ((0xFFFF << 14) & DEBOUNCE_MASK)

        .global samples_filled
        .bss
samples_filled:
        .long 0

        .global all_off
all_off:
        .long 0

        .global last_button_state
last_button_state:
        .long 0

        .global tmp_button_state
tmp_button_state:
        .long 0

        .global debug_r0
debug_r0:
        .long 0

        .global debug_r1
debug_r1:
        .long 0

        .global debug_r2
debug_r2:
        .long 0
        .data

// Definte a 16-bit button history. High-bytes are empty.
// sw6 - rtc_gpio0
// sw5 - rtc_gpio3
// sw4 - rtc_gpio4
// sw3 - rtc_gpio5
// sw2 - rtc_gpio9
// sw1 - rtc_gpio8
#define BUTTON_HISTORY_PIN_OFFSET 0
#define BUTTON_HISTORY_HISTORY_OFFSET 4
#define BUTTON_HISTORY_ENTRY_WORDS 2
        .global button_history
button_history:
        .global button_history0
button_history0:
        .long 0
        .long 0x0000FFFF
        .global button_history1
button_history1:
        .long 0
        .long 0x0000FFFF
        .global button_history2
button_history2:
        .long 0
        .long 0x0000FFFF
        .global button_history3
button_history3:
        .long 0
        .long 0x0000FFFF
        .global button_history4
button_history4:
        .long 0
        .long 0x0000FFFF
        .global button_history5
button_history5:
        .long 0
        .long 0x0000FFFF

// Program code here.
        .text
        .global entry
entry:
        JUMPS slow_poll, 1, LT
        JUMPS sample_buttons, 2, LT
        // On unknown stage, reset to 0 and halt.
        JUMP start_slow_poll

// Slow polling mode. This is used in deep sleep look for any gpio
// activity. On activity, makes the ULP wake faster and moves to
// sample buttons.
start_slow_poll:
        SLEEP 0
        STAGE_RST
        HALT

slow_poll:
        // Sample all the GPIOs now and start_sampling if any
        // of them are on.
        READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)
        MOVE R3, R0  // Save slow-poll GPIO snapshot to R3
        AND R1, R0, BUTTON_RTC_GPIO_MASK
        SUB R0, R1, BUTTON_RTC_GPIO_MASK  // Ensure none are set.

        JUMPR start_sampling, 0, GT
        HALT

start_sampling:
        // Signal not everything is off.
        MOVE R0, all_off
        MOVE R1, 0
        ST R1, R0, 0

        // Reset samples_filled
        MOVE R0, 0
        MOVE R1, samples_filled
        ST R0, R1, 0

        MOVE R0, tmp_button_state
        MOVE R1, 0
        ST R1, R0, 0

        MOVE R0, last_button_state
        MOVE R1, 0
        ST R1, R0, 0

        MOVE R0, debug_r0
        ST R3, R0, 0

        // Reset history. This shouldn't be necessary, but let's be sure.
        MOVE R0, 0
reset_history:
        MOVE R1, 0xFFFF
        ADD R2, R0, button_history
        ST R1, R2, BUTTON_HISTORY_HISTORY_OFFSET
        ADD R0, R0, BUTTON_HISTORY_ENTRY_WORDS
        JUMPR reset_history, 12, LT

        // Move to stage 2. Load GPIO back to R0 tne start sampling
        SLEEP 1
        STAGE_INC 1
        MOVE R0, R3  // Restore the GPIO snapshot saved from slow_poll.
        JUMP sample_buttons_no_read_gpio

// Sample Buttons mode produces a continuous 16-bit history of all the
// buttons, polling with a period of 5ms/16 for a 5ms debounce. If any
// button looks on after debounce, a wake signal is sent and then this
// data is read asynchronously by the main program. When no button
// activity is detected anywhere, the ULP is moved back to slow_poll mode.
sample_buttons:
        // Update history for all gpios.
        READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)

        // Use this label if GPIO data already in R0 from slow-poll.
sample_buttons_no_read_gpio:

        // Set R1 = current gpio sample.
        // Set R0 = entry offset from button_history
        AND R1, R0, BUTTON_RTC_GPIO_MASK
        MOVE R0, 0

push_history:
        // Set R2 = pin number for entry
        ADD R0, R0, button_history
        LD R2, R0, BUTTON_HISTORY_PIN_OFFSET

        // Set R3 = 1 if pin number is on in R1
        RSH R3, R1, R2
        AND R3, R3, 1
        
        // Load R2 with the history and then OR R3 into it.
        LD R2, R0, BUTTON_HISTORY_HISTORY_OFFSET
        LSH R2, R2, 1
        OR R2, R2, R3
        ST R2, R0, BUTTON_HISTORY_HISTORY_OFFSET

        // Move R0 forward to next entry
        ADD R0, R0, BUTTON_HISTORY_ENTRY_WORDS
        SUB R0, R0, button_history
        JUMPR push_history, 12, LT

        // All histories updated.
        // Increment samples filled counter and calculate button state.
        // Folding button state calculation into previous loop takes
        // too many registers.
        MOVE R0, samples_filled
        LD R1, R0, 0
        ADD R1, R1, 1
        ST R1, R0, 0

        MOVE R0, 0
debounce:
        // set R1 = current history
        // set R3 = entry pointer
        // set R2 = calculated on-off state used by debounce_next
        // R0 is scratch plus conditional.
        ADD R3, R0, button_history
        LD R1, R3, BUTTON_HISTORY_HISTORY_OFFSET

        // Test if button is down.
        AND R0, R1, DEBOUNCE_MASK
        SUB R0, R0, DEBOUNCE_DOWN
        JUMPR debounce_not_down, 0, GT

        // Button is transitioning down. Set history to all down.
        // Set R2 to off state.
        MOVE R2, 0
        ST R2, R3, BUTTON_HISTORY_HISTORY_OFFSET
        JUMP debounce_next

debounce_not_down:
        AND R0, R1, DEBOUNCE_MASK
        SUB R0, R0, DEBOUNCE_UP
        JUMPR debounce_unknown, 0, GT

        // Button is transitioning up. Set history to all up.
        MOVE R2, 0xFFFF
        ST R2, R3, BUTTON_HISTORY_HISTORY_OFFSET

        // Set R2 to on state.
        MOVE R2, 1
        JUMP debounce_next

debounce_unknown:
        // Default to off unless all bits are 0.
        // Note this does NOT clean the history.
        MOVE R2, 0
        MOVE R0, R1
        JUMPR debounce_next, 0, GT
        MOVE R2, 1

debounce_next:
        // Update the button state bitvector base on
        // ON/OFF state in R2.
        MOVE R0, tmp_button_state
        LD R1, R0, 0
        LSH R1, R1, 1
        OR R1, R1, R2
        ST R1, R0, 0

        // Restore and increment R0 by one entry
        ADD R0, R3, BUTTON_HISTORY_ENTRY_WORDS
        SUB R0, R0, button_history
        JUMPR debounce, 12, LT

        // Do not allow moving back to stage 0 if there haven't been enough samples.
        MOVE R0, samples_filled
        LD R0, R0, 0
        JUMPR publish_state, 16, GE
        HALT

publish_state:
        // Reset sample count to 0 so it takes
        // 16 more samples to publish a button state.
        MOVE R0, samples_filled
        MOVE R1, 0
        ST R1, R0, 0

        // If all off in tmp_button_state vetor, move back to slow polling. Otherwise send wake.
        MOVE R0, tmp_button_state
        LD R0, R0, 0
        AND R0, R0, 0x3F

        // Publish the button state
        MOVE R1, last_button_state
        ST R0, R1, 0
        WAKE
        JUMPR start_slow_poll, 1, LT
        HALT

