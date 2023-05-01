#include "soc/soc_ulp.h"
#include "soc/rtc_io_reg.h"

#define BUTTON_RTC_GPIO_MASK 0x01B9

#define DEBOUNCE_MASK 0xF00F
#define DEBOUNCE_UP ((0x1FFF) & DEBOUNCE_MASK)
#define DEBOUNCE_DOWN ((0xFFF8) & DEBOUNCE_MASK)

        .global samples_filled
        .bss
samples_filled:
        .long 0

        .global button_state
button_state:
        .long 0

        .data
// Definte a 16-bit button history. High-bytes are empty.
// sw6 - rtc_gpio0
// sw5 - rtc_gpio3
// sw4 - rtc_gpio4
// sw3 - rtc_gpio5
// sw2 - rtc_gpio9
// sw1 - rtc_gpio8
// Mask:  0x01B9
#define BUTTON_HISTORY_PIN_OFFSET 0
#define BUTTON_HISTORY_HISTORY_OFFSET 4
#define BUTTON_HISTORY_ENTRY_WORDS 2
        .global button_history
button_history:
        .long 0
        .long 0x0000FFFF
        .long 3
        .long 0x0000FFFF
        .long 4
        .long 0x0000FFFF
        .long 5
        .long 0x0000FFFF
        .long 9
        .long 0x0000FFFF
        .long 8
        .long 0x0000FFFF
button_history_end:


// Program code here.
        .text
        .global entry
entry:
        JUMPS slow_poll, 0, EQ
        JUMPS sample_buttons, 1, EQ
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
        AND R0,R0, BUTTON_RTC_GPIO_MASK
        JUMPR start_sampling, 0, GT
        HALT

start_sampling:
        // Reset samples_filled
        MOVE R0, 0
        MOVE R1, samples_filled
        ST R0, R1, 0

        // Reset history. This shouldn't be necessary, but let's be sure.
reset_history:
        MOVE R0, button_history
        MOVE R1, 0xFFFF
        ST R1, R0, BUTTON_HISTORY_HISTORY_OFFSET
        ADD R0, R0, BUTTON_HISTORY_ENTRY_WORDS
        JUMPR reset_history, button_history_end, LT

        // Reset button_state to 0
        MOVE R0, button_state
        MOVE R1, 0
        ST R1, R0, 0

        // Move to stage 2 then fall through.
        SLEEP 1
        STAGE_INC 1

// Sample Buttons mode produces a continuous 16-bit history of all the
// buttons, polling with a period of 5ms/16 for a 5ms debounce. If any
// button looks on after debounce, a wake signal is sent and then this
// data is read asynchronously by the main program. When no button
// activity is detected anywhere, the ULP is moved back to slow_poll mode.
sample_buttons:
        // Update history for all gpios.
        READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)

        // Set R1 = current gpio sample.
        // Set R0 = address of first entry in history array.
        MOVE R1, R0
        MOVE R0, button_history

push_history:
        // Set R2 = pin number for entry
        LD R2, R0, BUTTON_HISTORY_PIN_OFFSET

        // Set R3 = 1 if pin number is on in R1
        MOVE R3, 1
        LSH R3, R3, R2
        AND R3, R1, R3
        RSH R3, R3, R2
        
        // Load R2 with the history and then OR R3 into it.
        LD R2, R0, BUTTON_HISTORY_HISTORY_OFFSET
        LSH R2, R2, 1
        OR R2, R2, R3
        ST R2, R0, BUTTON_HISTORY_HISTORY_OFFSET

        // NOTE: R2 = cur history and R3 is open.

        // Move R0 forward to next entry
        ADD R0, R0, BUTTON_HISTORY_ENTRY_WORDS
        JUMPR push_history, button_history_end, LT

        // All histories updated.
        // Increment samples filled counter and calculate button state.
        // TODO: Fold button state calculation into previous loop.
        MOVE R0, samples_filled
        LD R1, R0, 0
        ADD R1, R1, 1
        ST R1, R0, 0
        MOVE R0, button_history
debounce:
        // Load history then save entry pointer to R3
        LD R1, R0, BUTTON_HISTORY_HISTORY_OFFSET
        AND R1, R1, DEBOUNCE_MASK
        MOVE R3, R0

        // Clear-up history if button is down.
        SUB R0, R1, DEBOUNCE_DOWN
        JUMPR debounce_on, 0, GT
        MOVE R2, 0
        ST R2, R3, BUTTON_HISTORY_HISTORY_OFFSET

        // Set R2 to on/off state.
        MOVE R2, 0
        JUMP debounce_next

debounce_on:
        SUB R0, R1, DEBOUNCE_UP
        JUMPR debounce_next, 0, GT
        MOVE R2, 0xFFFF
        ST R2, R3, BUTTON_HISTORY_HISTORY_OFFSET

        // Set R2 to on/off state.
        MOVE R2, 1
        JUMP debounce_next

debounce_next:
        // Update the button state bitvector base on
        // ON/OFF state in R2.
        MOVE R0, button_state
        LD R1, R0, 0
        LSH R1, R1, 1
        OR R1, R1, R2
        ST R1, R0, 0

        // Restore and increment R0 by one entry
        ADD R0, R3, BUTTON_HISTORY_ENTRY_WORDS
        JUMPR push_history, button_history_end, LT

        // Do not allow moving back to stage 0 if there haven't been enough samples.
        MOVE R0, samples_filled
        LD R0, R0, 0
        JUMPR send_wake, 16, LT

        // If all off in button_state vetor, move back to slow polling. Otherwise send wake.
        MOVE R0, button_state
        LD R0, R0, 0
        AND R0, R0, 0x003F
        JUMPR start_slow_poll, 1, LT

// Check RTC_CNTL_RDY_FOR_WAKEUP if we start trying to avoid spurious
// WAKE interrupts on the main processor. For now, just allow the
// interrupts.
send_wake:
        WAKE
        HALT
