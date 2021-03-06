#define ASM
//#define STANDALONE
#include <vs1053.h>
#include <vsmpg.h>

// Constants.
// For reference, CLOCKF is 36864000
#define ALL_LEDS 0xfc
#define MAX_LED 0xff       // Value of fully on LED.

// CLOCKF / (CLOCK_DIV + 1) / MAX_LED / PWM_FREQ Hz = ~ 6
//#define CLOCK_DIV 189
//#define TIMER_STOP_VAL 6   

// PWM_FREQ = 1000
//#define CLOCK_DIV 28
//#define TIMER_STOP_VAL 5

// PWM_FREQ = 500
//#define CLOCK_DIV 28
//#define TIMER_STOP_VAL 10

// PWM_FREQ = 300
#define CLOCK_DIV 47
#define TIMER_STOP_VAL 10

// PWM_FREQ = 100
//#define CLOCK_DIV 47
//#define TIMER_STOP_VAL 30

// PWM_FREQ = 30
//#define CLOCK_DIV 47
//#define TIMER_STOP_VAL 500

// PWM_FREQ = 1.5
//#define CLOCK_DIV 47
//#define TIMER_STOP_VAL 15000

// PWM_FREQ = 3000
//#define CLOCK_DIV 11
//#define TIMER_STOP_VAL 4   // CLOCKF / (CLOCK_DIV + 1) / MAX_LED / 256 Hz = ~ 2

     // GPIO Setup.
     .sect data_x,gpio_odata
_gpio_odata:
       .uword 0x0000
        
     .sect data_x,gpio_ddr
       .uword ALL_LEDS

     // Timer config.
     .sect data_x,timer_config
       .uword CLOCK_DIV

     .sect data_x,timer_enable
       .uword 0x0001         // Enable timer0 by setting TIMER_EN_T0 which is bit 1.

     .sect data_x,timer_t0_values
       .uword TIMER_STOP_VAL  // t0l
       .uword 0x0000          // t0h
       .uword 0x0000          // t0cntl
       .uword 0x0000          // t0cnth

///////
/////// Application Statics.
///////

      // Parallel memory regions on x/y bus on the interrupt for stripepd
      // access to variables. Ensure memory map loads them to the same address!
     .sect data_x,int_data_x
_pwm_tick:
       .uword MAX_LED

     .sect data_y,int_data_y
_led_brightness:
       .uword 0x0000          


     .sect data_y,data_y
_led_force_on:
       .uword 0x0000          
_plugin_first_run:
       .uword 0x0001


//////////
//////////  Code starts here.
//////////

     // Timer interrupt vector.
	.sect code,tim0_int
       jmpi _int_timer0_pwm,(i6)+1  // This sets up i6 to the new stack frame with a post-modification.
                                    // jmpi has no delay slot and takes 2 cycles. The lack of delay
                                    // slot is why it's used over j.

	// Plugin function starts at fixed address 0x50
	.sect code,ram_prog
_plugin_main:
       // Prologue.
       ldx (i6)+1,NULL                // Advance the stack frame by 1. Not sure why.
       stx mr0,(i6)+1 ; sty i5,(i6)   // Spill MR0 and I5 into the stack.
       stx i4,(i6)+1 ; sty b2,(i6)    // Spill I6 and B2 so all of B can be used.
       stx b0,(i6)+1 ; sty b1,(i6)    // Spill B registers to stack.
       ldc 0x200,mr0                  // Ensure processor is in a sane mode.

       // I0 has pointer to pointer with data.
       // A1 has the mode value.
       // A0 has the n count.  A0 is also teh return value register.
       //
       // Jump based on A1
       // Return Value is also stored in A0.

       // If-else block for mode in A1.
       // AAPL_AUDIO is most frequently accessed so it is first condition.
       add a1,ones,b0
       nop
       jzs __appl_audio

       sub a1,null,b0
       ldc APPL_W0, B0
       jzs __appl_reset

       sub a1,b0,b0
       nop
       jzs __appl_w0
       ldc _led_force_on, i5

       j __plugin_mode_switch_end
       nop

__appl_reset:
       // Load the first run and set the return value to 0 always.
       // If APPL_RESET does not return zero, sometimes the plugin
       // does weird things.
       ldc _plugin_first_run, i5
       ldy (i5),b0

       // Bail if first_run is 0.
       sub null,b0,b0
       ldc INT_ENABLE,i4
       jzs __plugin_mode_switch_end

       // Okay...turn on the timer here.

       // Set up to enable the interrupt.
       // INT_EN_TIM0 is bit 6 of INT_ENABLE.
       ldc 0x40,b0
       ldx (i4),b1
       or b0,b1,b0

       // Write _plugin_first_run is zero. We know a1 = APPL_RESET = 0
       // Also enable the interrupt. Yay parallel moves.
       j __plugin_mode_switch_end
       sty a1,(i5) ; stx b0,(i4)


__appl_audio:
       // Loop for n samples.
       // Hardware loop runs n + 1 times so use delay slot to decrement by 1.
       // Abuse the fact that a1 = APPL_AUDIO = 1
       add a0,ones,b0 ; sty ls,(I6)+1  // If there is one sample, this will be 0 but we should run.
       stx le,(i6) ; sty lc,(i6)
       jlt __appl_audio_mono_end
       nop
       
       // Yay we are looping. Load the data pointer and have fun.
       loop b0, __appl_audio_mono_end
       ldx (i0), i4

__appl_audio_mono_start:
       // Stereo to mono downmix here. Use L/2 + R/2.
       // This could be done as (L+R)/2 but semantically, super-imposing two
       // attenuated signals makes more sense from a signal processing
       // standpoint than averaging two of them. h/t to @acolwell.
       ldx (i4)+1,b0              // b0 = left channel
       asr b0,b0 ; ldx (i4)-1,b1  // b1 = right channel
       asr b1,b1
       add b0,b1,b0
       stx b0,(i4)+1              // Write back to right sample.
__appl_audio_mono_end:
       stx b0,(i4)+1              // Write back to left sample then increment to next sample.

       ldx (i6),le ; ldy (i6)-1,lc
       j __plugin_mode_switch_end
       ldy (i6),ls

__appl_w0:
       // The leds are connected to GPIO started on pin 2 so the ordinal LED number in a0
       // needs 2 more shifts to produce the correct _led_force_on.
       // More simplely, _led_force_on = 1 << (n + 2).
       //
       // We know a1 = APPL_W0 = 2 so abuse that as a constant to avoid generating an 0x1.
       // We shift that by a0 and then perform one more shift to get to the right value.
       ashl a1, a0, b0
       lsl b0,b0
       j __plugin_mode_switch_end
       sty b0,(i5)

__plugin_mode_switch_end:
       // Epilogue.
       ldx (i6)-1,NULL
       ldx (i6)-1,b0 ; ldy (i6),b1
       ldx (i6)-1,i4 ; ldy (i6),b2
       jr
       ldx (i6)-1,mr0 ; ldy (i6),i5


_int_timer0_pwm:
       sty i7,(i6) ; stx mr0,(i6)+1
       stx d2,(i6) ; sty a0,(i6)+1
       stx c2,(i6) ; sty b2,(i6)+1
       stx b0,(i6) ; sty b1,(i6)+1
       stx c0,(i6) ; sty c1,(i6)+1
       add null,p,c
       stx c0,(i6) ; sty c1,(i6)
       LDC 0x600,mr0  // Saturate so we can subtract without going negative.

       // Load _pwm_tick and _led_brightness
       ldc _pwm_tick,i7
       ldx (i7),b0 ; ldy (i7),b1

       // Decrement tick by 1.
       add b0,ones,b0
       nop
       jzc __int_timer0_pwm_duty_cycle
       nop

       // If tick hits zero, grab sample.
__int_timer0_pwm_new_brightness:
       // Snag sample.
       ldc DAC_LEFT,i7
       ldx (i7),a0

       abs a0,a0
       ldc -7,b0
       ashl a0,b0,a0
	  sub a0,b1,b0
	  nop
	  jge __int_timer0_pwm_new_peak
	  nop
	  // No new peak. Just degrade. Dropping by 3 takes
	  // 100 ticks to fully degrade which for 300Hz pwm is about
	  // 3 Hz..close to the VU-meter fall-off..
	  ldc 3,a0
	  sub b1,a0,a0

__int_timer0_pwm_new_peak:
	  mv a0,b1   // Assume new peak found.
       ldc MAX_LED,b0          // Restore _pwm_tick for next cycle. Note, pwm_ticks is
       ldc _led_brightness,i7

       // Check duty cycle.
       //   TODO(awong): you have an off-by-one.
       //   if tick[1,255] < ledBrightness [0,255].
       //    - Duty cycle on. Write ALL_LEDS
       //    - Duty cycle off. Write _led_force_on.
__int_timer0_pwm_duty_cycle:
       sub b0,b1,a0  // if _pwm_tick < _led_brightness
       stx b0,(i7) ; sty b1,(i7) // _pwm_tick and Store _led_brightnes
       jlt __int_timer0_pwm_write_led  // TODO(awong): Check boundary.
       ldc ALL_LEDS,b0  // Duty cycle on.

       ldc _led_force_on, i7  // Duty cycle off.
       ldy (i7), b0

__int_timer0_pwm_write_led:
       ldc GPIO_ODATA, i7
       stx b0,(i7)

__int_timer0_pwm_epilogue:
       ldx (i6),c0 ; ldy (i6)-1,c1
       resp c0,c1
       ldx (i6),c0 ; ldy (i6)-1,c1
       ldx (i6),b0 ; ldy (i6)-1,b1
       ldx (i6),c2 ; ldy (i6)-1,b2
       ldx (i6),d2 ; ldy (i6)-1,a0
       ldc INT_GLOB_ENA,i7
       ldx (i6),mr0
       reti
       stx i7,(i7) ; LDY (i6)-1,i7

	.end
