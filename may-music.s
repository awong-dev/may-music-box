#define STANDALONE
#include <vs1053.h>

     // Interrupts
	.sect code,tim0_int
	jmpi _int_pwm,(i6)+1  // This sets up i6 to the new stack frame with a post-modification.
	                      // jmpi has no delay slot and takes 2 cycles. The lack of delay slot is why it's used over j.

	// Plugin function starts at fixed address 0x50
	.sect code,ram_prog
	.org 0x50
	.import _LedFlash
	j _LedFlash
	nop

_int_pwm:
	STY i7,(i6) ; STX mr0,(i6)+1
	STX i5,(i6) ; STY lr0,(i6)+1
	STX a2,(i6) ; STY b2,(i6)+1
	STX c2,(i6) ; STY d2,(i6)+1
	STX a0,(i6) ; STY a1,(i6)+1
	ADD null,p,a
	STX a0,(i6) ; STY a1,(i6)

     // Safe body.

	LDC 0x200,mr0 // Must occur after add null,p,a , otherwise
	// unexpected things may happen.
	LDX (i6),a0 ; LDY (i6)-1,a1
	RESP a0,a1
	LDX (i6),a0 ; LDY (i6)-1,a1
	LDX (i6),c2 ; LDY (i6)-1,d2
	LDX (i6),a2 ; LDY (i6)-1,b2
	LDX (i6),i5 ; LDY (i6)-1,lr0
	LDC INT_GLOB_ENA,i7
	LDX (i6),mr0
	RETI
	STX i7,(i7) ; LDY (i6)-1,i7


     .sect data_y,data_y
_led_brightness:
     .uword 0x0000          
_pwm_tick:
     .uword 0x0000          
_pwm_cycle:
     .uword 0x0000          

	.end
