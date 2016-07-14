#ifndef __SPINDLE_CONTROL_PIC32_H__
#define __SPINDLE_CONTROL_PIC32_H__


//PIC32:
//Let's use Timer 3 for the spindle PWM
// The OC# and the Timer# do NOT need to match
// Though it can be somewhat confusing to view...
// I'll use:
//   Timer3, with 
//   Output-Compare Unit 5, mapped to
//   pin RB16
// NOTE: Changing these values, alone, is NOT enough to change the actual
//       PWM pin! See Spindle_enablePWM(), etc.
//These are defined in cpu_map_pic32...h
//#define SPINDLE_PWM_PORT   PORTB
//#define SPINDLE_PWM_BIT    6


//AVR:
// This removes OC2A from the port-pin and returns it to normal digital
// functionality, which, I have yet to check, is probably output, low.
//PIC32: That's how it's implemented... (and tested-functional)
//       Apparently disabling the OC unit returns the pin to normal GPIO
//       despite the PPS-mapping remaining unchanged
#define Spindle_disablePWMandOutputZero()     ( OC5CONCLR = (1<<15) )


//PIC32: Because I'm making this compatible with the atmega328p
// implementation of grbl, I will be using 8bit PWM
// This could easily be modified to 16bit if so-desired...
//Original (atmega328p) implementation:
// Configure PWM to:
//  Start at 0x00, with the pin high
//  Count to PWM-val
//  Set pin low
//  Continue to count until 0xff
//  Reset to 0x00, and repeat
// Do this, per atmega328p at 16MHz with clk/8, at a rate of 7.8KHz(?!)
//  (PIC32: I'm using 48MHz, clk/8, -> ~23KHz. clk/16 would be closer.
//   Or, it could be scaled, e.g. like the stepper-timers, but I don't see 
//   any particular reason for any particular PWM-frequency)
void Spindle_enablePWM(void);

 #define Spindle_PWM_t  uint8_t


#define Spindle_setOutputCompareValue(val)   ( OC5RS = (val) )

#endif //__SPINDLE_CONTROL_PIC32_H__
