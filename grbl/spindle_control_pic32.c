#include "grbl.h"

//Let's use Timer 3 for the spindle PWM
// The OC# and the Timer# do NOT need to match
// Though it can be somewhat confusing to view...
// I'll use:
//   Timer3, with 
//   Output-Compare Unit 5, mapped to pin
//   RB6

void Spindle_enablePWM(void)
{
   //TODO: A lot of this stuff needn't be repeated...
   //      But, so far there's no MCU-specific init-section in
   //      spindle_init()... So... ...

   //Choose a pin for the PWM output... pick a pin, [almost] any pin...
   ppsUNLOCK();
      RPB6R = 0x6;  //0110 = OC5
   ppsLOCK();

   //When the PWM is disabled, it returns to an I/O port, so, configure
   //that ahead of time...
   //ACTUALLY: This is handled in spindle_control.c/spindle_init()
   //setPORTout(SPINDLE_PWM_PORT, SPINDLE_PWM_BIT);

   //PIC32: It seems as though when you change the ON bit on the OC
   //units (to off), it returns port-functionality to the GPIO (LAT) 
   //registers, regardless of the PPS setting, above.
   //As far as Spindle_disablePWMandOutputZerio(), it seems to be fine
   //to set up the output-pin to Low (zero) once upon init, and go
   //from there.
   clearPORTpin(SPINDLE_PWM_PORT, SPINDLE_PWM_BIT);

   //It's wise to set the OCnR value to something sane before starting PWM
   OC5R = 0x7f;


   //From here-on, the compare-register OC5R is read-only...
   // must write to OC5RS, which is buffered until the next timer reset

   //Configure the PWM output:
   OC5CON =    (1<<15)  //ON
            |  (0<<13)  //Continue when the device is idle
            |  (0<<5)   //Use 16-bit (rather than 32-bit)
            |  (1<<3)   //Use Timer3 (0=Timer2)
            // PWM Mode, Fault Pin Disabled:
            |  (1<<2)
            |  (1<<1)
            |  (0<<0);


   //Set the Period-Register. Since the ATMEGA328P we're mimicking is using
   //an 8-bit timer, let's do the same... The timer resets *after* 0xff.
   //(Note that, allegedly, there's support for the ATMEGA2560, which has a
   // 16-bit timer, which could also be done here, by changing the PR and
   // a few other things... But I think the 328P is well-tested, so I'll
   // stick with that, for now).
   //NOTE: stepper_initPWM() Uses this same timer! So must stick with 0xff.
   PR3 = 0xff;

   //Set up the timer...
   //Timer3 is a Type B timer...
   T3CON =     (1<<15) //ON
            |  (0<<13) //Continue while in idle mode...
            |  (0<<7)  //"Gated Time Accumulation is disabled" O...K...?
                       //value ignored due to TCS = 0...
            //Prescaler:
            // The peripheral-bus is currently set to 1x (48MHz)
            // I can't math, right now...
            // So, here's the chart, for later adjustments:
            //bit 6-4 TCKPS<2:0>: Timer Input Clock Prescale Select bits(3)
            //111 = 1:256 prescale value
            //110 = 1:64 prescale value
            //101 = 1:32 prescale value
            //100 = 1:16 prescale value
            //011 = 1:8 prescale value
            //010 = 1:4 prescale value
            //001 = 1:2 prescale value
            //000 = 1:1 prescale value
            //This was giving about 23KHz PWM
            |  (0<<6)
            |  (1<<5)
            |  (1<<4)

            |  (0<<3)   //Use timer in 16-bit mode
            |  (0<<1);  //Use the internal peripheral clock (bit: TCS)


}

