
//Stepper Driver Interrupt
#define TIMER1A_enableOutputCompareInterrupt()   ( IEC0SET = (1<<24) )

#define TIMER1A_disableOutputCompareInterrupt()  ( IEC0CLR = (1<<24) )



//This is a weird one... what is it?! Why not *stop* the timer?
//NOTE: AVRs' changing of prescaler-select bits automatically starts
//running... PIC32, not quite. Thus T4CONSET
#define TIMER1_fullSpeed() \
({ \
   TIMER1_setPrescaler(0x1); \
   T4CONSET = (1<<15); \
   {}; \
})



#define TIMER1A_OutputCompareInterruptHandler() \
   void __attribute__ ((interrupt(IPL4SOFT))) __attribute__ ((vector(20)))\
        _timer45InterruptHandler(void)

#define TIMER1A_clearOutputCompareFlag()     ( IFS0CLR = (1<<24) )


//Because the PIC32 timers run 3x faster than the AVR timers, (at 48, and
//16MHz, respectively), we can't just use the prescaler-values directly...
//Instead, multiply all the values by 3...
// EXCEPT: the PIC32 doesn't have a prescaler-value of 1024, so in that
// case we need to have a different value (than 3)... so, use this variable
// to implement that.
//This is listed here because macros make use of it, which are
// accessed outside stepper_pic32.c
extern unsigned int timer45_additionalPrescaler;


#ifndef __IGNORE_MEH_ERRORS__
#error "How do we make sure that the period doesn't get screwed-up by changing half-words separately?"
#endif
#define TIMER1A_setCompareMatchValue(val)   \
({ \
   (PR4 = ( ( (val) * timer45_additionalPrescaler) & 0xffff ) ); \
   (PR5 = ( ( (val) * timer45_additionalPrescaler) >> 16 ) ); \
   {}; \
})




//Timer1 prescaler chart:
// The PIC32 doesn't use this same bit-scheme so will require a 
//  switch-statement. 
// ALSO: the timer doesn't run at the same speed!
// Finally: Setting the Prescaler, on an AVR, automatically starts the
// timer, but for PIC32, we have to do that explicitly...
// Oh yeah, and we have to compensate for the lack of clk/1024
//bit: 2     1     0       (as assigned in prescalerConfigBits)
//     CS12  CS11  CS10    (AVR bit definition)
//     0     0     0       Counter Stopped
//     0     0     1       clk/1
//     0     1     0       clk/8
//     0     1     1       clk/64
//     1     0     0       clk/256
//     1     0     1       clk/1024
//     1     1     X       external clock (should be unused)
void TIMER1_setPrescaler(unsigned int prescalerConfigBits);

//Initialize the 16bit timer to clear-on-compare-match mode
//Do not use its output-compare pins
//Don't start counting, yet
//Don't initialize interrupts, yet
//DO: Look at the AVR's code for this, as that's what grbl was written to
//work with, initially, and anything *here* is done so-as to match that
//original design-intent.
//NOTE: PIC32 uses Timers4 and 5 to implement this in 32-bit mode
// (necessary because it runs 3 times faster than the AVR's 16-bit timer)
//Every value loaded to the PIC32's timer-registers should, therefore, be
// 3x that of the same value when loaded to an AVR...
// (See Exception when the StepperTimer is loaded with a Prescaler of 1024)
void TIMER1_initClearTimerOnCompare(void);




//Initialize the 8bit timer to normal-counting 
//  (count to 0xFF, overflow to 0)
//Do not use its output-compare pins
//Don't start counting, yet
//DO INITIALIZE overflow-interrupt
//Don't initialize compare-match interrupt, yet
//NOTE: PIC32 uses Timer2 to implement this. The PIC32 timer is 3 times
// faster than the AVR timer, so its "overflow" value is bumped from 0xff
// to PR2 = (3 * 0xff), so good thing this timer is 16bit!
//Every value loaded to the PIC32's timer-registers should, therefore, be
// 3x that of the same value when loaded to an AVR...
void TIMER0_initNormalCountingAndOverflowInterrupt(void);

//PIC32 Note: Timers have a Period Register, which, in a sense is the same
//as setting up an AVR timer for Clear-On-Compare mode
// To run in "Normal Counting" mode, where the timer clears after e.g.
// 0xff, then the period-register must be set to 0xff.
//The StepPulseTimer uses "normal counting" AND a compare-match (without
//*clearing* the timer upon a compare-match). So, set the period-register
//accordingly (e.g. 0xff, but see other notes, elsewhere) for "normal
//counting" AND use the compare-match interrupt, similar to AVR
//The StepperTimer, OTOH, is only used in Clear-On-Compare mode, so we need
//only set the Period-Register to the compare-match value (see notes
//elsewhere)
//I've arbitrarily chosen to use Output Compare unit #2, because its number
//matches that of the timer I've chosen to associate with it.
//(Note that the Output Compare Units only work with Timers 2 and 3)
#define TIMER0A_enableCompareMatchInterrupt()    ( IEC0SET = (1<<12) )

//Used with STEP_PULSE_DELAY
#define TIMER0A_setOutputCompareValue(val)    ( OC2R = ((val) * 3) )

//This is confusing... not like it's counting *down*
#define TIMER0_loadCount(count)     ( TMR2 = ((count) * 3) )

//Start the timer with a Div8 prescaler
//This, too, might be a bit problematic...
// AVR(16MHz) Div8 needs to be 48MHz/16=3*8 = Div24
// But there's no such thing...
// So, much like Timer45, we need to add a fake prescaler
// e.g. run at div8, but multiply all values by 3
// (including PR = 0xff * 3)
//NOTE: AVR uses the prescale-bits to start/stop counting, as well as
//setting the prescale-value, so we need to set *two* values, here...
#define TIMER0_startDiv8()  ( T2CONSET = ((3<<4) | (1<<15)) )

#define TIMER0_stopCounting() ( T2CONCLR = (1<<15) )


#define TIMER0_OverflowInterruptHandler() \
   void __attribute__ ((interrupt(IPL3SOFT))) __attribute__ ((vector(8)))\
         _timer2OverflowInterruptHandler(void)

#define TIMER0_clearOverflowFlag()        ( IFS0CLR = (1<<9) )

#define TIMER0A_OutputCompareInterruptHandler() \
   void __attribute__ ((interrupt(IPL5SOFT))) __attribute__ ((vector(10)))\
         _timer2CompareMatchInterruptHandler(void)

#define TIMER0A_clearOutputCompareFlag()  ( IFS0CLR = (1<<12) )




#if (STEPPER_INTERFACE == STEPPER_INTERFACE__PHASE_AB_PWM)

//Per the pinout in cpu_map_pic32mx170f256b.h...
#define StepperX_setPWM(pwrA, pwrB) \
({ \
   OC1RS = (pwrA); /* X_STEP/OC1/RB4 */ \
   OC2RS = (pwrB); /* X_DIR/OC2/RB5 */ \
   {}; \
})

#define StepperY_setPWM(pwrA, pwrB) \
({ \
   OC4RS = (pwrA); /* Y_STEP/OC4/RB13 */ \
   OC3RS = (pwrB); /* Y_DIR/OC3/RB14 */ \
   {}; \
})

//This won't be called since the mask is only set for X and Y
//(Not enough PWM outputs for the Z-axis, it'll be single-stepped...)
//#define StepperZ_setPWM(pwrA, pwrB)    {}



#endif //(STEPPER_INTERFACE == STEPPER_INTERFACE__PHASE_AB_PWM)

