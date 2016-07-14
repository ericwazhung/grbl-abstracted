#include "config.h" //For STEPPER_INTERFACE

//Stepper Driver Interrupt
#define TIMER1A_enableOutputCompareInterrupt()  ( TIMSK1 |= (1<<OCIE1A) )

#define TIMER1A_disableOutputCompareInterrupt()  ( TIMSK1 &= ~(1<<OCIE1A) )

#if 0
#define TIMER1_fullSpeed() \
          (  TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10)  )
#else
#define TIMER1_fullSpeed() \
 ( TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10) )
#endif

#define TIMER1A_OutputCompareInterruptHandler()   ISR(TIMER1_COMPA_vect)

//Nothing to do, here... AVR handles it upon ISR-entry
#define TIMER1A_clearOutputCompareFlag()     {}


#define TIMER1A_setCompareMatchValue(val) ( OCR1A = (val) )

//Timer1 prescaler chart:
// Note: 328P: CS10 is bit 0, so <<CS10 is redundant
//       but maybe for another AVR...?
// TODO: The PIC32 likely doesn't use this same bit-scheme so will likely
// require a switch-statement. Fine... But ALSO: the timer probably doesn't
// run at the same speed!
//bit: 2     1     0
//     CS12  CS11  CS10
//     0     0     0       Counter Stopped
//     0     0     1       clk/1
//     0     1     0       clk/8
//     0     1     1       clk/64
//     1     0     0       clk/256
//     1     0     1       clk/1024
//     1     1     X       external clock (should be unused)
#if 1
#define TIMER1_setPrescaler(prescalerConfigBits) \
      writeMasked( ( (prescalerConfigBits)<<CS10 ), \
                   ( (1<<CS12) | (1<<CS11) | (1<<CS10) ), \
                   TCCR1B )
#else
//Looking into 8-byte size-decrease...
// Choosing this one doesn't affect it.
#define TIMER1_setPrescaler(prescalerConfigBits) \
 ( TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (st.exec_segment->prescaler<<CS10) )
#endif


//So far, this is the longest thing that we need that's Timer-Specific
// by macroizing it, we don't need to add a C file

//Initialize the timer to clear-on-compare-match mode
//Do not use its output-compare pins
//Don't start counting, yet
//Don't initialize interrupts, yet
#define TIMER1_initClearTimerOnCompare() \
({ \
   /* waveform generation = 0100 = CTC */ \
   TCCR1B &= ~(1<<WGM13); \
   TCCR1B |=  (1<<WGM12); \
   TCCR1A &= ~((1<<WGM11) | (1<<WGM10)); \
   /* disconnect the output-compare pins */ \
   TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0)); \
   /* The following are set in st_go_idle() */ \
   /* TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); */ \
   /* TIMSK1 &= ~(1<<OCIE1A); */ \
   {}; \
 })

//Initialize Timer0 to normal-counting (count to 0xFFFF, overflow to 0)
//Do not use its output-compare pins
//Don't start counting, yet
//DO INITIALIZE overflow-interrupt
//Don't initialize compare-match interrupt, yet
#define TIMER0_initNormalCountingAndOverflowInterrupt() \
({ \
   /* Disconnect OC0 outputs and OVF interrupt. */ \
   TIMSK0 &= ~((1<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0)); \
   /* "Normal" operation */ \
   TCCR0A = 0; \
   /* Disable Timer0 until needed */ \
   TCCR0B = 0; \
   /* Enable Timer0 overflow interrupt */ \
   TIMSK0 |= (1<<TOIE0); \
   /* Compare-Match A interrupt will be handled elsewhere */ \
   {}; \
 })


#define TIMER0A_enableCompareMatchInterrupt()   ( TIMSK0 |= (1<<OCIE0A) )


//Used with STEP_PULSE_DELAY
#define TIMER0A_setOutputCompareValue(val)      ( OCR0A = (val) )

//This is confusing... not like it's counting *down*
#define TIMER0_loadCount(count)                 ( TCNT0 = (count) )

//Note that this one doesn't mask the bits...
//Start the timer with a Div8 prescaler
#define TIMER0_startDiv8()       ( TCCR0B = (1<<CS01) )

#define TIMER0_stopCounting()    ( TCCR0B = 0 )

#define TIMER0_OverflowInterruptHandler()    ISR(TIMER0_OVF_vect)

//Nothing to do, here... AVR handles it upon ISR-entry
#define TIMER0_clearOverflowFlag()  {}

#define TIMER0A_OutputCompareInterruptHandler()   ISR(TIMER0_COMPA_vect)

//Nothing to do, here... AVR handles it upon ISR-entry
#define TIMER0A_clearOutputCompareFlag()   {}


#if (STEPPER_INTERFACE == STEPPER_INTERFACE__PHASE_AB_PWM)
 //See stepper_pic32.c/h...
 #error "PHASE_AB_PWM is not (yet?) implemented for AVR... see pic32 for example"
#endif
