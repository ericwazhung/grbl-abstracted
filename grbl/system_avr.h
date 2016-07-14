#ifndef __SYSTEM_AVR_H__
#define __SYSTEM_AVR_H__


#define Control_PinChangeInterruptHandler()  ISR(CONTROL_INT_vect)

//Some processors don't automatically clear the flag when enterring the ISR
//AVRs do, so this will compile to Nada.
#define Control_clearPinChangeFlag()    {}


#define Control_EnablePinChangeInterrupts() \
({ \
   /* Enable specific pins of the Pin Change Interrupt */ \
   CONTROL_PCMSK |= CONTROL_MASK;  \
   /* Enable Pin Change Interrupt */ \
   PCICR |= (1 << CONTROL_INT);  \
   {}; \
})


#endif //__SYSTEM_AVR_H__
