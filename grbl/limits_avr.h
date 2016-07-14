#ifndef __LIMITS_AVR_H__
#define __LIMITS_AVR_H__

#define Limits_PinChangeInterruptHandler()   ISR(LIMIT_INT_vect)


//Some processors don't automatically clear the flag when enterring the ISR
//AVRs do, so this will compile to Nada.
#define Limits_clearPinChangeFlag()    {}


#define Limits_EnablePinChangeInterrupts() \
({ \
   /* Enable the specific pins for the Pin-Change Interrupt */ \
   LIMIT_PCMSK |= LIMIT_MASK; \
   /* Enable the pin-change interrupt */ \
   PCICR |= (1 << LIMIT_INT); \
   {}; \
})

#define Limits_DisablePinChangeInterrupts()  \
({ \
   /* Disable specific pins of the Pin Change Interrupt */ \
   LIMIT_PCMSK &= ~LIMIT_MASK;   \
   /* Disable Pin Change Interrupt */  \
   PCICR &= ~(1 << LIMIT_INT);   \
   {};   \
})

#endif //__LIMITS_AVR_H__
