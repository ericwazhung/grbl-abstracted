#include "grbl.h"
/*
//Can't get away with merely using grbl.h due to my own stupidity at not
//being able to figure out how this grbl.h one-file-goes thing works.
// So, we need system.h which includes grbl.h AND includes system_<arch>.h
#include "system.h"
//And, on top of spaghetti, we need to include limits.h BECAUSE of
//limits_<arch>.h
//So, probably it would make more sense to figure out how this
//one-file-thing-goes... But It Aint Happenin Now.
#include "limits.h"
*/

//This is the general Pin-Change-Interrupt-Handler
// This is used for BOTH the Limits-inputs AND the Control-inputs
//BECAUSE: PIC32 has only one vector for all its pin-change interrupts
//(As far as I'm aware, e.g. AVR has different interrupt-vectors for
// different ports, so doesn't need a function like this)
void __attribute__ ((interrupt(IPL6SOFT))) __attribute__ ((vector(34)))
     _GeneralPinChangeInterruptHandler(void)
{
#ifndef __IGNORE_MEH_ERRORS__
#error "This may be riskay..."
#endif

   //Note, the clearing of the flag occurs within the associated
   //interrupt-handler function

   //TODO: What happens if we get here, and the flag for the *other* is
   //triggered but wasn't responsible for *this* interrupt?
   //(Is that where them things whatwerethey... Shadows? Comes into play?)
   if( bit_istrue(IFS1, ( 1 << (CONTROL_FLAG_BIT) ) ) )
   {
      control_PinChangeInterruptHandler();
   }

   //So.... not elsing... just in case, I guess.

   if( bit_istrue(IFS1, ( 1 << (LIMIT_FLAG_BIT) ) ) )
   {
      limits_PinChangeInterruptHandler();
   }
}


