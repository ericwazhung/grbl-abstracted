#ifndef __SYSTEM_PIC32_H__
#define __SYSTEM_PIC32_H__

// [Pin] Change-Notification (interrupt)

   //CNx - PORTx Input Change Interrupt
   //                                  PORTA    PORTB    PORTC
   //
   //    IRQ                           45       46       47
   //
   //    Vector                    34 ---------------------->
   //
   //    Flag: IFS1 bit                13       14       15
   //
   //    Enable: IEC1 bit              13       14       15
   //
   //    Priority: IPC8 bits       20-18 ------------------->
   //    Sub-priority: IPC8 bits   17-16 ------------------->

   //Change Notice Enable (CNENx)             
   // Set corresponding bit to enable interrupt for that pin
   //Change Notice Status (CNSTATx)
   // Set when a change has occurred on the corresponding pin
   // SINCE the last read of the PORTx bit
   // (Since can't read a specific bit, must handle *all* within the same
   //  interrupt, right...?)
   //Change Notice Pull-up Enable (CNPUEx)
   //Change Notice Pull-down Enable (CNPDx)
   //Change Notice Control (CNCONx)
   // bit15: ON (turn on the Change Notice for the corresponding PORT)

//Control Pins are on PORTB

//PIC32 (unlike the AVR) has a single interrupt-vector for multiple
//interrupt-sources...
// The Pin-Change interrupts all jump to the same vector, regardless of
// which port the change is on.
// (I.E. the Limit inputs and the Control inputs)
// Thus, there needs to be a test for the source, and a function-call to
// handle it appropriately.
// Because Control_PinChangeInterruptHandler() is merely a macro creating a
// function-name for the associated code in control.c, we can redirect it
// from an ISR-"function" to a regular-ol' function...
// Then we can call these functions from our interrupt-vector...
#define Control_PinChangeInterruptHandler()  \
   void control_PinChangeInterruptHandler(void)

//HOWEVER: Since it is, now, here, a regular 'ol function, we need its
//declaration available...
//Of course, this is just a macro... the actual function-call will be to
//control_PinChangeInterruptHandler() (where "control" is lower-case)
Control_PinChangeInterruptHandler();

//This is PIC32-specific, and is used in system_pic32.c's general-purpose
//interrupt-vector to detect the source and route to the appropriate
//handler
#define CONTROL_FLAG_BIT   14


//PIC32 doesn't automatically clear the flag when enterring the ISR
#define Control_clearPinChangeFlag()    \
                        ( IFS1CLR = (1<<(CONTROL_FLAG_BIT)) )

//This contains some redundant initialization code with e.g.
//Limits_EnablePinChangeInterrupts()
//Thus, it shouldn't matter in which order they're called.
#define Control_EnablePinChangeInterrupts() \
({ \
   /* First, make sure it's *NOT* enabled */ \
   IEC1CLR = (1<<14); \
   /* Set the priority-level to 6 */ \
   IPC8 |= (6<<18); \
   /* Enable the change-notice on RB0, 1, and 2... */ \
   CNENB = CONTROL_MASK; \
   /* Enable PORTB's Change-Notice peripheral */ \
   CNCONBSET = (1<<15); \
   /* Make sure the interrupt-flag is cleared before starting */ \
   IFS1CLR = (1<<(CONTROL_FLAG_BIT)); \
   /* Enable multi-vector interrupts... */ \
   INTCONSET = (1<<12); /* MVEC */ \
   /* Enable the interrupt */ \
   IEC1SET = (1<<14); \
   {}; \
})


#endif //__SYSTEM_PIC32_H__
