// meh: TODO: This needs to be updated with license-info
//
// This file handles port-stuff, such as reading/writing bits to port-pins
//   or setting such pins as inputs/outputs...
// One of its key features is the use of ONLY the PORTx register-name
//   such that, e.g. setting the direction-register (DDRx, on an AVR)
//   can be accomplished by calling one of these macros with the PORTx
//   register-name.
//   With AVR's optimization-scheme, this generally results in *zero* extra
//   instructions, the math/remapping is handled during compilation-time.
// This file consists of code from esot.eric's "bithandling" which is part
// of his "_commonCode." which generally falls under the "mehPL" but is 
// presented here, by him, under the overall grbl licensing-scheme with 
// the exception that this message must be included in these files and/or 
// their derivatives.

#ifndef __PORTHANDLING_AVR_H__
#define __PORTHANDLING_AVR_H__

#include <avr/io.h>

//BEWARE when implementing for your own architecture!!!!
//  Many of these macros require read-modify-write operation
//  If they're called when interrupts are running, they may result in
//  damaged-data!
//  Note that this message is the result of seeing *numerous* cases where
//  R-M-W operation *could* be problematic in the original code, EXCEPT
//  that the original code was implemented on an AVR. avr-gcc's optimizer
//  is smart-enough to recognize e.g. 'PORTx |= (1<<BITNUM);' as an 'sbi'
//  instruction, which is inherently atomic. 
//  HOWEVER, if this same operation was written and compiled for another 
//  architecture, it would most-likely NOT be atomic.
//  If this R-M-W operation was interrupted, and PORTx was modified during
//  that interrupt, then when the interrupt completed, PORTx's other bits
//  would be rewritten to their original values, negating the effect of the
//  interrupt.
//  This effect/bug, most-likely, could go unnoticed a million times before
//  randomly showing its effects, and then could go unrepeated a million
//  more times... Debugging these things is difficult without knowing the
//  code inside-and-out. 
//  DO NOT CODE PACEMAKERS nor auto-driving cars LIKE THIS!
//  (even if you *know* the optimizer in version x of compiler y will
//   optimize it correctly!)
//
//  I have done my best to verify that wherever such operations occur,
//  they're *inside* an interrupt, (where other interrupts are disabled),
//  or otherwise will not cause trouble. 
//  BUT I have only done *my* best, not knowing the grbl code 
//   inside-and-out.


//GPIO ports on AVRs are 8-bits wide
// in some cases it's handy to e.g. read an entire port to a variable
// then work on that variable... other architectures may have wider
// port-widths (e.g. PIC32 has, it seems, 16-bit ports)
#ifndef __AVR_PORT_WIDTH_16__
 #define gpioPortWidth_t    uint8_t
#else
 //This is NOT RIGHT.
 //The ONLY reason it's here is to use with -Wconversion to determine
 //whether I missed any comparisons between PORT register-values and
 //variables... in my porting of a bunch of uint8_t's to gpioPortWidth_t's
 #ifndef __IGNORE_MEH_ERRORS__ 
  #warning "THIS IS WRONG, ONLY FOR COMPARISON PURPOSES!"
 #endif
 #define gpioPortWidth_t    uint16_t
#endif
//Generally:
// uint8_t oldState;
// CLI_SAFE(oldState);
//  ...
// SEI_RESTORE(oldState);
//
//Can also call this as:
// uint8_t CLI_SAFE(uint8_name);
// ...
// SEI_RESTORE(uint8_name);
// ...
// //And can also be reused (but not inside!):
// CLI_SAFE(uint8_name);
// ...
// SEI_RESTORE(uint8_name);
//
//Newly-discovered:
//NOTE: This is similar in functionality to <util/atomic.h> which has a
//much better description of the potential necessity for this sorta thing
//It also appears to be more fully-featured, but also seems a bit more
//complicated to use... Also, it require C99, which, I suppose, these days
//is kinda stupid to try to avoid...
//
//TODO: was planning to have something like:
//      GET_ATOMIC(variable) (SET_ATOMIC?)
//      or maybe
//      DO_ATOMIC(...)?
//      
//      then could do, e.g. while(GET_ATOMIC(variable) != 0) {...}
//
//      This could probably be implemented with <util/atomic.h> if I
//      thought about it.
//Newest-discovered (thanks grbl!):
// There's no reason to use masking... just back up the entire SREG and
// restore it...
//#define CLI_SAFE(uint8_name) (uint8_name) = (SREG & (1<<7)); cli()
//#define SEI_RESTORE(uint8_name) if(uint8_name) sei()

#define CLI_SAFE(uint8_name)        (uint8_name) = (SREG); cli()
#define SEI_RESTORE(uint8_name)     (SREG) = (uint8_name)




//By accessing, essentially, PORT+<offset> we can refer to ONLY the PORTx
//definition everywhere in our code, regardless of whether we're trying to
//write a pin-value or set a pull-up resistor...
//With -Os, this code will generally reduce to a single register-write 
// instruction;
//  The math of calculating the offsets, etc. will be handled during
//  compile-time.

//I've never run into an AVR that doesn't follow this scheme, though I
//suppose it's plausible they exist.
#define PORTPORTOFFSET  0
#define DDRPORTOFFSET   1
#define PINPORTOFFSET   2

#define DDR_FROM_PORT(PORTx) \
      ((_MMIO_BYTE(&(PORTx) - DDRPORTOFFSET)))


//Read the bits at the port's inputs (and/or outputs, as-configured)
//Note this could seem a bit confusing when considering different
//Microcontroller architectures and their differing I/O registers
// (Especially: AVR, where reading the register called "PORT" reads the
// values written by the AVR to that register, and NOT the values on the
// pins. Whereas, e.g. reading a PIC32's "PORT" register returns the values
// on the pins.)
//THIS MACRO DOES NOT READ THE "PORT REGISTER"
//This macro reads the values at the pins connected to PORTx
// (Regardless of whether those pins are input or output).
// For AVR: This is essentially "{ return PINx; }"
// For PIC32:                   "{ return PORTx; }"
//THE ARGUMENT, however, is PORTx, regardless of the architecture.
// (Doing-so allows for fewer #defines...
//  e.g. #define inputPinBit  3
//       #define inputPinPORT PORTA
//     //#define inputPinPIN  PINA  //This is unnecessary
//  Again, Doing-so also allows for more hardware-abstraction...
//   as most devices will have "Ports" with "pins" attached to its bits
//   but their register-definitions will vary.
//  NOTE: That because these are all constants, most optimizers will reduce
//  this entire operation to a single instruction
//  e.g. "move to a general-purpose register the value at PINx"
#define readioPORT(PORTx)  (_MMIO_BYTE(&(PORTx) - PINPORTOFFSET))


//A rare occurance, but say one wants to read the port-outputs into a
//variable... do something with that variable... then rewrite the port
//outputs. But can't do-so in a single write-masked operation...
//(Frankly, this seems like bad-practice, to me... what if those
//port-outputs are changed in an interrupt during the math...?)
//Anyways, one might wish to *read* the values *written* to a
//port-output-register.
//This will give *all* the bits on that register, regardless of whether
//those bits are outputs or inputs or anything else.
//(E.G. on an AVR, this will read-back the latched output-bits AND
//read-back whether the pull-up resistors are active on the input-bits!)
//So, only work with the bits you intend to!
// This is *ONLY* intended to enable you to work with the OUTPUT bits
// Any other use would be HIGHLY architecture-specific.
#define readoutputsPORT(PORTx)   (PORTx)


//Likewise, this is *equally* if not MORE risky...
// This will write *all* bits in the output-register
// BUT, again, the output-register is usually shared with other purposes
// when the associated bits are *inputs*
// E.G. again, on AVR, writing 1 to *input* bits, via this macro will turn
// on the pull-up resistors on those bits. Whereas, on PIC32, writing 1 to
// input-bits via this macro will have no effect.
// THUS: This macro is *highly* architecture-specific unless used *only* to
// manipulate output-bits. In other words. Unless you know *exactly* what
// you are doing, and all the possible consequences of your doing-so, that
// you might not foresee (e.g. what happens if those input-bits were
// modified via an interrupt before you rewrite their old values via
// this?)
// (e.g.2. If you have inputs on the same port, are you using this in an
// architecture-specific way?)
// Basically, this macro only exists for the purpose of
// backwards-compatibility-testing of the original grbl code.
//
// THIS MACRO SHOULD NOT BE USED except, maybe, if the *entire* port is
// being used for a single-purpose (e.g. an 8bit data-bus)
// Use writePORToutputs_masked() instead!
#define writeAllPORToutputs_unsafe(PORTx, value)    ((PORTx) = (value))

//Set a single output-pin to 1
#define setpinPORT(Pxn, PORTx)   \
   setbit((Pxn), _MMIO_BYTE(&(PORTx) - PORTPORTOFFSET))

//Clear a single output-pin to 0
#define clrpinPORT(Pxn, PORTx)   \
   clrbit((Pxn), _MMIO_BYTE(&(PORTx) - PORTPORTOFFSET))





//This sets several bits/pins on a port as outputs
//This used to use "value" = 0xff, but writeMasked() may be unsafe in this
//version
#define setPORToutMasked(PORTx, Mask)  \
      writeMasked(Mask, Mask, DDR_FROM_PORT(PORTx))


//Again, for PIC32 (as opposed to, say, AVR) using the Masked macros is
//just as efficient (?) as writing indivudual bit-handling macros...
//So, here, I'll just call them as appropriate.
// (In, say, an AVR, it makes more sense to have separate macros for
//  masked-writes vs. individual bit-writes, as it has SBI and CBI)

//This sets a single bit/pin on a port as an output...
#define setoutPORT(Pxn, PORTx)   \
      setbit((Pxn), _MMIO_BYTE(&(PORTx) - DDRPORTOFFSET))


//This writes a value to a port, but only to those bits that are Masked (1)
// This handles writing both 1's and 0's to those masked-bits.

#ifndef __IGNORE_MEH_ERRORS__
#error "writePORToutputsMasked() is new for AVR, and UNTESTED"
#error "This is a long operation requiring read-modify-write! It should probably be SEI/CLI'd unless in an interrupt"
#endif
#define writePORToutputsMasked(PORTx, Mask, Value) \
   writeMasked((Value), (Mask), (PORTx))


//Configure PORTx's pins as inputs, per the Mask
// (Mask Bits: 1 = input, 0 = no change)
#define setPORTinMasked(PORTx, Mask) \
   writeMasked(0x00, Mask, DDR_FROM_PORT(PORTx))


//Enable the pull-ups on the masked bits.
#define setPORTpuMasked(PORTx, Mask) \
      writeMasked(Mask, Mask, PORTx)

//Disable the pull-ups on the masked bits.
#ifndef __IGNORE_MEH_ERRORS__
#error "clrPORTpuMasked() is new for AVR, and UNTESTED"
#endif
#define clrPORTpuMasked(PORTx, Mask) \
      writeMasked(0x00, Mask, PORTx)

//Is the pin on the port high...? 0 = no, non-zero = yes
//PIC32: NOTE that reading the PORTx register on a PIC32 reads the value on
//the pin... Whereas reading the PORTx register on an AVR reads the value
//*written to* that pin (by internal registers), which, if an input, would
//therefore read-back not the value on the pin, but the value of whether
//the internal pull-up resistor is enabled.
// This macro reads the value on the pin (especially if an input)
// It will also read the value written to an output, but should NOT be used
// that way, as its results may be architecture-dependent.
#define isPinPORT(Pxn, PORTx) \
      isBitHigh((Pxn), _MMIO_BYTE(&(PORTx) - PINPORTOFFSET))



#endif //__PORTHANDLING_PIC32_H__

