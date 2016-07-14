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
//   This is in direct-opposition to e.g. "digital-write()" which execute
//   in real-time, performing considerate amounts of math and jumping in
//   order to relate a pin number to its actual pin/port registers.
//   Using these macros should be as fast as using direct 'PORT = '
//   statements. Though a tiny bit of overhead may be existant due to some
//   risk-avoidance (e.g. Read-Modify-Write macros may be surrounded by
//   CLI/SEI where it may be unnecessary, And some processor-architectures
//   have requirements for, e.g. disabling the "analog" circuitry on
//   input-pins before they can be made digital-inputs. Rather'n having two
//   separate macros, these are combined into a single macro, which can
//   result in some redundant register-writes, resulting in a few
//   unnecessary clock-cycles, in *some* cases. Though, these are usually
//   handled upon init, and generally don't slow actual pin-toggling.
// This file consists of code from esot.eric's "bithandling" which is part
// of his "_commonCode." which generally falls under the "mehPL" but is 
// presented here, by him, under the overall grbl licensing-scheme with 
// the exception that this message must be included in these files and/or 
// their derivatives.
#ifndef __PORTHANDLING_H__
#define __PORTHANDLING_H__

// Some of this code is MCU-architecture-specific, so include the
// associated architecture-specific file.

//Essentially: #include "port_handling_avr.h" or "port_handling_pic32.h",
// as appropriate
#include CONCAT_HEADER(port_handling_,__MCU_ARCH__)


//Others:
// #error "You're either using an unsupported MCU architecture, or you're using a compiler that doesn't define __AVR_ARCH__ or __PIC32MX__. In the former case, create your own port_handling_<mcu-architecture>.h file, by copying one of the others, and modifying that (and this) as appropriate. In the latter case, define one of those, as apporpriate"







//This writes a value to a variable, but only to those bits that are Masked
// (1). This handles writing both 1's and 0's to those masked-bits.

//(variable & ~mask) clears the bits to be written
//(value & mask)     assures value doesn't overwrite masked bits
#ifndef __IGNORE_MEH_ERRORS__
#warning "if an interrupt modifies variable, we could screw up the variable!"
#endif



#ifndef __WRITE_MASKED__UNSAFE__
#define writeMasked(value, mask, variable)   \
   (variable) = (((variable) & ~(mask)) | ((value) & (mask)))
#else
#warning "writeMasked is UNSAFE for comparison-purposes!"
//Here your value MUST be already masked.
// This is *not* how I like writeMasked to work
// It is only here for comparison-purposes with grbl-master
#define writeMasked(value, mask, variable)   \
   (variable) = (((variable) & ~(mask)) | ((value)          ))
#endif


//Set a bit in a byte, leaving all other bits unchanged
//Wow... apparently avr-gcc is smart enough to optimize this to a cbi
// if possible!
#define setbit(bitNum, bitwiseByte) \
      (bitwiseByte = ((bitwiseByte) | (1 << (bitNum))))

//Clear a bit in a byte, leaving all other bits unchanged
//Wow... apparently avr-gcc is smart enough to optimize this to a cbi
// if possible!
#define clrbit(bitNum, bitwiseByte) \
      (bitwiseByte = ((bitwiseByte) & (~(1 << (bitNum)))))



// These are newly-named for grbl...
//  (been meaning to make these changes for a while now...)
#ifndef __IGNORE_MEH_ERRORS__
#error "Realistically, I should create bithandling1.00, and rename all the macros to these names, creating macros to them as necessary for backwards-compatibility, rather'n inflicting my old naming-scheme upon grbl users"
#endif
//  (Although, it's kinda nice to have the declarations here, rather'n only
//   in the architecture-specific files... hmm...)
//  Their actual implementations are most-likely architecture-specific
//  so see the appropriate port_handling_<arch>.h


//NOTE: IMPORTANT:
// SOME pins on SOME devices need special handling to be configured as
// inputs/outputs.
// THESE macros will handle the vast-majority of normal pins.
// HOWEVER, some pins will default to a mode which can NOT be overridden
// merely by using these macros
// E.G. the ICSP (in-system-programmer) or JTAG pins on a PIC32 can only be
// used for I/O if the ICSP/JTAG unit is disabled.




//Configure PORTx's pins as outputs, per the Mask 
// (Mask Bits: 1 = output, 0 = no change)
#define configurePORToutputs_masked(PORTx, Mask) \
         setPORToutMasked((PORTx), (Mask))

//Configure one pin on PORTx as an output
// (This may seem unnecesary, with ..._masked, above... but some
// architectures can handle single-bit-writes more efficiently (e.g. AVR))
#define configurePORToutput(PORTx, Bit) \
         setoutPORT((Bit), (PORTx))


//Configure PORTx's pins as inputs, per the Mask
// (Mask Bits: 1 = input, 0 = no change)
#define configurePORTinputs_masked(PORTx, Mask) \
         setPORTinMasked((PORTx), (Mask))

//Disable PORTx's pull-up resistors, per the Mask
// (Mask Bits: 1 = Disable the pull-up, 0 = no change)
#define disablePORTpullups_masked(PORTx, Mask) \
         clrPORTpuMasked((PORTx), (Mask))

//Enable PORTx's pull-up resistors, per the Mask
// (Mask Bits: 1 = Enable the pull-up, 0 = no change)
#define enablePORTpullups_masked(PORTx, Mask) \
         setPORTpuMasked((PORTx), (Mask))


//Write a value to a port, but only to those bits that are Masked (1)
// This handles writing both 1's and 0's to those masked-bits.
//THIS IS NOT SAFE when PORTx may be modified e.g. in an interrupt...
#define writePORToutputs_masked(PORTx, Mask, MaskedVals) \
         writePORToutputsMasked((PORTx), (Mask), (MaskedVals))

//Clear an ouput-pin to 0
#define clearPORTpin(PORTx, Pxn) \
         clrpinPORT((Pxn), (PORTx))

//Set an output-pin to 1
#define setPORTpin(PORTx, Pxn) \
         setpinPORT((Pxn), (PORTx))



//Is the bit high? 
// Returns: 0        = bit is low
//          NON-ZERO = bit is high
// (generally: returns the binary value of the bit... e.g. bit6 would
//  return 0 if the bit is low and 0100 0000 = 0x40 if the bit is high
//  This is much more efficient than returning 0 or 1, as that would
//  require bit-shifts... but if you need that, see getPORTpinVal() )
#define isBitHigh(bitNum, bitwiseByte) \
   (((bitwiseByte) & (1<<(bitNum))))

//Like isBitHigh, this is specific to PORT-pins 
// (which've been configured as inputs)
// NOTE that this may return undefined (architecture-specific) values for
// pins that are *not* already configured as inputs.
//TODO: Plausibly: gpio_getInputZNZ()
//       (ZNZ = Zero v Non-Zero)?
#define isPORTinputHigh(PORTx, Pxn) \
   isPinPORT((Pxn), (PORTx))




#endif //__PORTHANDLING_H__
