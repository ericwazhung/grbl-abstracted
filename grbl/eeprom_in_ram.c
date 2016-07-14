// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               eeprom.c
* \li Compiler:           IAR EWAAVR 3.10c
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All devices with split EEPROM erase/write
*                         capabilities can be used.
*                         The example is written for ATmega48.
*
* \li AppNote:            AVR103 - Using the EEPROM Programming Modes.
*
* \li Description:        Example on how to use the split EEPROM erase/write
*                         capabilities in e.g. ATmega48. All EEPROM
*                         programming modes are tested, i.e. Erase+Write,
*                         Erase-only and Write-only.
*
*                         $Revision: 1.6 $
*                         $Date: Friday, February 11, 2005 07:16:44 UTC $
****************************************************************************/

//NOTE: This file does NOT get compiled on its own...
// It is to be #included, as necessary, by eeprom_<architecture>.c
// Yes, #inclusion of C files is generally a "no-no"
// See eeprom_pic32.c for an explanation.


//This file (eeprom_in_ram.c) mimics the functionality of grbl's original eeprom.c
// except, rather than reading/writing from/to EEPROM, it does-so with an
// array in RAM.
// The idea, mainly, is to allow for easy-ish implementation of GRBL on
// architectures that don't already have an eeprom built-in.
// (NOTE: It's generally not difficult to add an external EEPROM (e.g. I2C)
//        OR, to implement similar functionality by reading/writing to
//        the processor's internal FLASH memory. But NEITHER of these
//        methods are implemented, here.)
//THUS:
//   * Upon Powerup, default settings (found in config.h) will be loaded,
//     just as though the eeprom was erased. (per grbl's functionality).
//   * These settings will *only* be stored as long as the power remains
//   ** This uses 1KB additional RAM to implement!
//      (Highly inefficient, but easy to do)
//      Per the original atmega328P grbl was originally implemented on:
//      EEPROM is 1kB (SRAM is only 2KB, so this probably won't work on
//      that device, but then why wouldya?).

#include "grbl.h"


#define EEPROM_SIZE  1024  //Bytes

//an erased eeprom reads-back as 0xff
// so, when we boot, this looks like an erased eeprom.
volatile unsigned char eeprom_in_ram[EEPROM_SIZE] = 
         { [0 ... ((EEPROM_SIZE)-1)] = 0xff }; 


/*! \brief  Read byte from EEPROM.
 *
 *  This function reads one byte from a given EEPROM address.
 *
 *  \param  addr  EEPROM address to read from.
 *  \return  The byte read from the EEPROM address.
 */
unsigned char eeprom_get_char( unsigned int addr )
{
   return eeprom_in_ram[addr];
}

/*! \brief  Write byte to EEPROM.
 *
 *  This function writes one byte to a given EEPROM address.
 *
 *  \param  addr  EEPROM address to write to.
 *  \param  new_value  New EEPROM value.
 */
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
   eeprom_in_ram[addr] = new_value;
}

// end of file
