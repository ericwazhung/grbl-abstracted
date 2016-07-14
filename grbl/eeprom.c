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

#include "eeprom.h"

//NOTE: This file, and the associated eeprom_<architecture>.c/h files are
//specifically for storing non-volatile data; settings like steps/inch,
//etc.
// There's no reason this has to be stored to a microcontroller's internal EEPROM; 
// it could just as well be stored to an external I2C or SPI EEPROM chip
// or even to the microcontroller's internal (FLASH) program-memory
// But, it's up to you to implement those as appropriate in
// eeprom_<architecture>.c


#ifdef __BYTE_IDENTICAL_TEST__
 //#inclusion of C files is generally a no-no
 // This case is only for making certain the compilation is byte-identical
 // to the original grbl-master
 // (by keeping the function definitions in order)
 #include "eeprom_avr.c"
#endif

// Extensions added as part of Grbl 
// These functions are architecture-agnostic
// eeprom_put_char() etc, are not, and should be defined in e.g. eeprom_avr.c

void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = eeprom_get_char(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return(checksum == eeprom_get_char(source));
}

// end of file
