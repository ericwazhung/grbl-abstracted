//While, generally, it's considered a "no-no" to #include a C file,
// That's one way to make this easy, while only requiring 
//  'SOURCE += eeprom_$(MCU_ARCH).c' in the Makefile...
// Should the time come when an external EEPROM chip is added, or
// (better-yet?) support for the PIC32's internal FLASH be implemented,
// Then this line should be removed.
#include "eeprom_in_ram.c"
