/*
  grbl.h - main Grbl include file
  Part of Grbl

  Copyright (c) 2015 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef grbl_h
#define grbl_h



//__BYTE_IDENTICAL_TEST__ is assigned in Makefile, not here...
// via -D option and BYTE_IDENTICAL_TEST (no pre/post __'s)
// BECAUSE some of the source-files need to be changed, as well.

//If this is defined, this version of grbl should compile to a
//byte-for-byte identical compilation of the original grbl
//See the notes, below, to see what changes are made, otherwise.
#ifdef __BYTE_IDENTICAL_TEST__

 //If this is defined, then we'll use the "unsafe" version of writeMasked
 // This version does *NOT* assure that your value actually fits within the
 // masked-bits, so might write-over bits outside the mask
 //The ONLY reason this version exists, at all, is to do comparisons between
 // grbl-master and my abstracted-version
 //With This Defined, so far, my abstracted-version is literally
 // byte-for-byte identical to the original grbl-master compilation.
 #define __WRITE_MASKED__UNSAFE__

 //my errors are notes, more like warnings, about things I need to finish...
 //With this defined, those messages are hidden so compilation can be tested
 //despite intermediate-steps
// #define __IGNORE_MEH_ERRORS__
//This guy's in Makefile...



 //I think I found a mutex-error, and have created a fix. However, for
 //comparison with grbl-master, this needs to be defined
 //NOTE: grbl-master's Default is to NOT use STEP_PULSE_DELAY
 //      So this has no effect either way
 //      AND this is untested.
 #define __ORIGINAL_STEP_BITS__

 //I've modified bit_xxx_atomic() to use CLI_SAFE() and SEI_RESTORE()
 //To use the original versions:
// #define __ORIGINAL_BIT_ATOMICS__

#endif //__BYTE_IDENTICAL_TEST__





// Grbl versioning system
#define GRBL_VERSION "0.9j"
#define GRBL_VERSION_BUILD "20160317"

// Define standard libraries used by Grbl.

//AVR-specific libraries
#ifdef __AVR_ARCH__

 #include <avr/io.h>
 #include <avr/pgmspace.h>
 #include <avr/interrupt.h>
 #include <avr/wdt.h>
 #include <util/delay.h>

//PIC32-specific libraries (using xc32-gcc)
#elif defined( __XC32__ )

 #if (!defined(__XC32_VERSION) || (__XC32_VERSION < 1420))
  #error "This version of xc32-gcc is untested. Beware of the 'optimizer math bug' found in the linux version of xc32-gcc v1.40 which WILL cause problems, here. This bug was fixed as-of v1.42. Remove this error with caution."
 #endif

 //For PIC32, so far, grbl's only been implemented with xc32-gcc
 // thus, some of its specific libraries are being used
 #include <xc.h>
 //#include <sys/attribs.h> //for interrupt vectoring with __ISR()
 #include <stdio.h> //Instead of direct register read/writes I've opted to
                    //use _mon_getc and _mon_putc... stdio, I think,
                    //is not used elsewhere in grbl

#else
 #error " your architecture/compiler combo is not yet supported! Check out cpu_map.h, serial.h, etc. for examples. And throw your necessary #includes here."
#endif


#include <math.h>
#include <inttypes.h>    
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

// Define the Grbl system include files. NOTE: Do not alter organization.
#include "stringify.h"
#include "config.h"
#include "nuts_bolts.h"
#include "settings.h"
#include "system.h"
#include "defaults.h"
#include "cpu_map.h"
#include "coolant_control.h"
#include "eeprom.h"
#include "gcode.h"
#include "limits.h"
#include "motion_control.h"
#include "planner.h"
#include "print.h"
#include "probe.h"
#include "protocol.h"
#include "report.h"
#include "serial.h"
#include "spindle_control.h"
#include "stepper.h"

#endif
