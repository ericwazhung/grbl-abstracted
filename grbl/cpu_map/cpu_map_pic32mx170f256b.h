/*
  cpu_map_pic32mx170f256.h - CPU and pin mapping configuration file
  Part of Grbl

  Copied and modified from cpu_map_atmega328p.h: 
   Copyright (c) 2012-2015 Sungeun K. Jeon
    Modified by Esot.Eric 2016
     notes/comments by Esot.Eric prefaced with "meh:"

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

/* Grbl officially supports the Arduino Uno, but the other supplied pin mappings are
   supplied by users, so your results may vary. */
/* meh:
   This file is for the PIC32MX170F256, rather'n an arduino.
   In so doing, there'll have to be quite a bit of abstraction done in the rest of the code
   We'll see where it goes... */

//meh: TODO: The other cpu_map_xxx.h files need to be modified, as
//appropriate... matching the 'meh' comments, here...

#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif

//meh: Apparently this is *only* used in these cpu_map_...h files, in the above test.
//     (per 'grep') 
#define GRBL_PLATFORM "PIC32MX170F256B"

#include "avr_compatibility.h"
//meh: This is new, see the file...
#include "port_handling.h"






//Stolen from Bipolarator5 (and modified accordingly):
// This is intended to match the "toupee2" pinout for TX/RX, Heart, etc.
//
// * = 5V-tolerant
// # = required
//
// 3V3      NOTE!!! a/o v4, this uses the NEW "toupee" pinout
//  ^
//  |
//  \                      uC
//  / 10K 'Rrst1'            PIC32
//  \                           MX170F256B            3V3
//  /  .1uF 'Crst1'          _________________         ^ 'CuCA'
//  |   ,                   |       |_|       |        | .1uF
//  +--,----------'nMCLR'---|1*#           #28|-'AVDD'-+--||--.
//  | ,    'Tx0'<-----------|2 RA0    AVSS #27|-'AGND'--------+-> GND
// ===   'X_LIMIT'>-(heart)-|3 RA1     RB15 26|--------->COOLANT_FLOOD
//  |          'RESET'>-----|4 RB0     RB14 25|-ppsOC3-->'Y_DIR'
//  | 'FEED_HOLD/DOOR'>-----|5 RB1     RB13 24|-ppsOC4-->'Y_STEP'
//  |    'CYCLE_START'>-----|6 RB2     RB12 23|------>'SPINDLE_DIRECTION' +
//  v          'PROBE'>-----|7 RB3    RB11 *22|-'jTMS'
// GND<------------'GND1'---|8# VSS   RB10 *21|--------->'SPINDLE_ENABLE' ?
//  ^      'Rx0'>-----------|9 RA2         #20|-'VCAP'--||--. 10uF 'CuCAP'
//  |    'Y_LIMIT'>---------|10 RA3    VSS #19|-'GND2'------+-> GND  
// ===    'X_STEP'<--ppsOC1-|11 RB4    RB9 *18|-'jTDO'-->'/STEP_DISABLE/'
//  | `  'Z_LIMIT'>---------|12 RA4    RB8 *17|-'jTCK'-->'/Z_DIR/'
//  +--`------------'VDD'---|13# V+    RB7 *16|-'jTDI'-->'/Z_STEP/'
//  |   `  'X_DIR'<--ppsOC2-|14* RB5   RB6 *15|-ppsOC5-->'SPINDLE_PWM'    +
//  v    `                  |_________________|  
// 3V3   .1uF 'CuC1'

// 'CuCAP' = 10uF Tantalum/Ceramic (Toupee2 uses two parallel 4.7uF Tant)

/*
Measurements...
Rx/Tx                OK

Floating Inputs...
RESET                High(PU)
FEED_HOLD/DOOR       High(PU)
CYCLE_START          High(PU)
PROBE                High(PU)

X_LIMIT              High(PU)
Y_LIMIT              OSCILLATOR OUTPUT....?!?!?!      RA3   CLKO
                        High(PU) after #pragma (OSCIOFNC=OFF)
Z_LIMIT              Floating...?! .5Vish             RA4

SPINDLE_PWM          OK    M5(spindle off)=Low
                           M3S4000 == ~100%
                           M3S0 == ~0%
SPINDLE_ENABLE       ???   (low)
SPINDLE_DIRECTION    OK    M3(CW)=Low  M4(CCW)=High


MUCH LATER:
Apparently Y_LIMIT == CLKO <--Disabled via pragma
Apparently Z_LIMIT == Secondary Oscillator <--Disabled via pragma
So, therefore, limits were detected... ...?
Oddly:
   Needed to enable BOTH (Tried: Y first, then Z, not reversed)
   Y was osc-out, so therefore reasonable to assume it was toggling
   But Z was NOT, relatively stable (but not high)
   Pin-CHANGE interrupt shouldn't've been activated, correct?
      (Maybe: SEI was already enabled, flag might've been raised during
              activation/initialization? Maybe, was *just* at threshold,
              and noise was the issue...? )
   ODDLY: Doesn't the limit-interrupt disable the entire system...?
         Wouldn't it have interfered with the serial communication and the
         processing of g-code?! Yet, the buffer was clearly advancing...
   EVEN MORE ODDLY: The limit-switches don't seem to have any effect, now.
         (Was it just continuously interrupting? Then why'd it still Rx/Tx?
          And apparently continue processing commands...)
          $20=0 (soft limits, bool)
          $21=0 (hard limits, bool)
          Something to do with initializing pcint before settings are
          loaded?



LATER:
   X_STEP/X_DIR Both Tested OK
   Y_STEP/Y_DIR Both Tested OK
*/


// Define serial port pins and interrupt vectors.
//meh: These interrupt-vector-things are AVR-specific
// They'll have to be handled explicitly in serial_pic32.c/h
//#define SERIAL_RX     USART_RX_vect
//#define SERIAL_UDRE   USART_UDRE_vect

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_PORT       PORTB
#define X_STEP_BIT      4   // 
#define Y_STEP_BIT      13  // 
#define Z_STEP_BIT      7   //This is a JTAG pin, so will go unaccessible 
#define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_PORT    PORTB
#define X_DIRECTION_BIT   5   //
#define Y_DIRECTION_BIT   14  //
#define Z_DIRECTION_BIT   8   //This is a JTAG pin, so will go unaccessible
#define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_PORT   PORTB
#define STEPPERS_DISABLE_BIT    9  //JTAG pin, so unaccessible
//#define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

// Define homing/hard limit switch input pins and limit interrupt vectors. 
// NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (CONTROL).
//PIC32: For now, bits 1,3,4 seem to make sense...
// limits_pic32.h is hardcoded for PORTA... changing this, here, will NOT
// work properly.
#define LIMIT_PORT       PORTA
#define X_LIMIT_BIT      1 
#define Y_LIMIT_BIT      3 
#ifndef __IGNORE_MEH_ERRORS__
#error "Have not looked into VARIABLE_SPINDLE"
#endif
#if 0 //def VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
  #define Z_LIMIT_BIT	   4 // Uno Digital Pin 12
#else
  #define Z_LIMIT_BIT    4 //3  // Uno Digital Pin 11
#endif
#define LIMIT_MASK       ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits


// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT   PORTB
// Z Limit pin and spindle PWM/enable pin swapped to access hardware PWM on Pin 11.
#ifndef __IGNORE_MEH_ERRORS__
#error "Haven't analyzed this VARIABLE_SPINDLE stuff"
#endif
#if 0 //def VARIABLE_SPINDLE 
  #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
    // If enabled, spindle direction pin now used as spindle enable, while PWM remains on D11.
    #define SPINDLE_ENABLE_BIT    5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)
  #else
    #define SPINDLE_ENABLE_BIT    3  // Uno Digital Pin 11
  #endif
#else
  #define SPINDLE_ENABLE_BIT    10
#endif
#ifndef __IGNORE_MEH_ERRORS__
#error "Also haven't analyzed this USE_SPINDLE_DIR_AS_ENABLE_PIN stuff"
#endif
#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
  #define SPINDLE_DIRECTION_PORT  PORTB
  #define SPINDLE_DIRECTION_BIT   12
#endif
 

// Define flood and mist coolant enable output pins.
// NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
// a later date if flash and memory space allows.
#define COOLANT_FLOOD_PORT  PORTB
#define COOLANT_FLOOD_BIT   15
#ifndef __IGNORE_MEH_ERRORS__
#error "Haven't looked too thoroughly into ENABLE_M7"
#endif
#ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
 #error "ENABLE_M7 is NYI for PIC32"
  #define COOLANT_MIST_PORT  PORTC
  #define COOLANT_MIST_BIT   4 // Uno Analog Pin 4
#endif  



// Define user-control controls (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
#define CONTROL_PORT      PORTB //PORTC
#define RESET_BIT         0
#define FEED_HOLD_BIT     1
#define CYCLE_START_BIT   2
#define SAFETY_DOOR_BIT   1 //NOTE: Safety door is shared with feed hold. Enabled by config define.
#define CONTROL_MASK ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))
#define CONTROL_INVERT_MASK CONTROL_MASK // May be re-defined to only invert certain control pins.
  
// Define probe switch input pin.
#define PROBE_PORT      PORTB
#define PROBE_BIT       3
//meh: TODO: This is kinda hokey, it's used, apparently, for XORing the
//bit-value... which could be less confusingly-implemented, and not require
//a definition in every cpu_map file... but for now, I'mma leave it.
#define PROBE_MASK      (1<<PROBE_BIT)

// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
  // Advanced Configuration Below You should not need to touch these variables
  #define PWM_MAX_VALUE    255.0
      
  // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
  //#define SPINDLE_PWM_DDR	  DDRB
  //THESE are defined in spindle_control_pic32.h
  // I THINK doing it there is more-reasonable, as doing-so here leaves one
  // with the feeling that by merely changing these values, it therefore
  // changes all the code to associate with the new pin-mapping. That Is
  // Not The Case.
  //#define SPINDLE_PWM_PORT  PORTB
  //#define SPINDLE_PWM_BIT	  3    // Uno Digital Pin 11
  //PIC32:
  //Let's use Timer 3 for the spindle PWM
  // The OC# and the Timer# do NOT need to match
  // Though it can be somewhat confusing to view...
  // I'll use:
  //   Timer3, with 
  //   Output-Compare Unit 5, mapped to
  //   pin RB13
  // NOTE: Changing these values, alone, is NOT enough to change the actual
  //       PWM pin! See Spindle_enablePWM(), etc.
#define SPINDLE_PWM_PORT   PORTB
#define SPINDLE_PWM_BIT    13

#endif // End of VARIABLE_SPINDLE
