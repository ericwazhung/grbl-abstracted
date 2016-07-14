/*
  nuts_bolts.h - Header file for shared definitions, variables, and functions
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon 
  Copyright (c) 2009-2011 Simen Svale Skogsrud 

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

#ifndef nuts_bolts_h
#define nuts_bolts_h


#include CONCAT_HEADER(nuts_bolts_,__MCU_ARCH__)


#ifndef __IGNORE_MEH_ERRORS__
#error "THIS NEEDS TO BE CHANGED"
#endif



#ifdef __XC32__
 //No joke, PIC32's xc32-gcc doesn't seem to have these functions...
 //see nuts_bolts_pic32.c for their definitions...

 //These are per 'man trunc' as returned by the Linux Programmer's Manual
 // 'If x is integral, +0, -0, NaN,  or infinite, x itself is returned.'

 //round x to the nearest integer not larger in absolute value
 //double trunc(double x);
 int32_t g_trunc(float x);

 //round x to the nearest integer (halfway cases away from zero)
 //  round(0.5) -> 1.0    round(-0.5) -> -1.0
 //double round(double x);
 uint16_t g_round(float x);

 //round x to the nearest integer value, away from zero
 //long int lround(double x);
 int32_t g_lround(float x);

#else
 //Because of that, I suppose, I'm looking into how these functions are
 //used, so as to see how they can be implemented...
 //And renaming them, as they're implemented less-fully than the originals

 #define g_trunc   trunc
 #define g_round   round
 #define g_lround  lround
#endif





#ifndef __AVR_ARCH__
 //AVR's _delay_ms() function is used directly only by mc_dwell
 // Not sure why they didn't use the local delay_ms() (without the '_'
 // prefix) except, maybe, that it is more accurate, since it uses a
 // constant value as an input to an inline function
 // Can't imagine it needs to be *so* precise, so on other (especially
 // faster) architectures, it should be acceptable to use the local
 // delay_ms instead.
 #define _delay_ms delay_ms
#endif




#define false 0
#define true 1

// Axis array index values. Must start with 0 and be continuous.
#define N_AXIS 3 // Number of axes
#define X_AXIS 0 // Axis indexing value. 
#define Y_AXIS 1
#define Z_AXIS 2
// #define A_AXIS 3

// CoreXY motor assignments. DO NOT ALTER.
// NOTE: If the A and B motor axis bindings are changed, this effects the CoreXY equations.
#ifdef COREXY
 #define A_MOTOR X_AXIS // Must be X_AXIS
 #define B_MOTOR Y_AXIS // Must be Y_AXIS
#endif

// Conversions
#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)


//This used to be TICKS_PER_MICROSECOND
//But some architectures are treated differently so see nuts_bolts_<arch>.h
//regarding TIMER_CLOCK_TICKS_PER_MICROSECOND
//THIS IS UGLAY... Problem is, it's used with some big-ol-math that is
//*right near* the maximum of its type... Found by multiplying F_CPU by 3
// (PIC32), causes an overflow-warning (which is only visible because of a
// lot of *constant* math). Hmm...
#define CPU_TICKS_PER_MICROSECOND (F_CPU/1000000)

 


// Useful macros
#define clear_vector(a) memset(a, 0, sizeof(a))
#define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

// Bit field and masking macros
#define bit(n) (1 << n) 


#ifdef __ORIGINAL_BIT_ATOMICS__

//Personally, I find these names confusing...
// is it a test or is it an assignment?
// setbit_atomic()
// clearbit_atomic()
// bit_is_true_atomic()... but maybe I'm just being pedantic.
//Handy routines, though!
#define bit_true_atomic(x,mask) {uint8_t sreg = SREG; cli(); (x) |= (mask); SREG = sreg; }


#define bit_false_atomic(x,mask) {uint8_t sreg = SREG; cli(); (x) &= ~(mask); SREG = sreg; }


#define bit_toggle_atomic(x,mask) {uint8_t sreg = SREG; cli(); (x) ^= (mask); SREG = sreg; }

#else

//Huh, hadn't considered that there's really no reason to mask the bits and
//use sei() at the end... Tremendous savings! Thanks for the idea, yo!
//Unfortunately, these operations are architecture-specific, so see
//CLI_SAFE and SEI_RESTORE in port_handling_<mcu>.h
#define bit_true_atomic(x,mask) \
{  \
   uint8_t CLI_SAFE(sreg); \
   (x) |= (mask); \
   SEI_RESTORE(sreg); \
}


#define bit_false_atomic(x,mask) \
{ \
   uint8_t CLI_SAFE(sreg); \
   (x) &= ~(mask); \
   SEI_RESTORE(sreg); \
}


#define bit_toggle_atomic(x,mask) \
{ \
   uint8_t CLI_SAFE(sreg); \
   (x) ^= (mask); \
   SEI_RESTORE(sreg); \
}  

#endif


#define bit_true(x,mask) (x) |= (mask)
#define bit_false(x,mask) (x) &= ~(mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)

// Read a floating point value from a string. Line points to the input buffer, char_counter 
// is the indexer pointing to the current character of the line, while float_ptr is 
// a pointer to the result variable. Returns true when it succeeds
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);

// Delays variable-defined milliseconds. Compiler compatibility fix for _delay_ms().
void delay_ms(uint16_t ms);

// Delays variable-defined microseconds. Compiler compatibility fix for _delay_us().
void delay_us(uint32_t us);

// Computes hypotenuse, avoiding avr-gcc's bloated version and the extra error checking.
float hypot_f(float x, float y);

#endif
