#include "grbl.h"

//######################################
// delay_us() and delay_ms() are in-use in grbl...
// These guys are blocking functions...
// In general, they use a bunch of NOPs, but that's a pain to implement.
//
// So, my idea is to use the PIC32's core-timer to create these delays
// The method using countReg_isItTime(), below, is designed to handle 
// wraparound-math... e.g. when the core-timer overflows, it should still
// work appropriately.
// countReg_isItTime() is *only* used by delay_us, and delay_ms for PIC32
// and is not a normal grbl thing.
//
// startCount is usually assigned, initially, by:
//   startCount = Count;
//  But, thereafter gets reassigned to account for the time at which it
//   would've isItTimed, without allowing cumulative-error
//  That's a bit overkill for these purposes, but it's reused code that's
//  been well-tested.

//#define F_CPU  48000000
//_mfc0(reg,sel) reads a value from the CP0 register reg,sel
//  So, Count is: "Count Register (CP0 Register 9, Select 0)"
#define getCountReg()   _mfc0(9,0) //_mfc0(_CP0_COUNT, _CP0_COUNT_SELECT)
//(_CP0_COUNT/SELECT also works, but just verifying my understanding...)
// See: http://www.microchip.com/forums/m640314.aspx
uint8_t countReg_isItTime(uint32_t *startCount, uint32_t deltaCount)
{
   uint32_t thisCount = getCountReg();

   uint32_t thisDelta = thisCount - *startCount;

   if(thisDelta >= deltaCount)
   {
      *startCount = thisCount - (thisDelta - deltaCount);
      return true;
   }
   else
      return false;
}


void delay_us(uint32_t us)
{
   //The Count Register always runs
   // Incrementing every-other clock-cycle
   // (1/2 FCPU, right?)
   //
   // with only the exception of optional in Debug Mode
   // and possibly some reason to set the DC bit in the Cause Register to 0
   //  (DC = Disable Count, for power-sensitive applications
   //       Power-Up-Default = 0 = Counting Enabled 

   //NOTE: It *is* possible for another function to *set* the
   //count-register... I haven't done this, but if it's manipulated, at
   //some time later down the line, then this will not work appropriately

   // CPU Clock: 48,000,000 Cycles / sec
   // Count Rate: 1 count / 2 cycles
   // 1,000,000 us / sec
   //
   // counts = Time <us> * (1 sec / 1,000,000 us) -> sec
   //          * F_CPU <48,000,000 cycles / sec> -> CPU Cycles
   //          * ClockRate <1count/2cycles> -> counts
   //
   // counts = Time <us> * F_CPU / ClockRate / 1,000,000
   uint32_t counts = us * (uint64_t)F_CPU/2000000;
   //FYI the count register is 32 bits
   //    so it overflows at a rate of ~4.3billion counts
   //    or ~89*2 seconds at 48MHz.
   // So, let's increase it to uint64 prior to the division
   // but as long as it's less than 89 seconds, it will fit into uint32

   uint32_t startCount = getCountReg();

   while( !countReg_isItTime(&startCount, counts) )
   {
      asm("nop;");   //just make sure it doesn't optimze-away
   }
}
// Here we go... just assuming we're never going to set it to 89seconds*2
void delay_ms(uint16_t ms)
{
   delay_us( ms * 1000 );
}


//Believe it or not, trunc(), round(), and lround() are apparently not
//available in xc32-gcc...
// see also: nuts_bolts.h

//These are per 'man trunc' as returned by the Linux Programmer's Manual
// 'If x is integral, +0, -0, NaN,  or infinite, x itself is returned.'

//"round x to the nearest integer not larger in absolute value"
//uses:
//  system.c: uint8_t helper_var = trunc(float parameter);
//  gcode.c:  uint8_t int_value = trunc(float value);
//            int32_t gc_block.values.n = trunc(float value);
//             parser_block_t . gc_values_t . int32
//            uint8_t coord_select = trunc(float gc_block.values.p);
//  settings.c: uint8_t int_value = trunc(float value);
//double trunc(double x)
int32_t g_trunc(float x)
{
   //Tested in cTests truncToInt.c
   // This seems to do the job, as described
   // except, of course, with floating point we might get some weird-cases
   // e.g. -1.000000 -> 0 (?!) when stepped down from 2 in steps of 0.1
   // but goes to -1 when manually-assigned.
   // Gotta love floating-point.
   return (int32_t)x;
}

//"round x to the nearest integer (halfway cases away from zero)"
//  "round(0.5) -> 1.0    round(-0.5) -> -1.0"
//uses:
// gcode.c: uint16_t mantissa = round(100*(float value - uint8_t int_value));
//double round(double x)
uint16_t g_round(float x)
{
   //But since it's only going to a uint16, we don't need to watch the sign
   return (uint16_t)(x+0.5);
}

//"round x to the nearest integer value, away from zero"
//uses:
//limits.c:          set_axis_position = lround((settings.max_travel[idx]+settings.homing_pulloff)*settings.steps_per_mm[idx]);
//                         int32_t     = lround( float )
//limits.c:          set_axis_position = lround(-settings.homing_pulloff*settings.steps_per_mm[idx]);
//                         int32_t     = lround( float )
//planner.c:    target_steps[A_MOTOR] = lround(target[A_MOTOR]*settings.steps_per_mm[A_MOTOR]);
//planner.c:    target_steps[B_MOTOR] = lround(target[B_MOTOR]*settings.steps_per_mm[B_MOTOR]);
//planner.c:        target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
//planner.c:      target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
//                         int32_t   = lround (float)
//long int lround(double x)
int32_t g_lround(float x)
{
   //Basically the same as g_round() except we need to watch for signs and return a larger integer type
   if(x >= 0)
   {
      return (int32_t)(x+0.5);
   }
   else 
   {
      return (int32_t)(x-0.5);
   }
}




