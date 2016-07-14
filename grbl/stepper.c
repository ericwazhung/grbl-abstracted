/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
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

#include "grbl.h"
#include CONCAT_HEADER(stepper_,__MCU_ARCH__)


#ifndef __BYTE_IDENTICAL_TEST__
//Needed for some of the new functions...
static gpioPortWidth_t dir_port_invert_mask;
#endif


//See these definitions in stepper.h and the selection in config.h
#if( (STEPPER_INTERFACE == STEPPER_INTERFACE__STEP_DIR) \
     || !defined(STEPPER_INTERFACE) )
 //TODO: These names suck.

 #define STEPPER_SetDirections(directionBits) \
      writePORToutputs_masked(DIRECTION_PORT, DIRECTION_MASK, \
            ((directionBits) & DIRECTION_MASK) )

 #define STEPPER_SetStepBits(stepBits) \
    writePORToutputs_masked(STEP_PORT, STEP_MASK, (stepBits))

 //TODO: This is confusing... step_port_invert_mask is clearly used here
 // But wasn't so clearly-used in/leading-up-to SetStepBits...
 #define STEPPER_ClearStepBits() \
  writePORToutputs_masked(STEP_PORT, STEP_MASK, (step_port_invert_mask & STEP_MASK) );

#else //NOT using the officially supported Step/Dir interface

 //To make it backwards-compatible, we'll just tack this on as a new bit of
 //code, rather'n rewriting the existing code to support the new interface
 //This isn't *so* crazy, any motor would have a direction and a position
 //Calling it a "step" to, e.g., increment a DC-motor one encoder-tick is
 //just a matter of naming...

 //However: grbl uses the pin-masks to track the step/direction bits in
 // most cases...
 // (rather'n, say, always using bit0=X, bit1=Y, bit2=Z, regardless of
 //  which pin they're actually wired-to)

 //SO: again, rather'n changing all the code, we'll work with that standard
 //    and therefore will use variables of type gpioPortWidth_t to store
 //    that information, despite the fact it will NOT be used with a gpio.

 //We need to track the direction, as it's set up *before* the step-command
 //is issued
 //It's entirely plausible we could use the original variables for these
 //things, but this adds a level of abstraction, ish, or something. Maybe
 //I'm just lazy.
 //TODO: Am making this "global" for all non step/dir interfaces
 //      But, it may make sense to make it specific to the interface
 //      (again, e.g. DC-motors with encoders would likely have a
 //       direction-input on their H-bridge chip... ToPonder)
 //TODO: Similarly, it might make sense to have STEP_PULSE_DELAY be an
 //      option for such motor/H-bridge combos, as well... hmmmm.... NYI
// volatile gpioPortWidth_t  directionBits;
// #define STEPPER_SetDirections(dirBitsIn) (directionBits = ((dirBitsIn) & DIRECTION_MASK))


 //NO: Too friggin' confusing...
 // COULD probably get rid of the whole wonky-bit-arrangement thing,
 // altogether... but let's, instead, for now, just normalize what we've
 // got coming in...
 
 //Bits arranged per X_AXIS, etc. definitions...
 volatile uint8_t directions;
 
 void STEPPER_SetDirections(gpioPortWidth_t dirBitsIn)
 {
    //Undo whatever doings may've been done before.
    dirBitsIn ^= dir_port_invert_mask;

    uint8_t dirTemp=0;

    uint8_t axis;
    for(axis=X_AXIS; axis<=Z_AXIS; axis++)
    {
      if(dirBitsIn & get_direction_pin_mask(axis))
         dirTemp |= (1<<axis);
    }

    directions = dirTemp;
 }



 #if ( (STEPPER_INTERFACE == STEPPER_INTERFACE__PHASE_AB) \
       || (STEPPER_INTERFACE == STEPPER_INTERFACE__PHASE_AB_PWM) )

  //So, this guy is originally used to set the STEP bits on the outputs
  // In other words, this guy activates the step-pulse for axes which are
  // to be moved by a step...
  // but, when not using a Step/Dir interface, this guy is essentially the
  // invoking of the advancing of the motor by one step, as appropriate.
  //THUS: in PHASE_AB mode, this is the function which increments or
  //decrements the phase, as appropriate.
  //AGAIN: its input is "outbits" which are NOT 0->2==X->Z
  void STEPPER_SetStepBits(gpioPortWidth_t stepBitsIn)
  {
      //stepperPhase is just an integer corresponding, roughly, to...
      // not the *position*, per se...
      // It kinda represents the phase of one of the windings
      // (where the other winding is 90degrees out of phase from that)
      // But... since...
      //  there's no real reason to limit its value
      //  Having a hard time explaining this.
      //   E.G. Corresponding to FULL-STEPPING, there are four phases
      //        or combinations of outputs that result in four full steps
      //        Those "phases" are output in sequence and repeat
      //   E.G.2. Corresponding to MICRO-STEPPING, there may be any number
      //        of micro-steps between full steps, but as with
      //        full-stepping there are four specific phases which
      //        correspond to full-steps which must occur in sequence and
      //        repeat. Thus, say there's 4 microsteps between each full
      //        step, then the stepperPhase would advance 16 times before
      //        repeating.
      // But, again, there's no need to *reset* stepperPhase after a cycle
      // completes, because it's in an integer which will overflow properly
      // (and underflow, as well), as long as
      // 256 % (stepsPerCycle(=4) * microstepsPerStep) == 0
      // Which it will.
      // The values of stepperPhase[axis1] and stepperPhase[axis2] are
      // never compared, either mathematically, programmatically, nor in
      // any physical sense, so they needn't be of the same "units"
      // (full-steps vs. microsteps, etc.)
      // Except, of course, that a microstepped phase of, say 3 might be
      // less than a full step, whereas a full-stepped phase of 3 would be
      // three full steps... Generally irrelevent, the axes' steps/mm would
      // have to be adjusted accordingly, anyhow.
      // (Where steps/mm corresponds NOT to FULL steps, but to increments
      // in position (steps, or microsteps, as appropriate)).
      static uint8_t stepperPhase[3];

      gpioPortWidth_t phaseOutBits = 0;


      uint8_t axis;
      for(axis=X_AXIS; axis<=Z_AXIS; axis++)
      {
         if(stepBitsIn & get_step_pin_mask(axis))
         {
            //We need to take a step...
            // one direction or the other...
            //Note that direction is positive when zero and negative when 1
            // per something I read somewhere
            // (TODO: Does This Get Affected By invert-masks/bits???)
            if(!(directions & (1<<axis)))
               stepperPhase[axis]++;
            else
               stepperPhase[axis]--;
         }


        #if (STEPPER_INTERFACE == STEPPER_INTERFACE__PHASE_AB_PWM)
         //If we have pwm-handling/microstepping for the axis, handle that
         // otherwise, use single-stepping
         if( (1<<axis) & PHASE_AB_PWM__AXES_MASK ) 
         {
            Stepper_setPWMfromPhase(axis, stepperPhase[axis]);
         }
         else
        #endif
         {
         //Assuming all axes' phase-output bits are on the same port...
         //NOTE: we're using the Step/Dir output bits as phase outputs
         // STEP = PHASE_A
         // DIR = PHASE_B
         // So, we can use get_step/direction_pin_mask() to assign the
         // appropriate bits' output-values.
         switch(stepperPhase[axis]%4)
         {
            //Step/Phase Order: AB = 00 01 11 10
            // Some sorta vague idea that the phase-value could be used to
            // directly calculate the phase-output (without a table) e.g.
            // using the second bit of the phase-value for one output
            // and adding one to the phase-value, then taking (again) the
            // second-bit of the (new) phase-value for the other output...?
            // But I can't quite wrap my head around it right now.

            // 00
            case 0:
               //Nothin' To Do.
               break;
            // 01
            case 1:
               //A=0, nothing to do
               //B=1: B=DIR...
               phaseOutBits |= get_direction_pin_mask(axis);
               break;
            // 11
            case 2:
               //A=1: A=STEP...
               phaseOutBits |= get_step_pin_mask(axis);
               //B=1: B=DIR...
               phaseOutBits |= get_direction_pin_mask(axis);
               break;
            // 10
            case 3:
            default:
               //A=1: A=STEP...
               phaseOutBits |= get_step_pin_mask(axis);
               //B=0, nothing to do.
               break;
         }
         }
      }

      //Write The STEP/DIR outputs WITH PHASE OUTPUT VALUES.
      writePORToutputs_masked(STEP_PORT, (STEP_MASK | DIRECTION_MASK) , phaseOutBits);
  }

 //Nothing to do, here...
 //TODO: Then there's no reason to have Timer0 running at all... hmmm...
 #define STEPPER_ClearStepBits() {}

 #else
  #error "WTF? This isn't a valid STEPPER_INTERFACE"
 #endif
#endif






// Some useful constants.
#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // min/segment 
#define REQ_MM_INCREMENT_SCALAR 1.25                                   
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2

// Define Adaptive Multi-Axis Step-Smoothing(AMASS) levels and cutoff frequencies. The highest level
// frequency bin starts at 0Hz and ends at its cutoff frequency. The next lower level frequency bin
// starts at the next higher cutoff frequency, and so on. The cutoff frequencies for each level must
// be considered carefully against how much it over-drives the stepper ISR, the accuracy of the 16-bit
// timer, and the CPU overhead. Level 0 (no AMASS, normal operation) frequency bin starts at the 
// Level 1 cutoff frequency and up to as fast as the CPU allows (over 30kHz in limited testing).
// NOTE: AMASS cutoff frequency multiplied by ISR overdrive factor must not exceed maximum step frequency.
// NOTE: Current settings are set to overdrive the ISR to no more than 16kHz, balancing CPU overhead
// and timer accuracy.  Do not alter these settings unless you know what you are doing.
#define MAX_AMASS_LEVEL 3
// AMASS_LEVEL0: Normal operation. No AMASS. No upper cutoff frequency. Starts at LEVEL1 cutoff frequency.
#define AMASS_LEVEL1 (F_CPU/8000) // Over-drives ISR (x2). Defined as F_CPU/(Cutoff frequency in Hz)
#define AMASS_LEVEL2 (F_CPU/4000) // Over-drives ISR (x4)
#define AMASS_LEVEL3 (F_CPU/2000) // Over-drives ISR (x8)


// Stores the planner block Bresenham algorithm execution data for the segments in the segment 
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use. 
typedef struct {  
  gpioPortWidth_t direction_bits;
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
} st_block_t;

static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper 
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by 
// the planner, where the remaining planner block steps still can.
typedef struct {
  uint16_t n_step;          // Number of step events to be executed for this segment
  uint8_t st_block_index;   // Stepper block data index. Uses this information to execute this segment.
  uint16_t cycles_per_tick; // Step distance traveled per ISR tick, aka step rate.
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
  #else
    uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
  #endif
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct {
  // Used by the bresenham line algorithm
  uint32_t counter_x,        // Counter variables for the bresenham line tracer
           counter_y, 
           counter_z;
  #ifdef STEP_PULSE_DELAY
    gpioPortWidth_t step_bits;  // Stores out_bits output to complete the step pulse delay
  #endif
  
  uint8_t execute_step;     // Flags step execution for each interrupt.
  uint8_t step_pulse_time;  // Step pulse reset time after step rise
  gpioPortWidth_t step_outbits;         // The next stepping-bits to be output
  gpioPortWidth_t dir_outbits;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif

  uint16_t step_count;       // Steps remaining in line segment motion  
  uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.
  st_block_t *exec_block;   // Pointer to the block data for the segment being executed
  segment_t *exec_segment;  // Pointer to the segment being executed
} stepper_t;

static stepper_t st;

// Step segment ring buffer indices
static volatile uint8_t segment_buffer_tail;
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

// Step and direction port invert masks. 
static gpioPortWidth_t step_port_invert_mask;


//Moved up near the top
#ifdef __BYTE_IDENTICAL_TEST__
 //Except, relocation in memory-space causes BYTE_IDENTICAL_TEST to fail...
 static gpioPortWidth_t dir_port_invert_mask;
#endif


// Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
static volatile uint8_t busy;   

// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
static plan_block_t *pl_block;     // Pointer to the planner block being prepped
static st_block_t *st_prep_block;  // Pointer to the stepper block data being prepped 

// Segment preparation data struct. Contains all the necessary information to compute new segments
// based on the current executing planner block.
typedef struct {
  uint8_t st_block_index;  // Index of stepper common data block being prepped
  uint8_t flag_partial_block;  // Flag indicating the last block completed. Time to load a new one.

  float steps_remaining;
  float step_per_mm;           // Current planner block step/millimeter conversion scalar
  float req_mm_increment;
  float dt_remainder;
  
  uint8_t ramp_type;      // Current segment ramp state
  float mm_complete;      // End of velocity profile from end of current planner block in (mm).
                          // NOTE: This value must coincide with a step(no mantissa) when converted.
  float current_speed;    // Current speed at the end of the segment buffer (mm/min)
  float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
  float exit_speed;       // Exit speed of executing block (mm/min)
  float accelerate_until; // Acceleration ramp end measured from end of block (mm)
  float decelerate_after; // Deceleration ramp start measured from end of block (mm)
} st_prep_t;
static st_prep_t prep;


/*    BLOCK VELOCITY PROFILE DEFINITION 
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  time ----->      EXAMPLE: Block 2 entry speed is at max junction velocity
  
  The planner block buffer is planned assuming constant acceleration velocity profiles and are
  continuously joined at block junctions as shown above. However, the planner only actively computes
  the block entry speeds for an optimal velocity plan, but does not compute the block internal
  velocity profiles. These velocity profiles are computed ad-hoc as they are executed by the 
  stepper algorithm and consists of only 7 possible types of profiles: cruise-only, cruise-
  deceleration, acceleration-cruise, acceleration-only, deceleration-only, full-trapezoid, and 
  triangle(no cruise).

                                        maximum_speed (< nominal_speed) ->  + 
                    +--------+ <- maximum_speed (= nominal_speed)          /|\                                         
                   /          \                                           / | \                      
 current_speed -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |                       
                  +-------------+                     current_speed -> +----+--+                   
                   time -->  ^  ^                                           ^  ^                       
                             |  |                                           |  |                       
                decelerate_after(in mm)                             decelerate_after(in mm)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                accelerate_until(in mm)                             accelerate_until(in mm)
                    
  The step segment buffer computes the executing block velocity profile and tracks the critical
  parameters for the stepper algorithm to accurately trace the profile. These critical parameters 
  are shown and defined in the above illustration.
*/


// Stepper state initialization. Cycle should only start if the st.cycle_start flag is
// enabled. Startup init and limits call this function but shouldn't start the cycle.
void st_wake_up() 
{
  // Enable stepper drivers.
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) 
  { 
     //STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); 
     setPORTpin(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_BIT);
  }
  else 
  { 
     //STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); 
     clearPORTpin(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_BIT);
  }

  if (sys.state & (STATE_CYCLE | STATE_HOMING)){
    // Initialize stepper output bits
    st.dir_outbits = dir_port_invert_mask; 
    st.step_outbits = step_port_invert_mask;
    
    // Initialize step pulse timing from settings. Here to ensure updating after re-writing.
    #ifdef STEP_PULSE_DELAY
      // Set total step pulse time after direction pin set. Ad hoc computation from oscilloscope.
      st.step_pulse_time = -(((settings.pulse_microseconds+STEP_PULSE_DELAY-2)*TIMER_CLOCK_TICKS_PER_MICROSECOND) >> 3);
      // Set delay between direction pin write and step command.
      //OCR0A = -(((settings.pulse_microseconds)*TICKS_PER_MICROSECOND) >> 3);
      TIMER0A_setOuputCompareValue(
         -(((settings.pulse_microseconds)*TIMER_CLOCK_TICKS_PER_MICROSECOND) >> 3) );
    #else // Normal operation
      // Set step pulse time. Ad hoc computation from oscilloscope. Uses two's complement.
      st.step_pulse_time = -(((settings.pulse_microseconds-2)*TIMER_CLOCK_TICKS_PER_MICROSECOND) >> 3);
    #endif

    // Enable Stepper Driver Interrupt
    //TIMSK1 |= (1<<OCIE1A);
    TIMER1A_enableOutputCompareInterrupt();
  }
}


// Stepper shutdown
void st_go_idle() 
{
  // Disable Stepper Driver Interrupt. Allow Stepper Port Reset Interrupt to finish, if active.
  //TIMSK1 &= ~(1<<OCIE1A); // Disable Timer1 interrupt
  TIMER1A_disableOutputCompareInterrupt();

  //TODO: Wait, what? We want the clock to count at full-speed when Idle?
  //      Is this because the timer-interrupt is responsible for setting up
  //      the next timer-interrupt time...? So, interrupt quickly so it'll
  //      be ready as quickly as possible...?
  //TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // Reset clock to no prescaling.
  TIMER1_fullSpeed();
  
  busy = false;
  
  // Set stepper driver idle state, disabled or enabled, depending on settings and circumstances.
  bool pin_state = false; // Keep enabled.
  if (((settings.stepper_idle_lock_time != 0xff) || sys_rt_exec_alarm) && sys.state != STATE_HOMING) {
    // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
    // stop and not drift from residual inertial forces at the end of the last movement.
    delay_ms(settings.stepper_idle_lock_time);
    pin_state = true; // Override. Disable steppers.
  }
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { pin_state = !pin_state; } // Apply pin invert.
  if (pin_state) 
  { 
#ifndef __IGNORE_MEH_ERRORS__
#error "is this interruptable?"
#endif
     //STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); 
     setPORTpin(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_BIT);
  }
  else 
  { 
     //STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT);
     clearPORTpin(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_BIT);
  }
}


/* "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. Grbl employs
   the venerable Bresenham line algorithm to manage and exactly synchronize multi-axis moves.
   Unlike the popular DDA algorithm, the Bresenham algorithm is not susceptible to numerical
   round-off errors and only requires fast integer counters, meaning low computational overhead
   and maximizing the Arduino's capabilities. However, the downside of the Bresenham algorithm
   is, for certain multi-axis motions, the non-dominant axes may suffer from un-smooth step 
   pulse trains, or aliasing, which can lead to strange audible noises or shaking. This is 
   particularly noticeable or may cause motion issues at low step frequencies (0-5kHz), but 
   is usually not a physical problem at higher frequencies, although audible.
     To improve Bresenham multi-axis performance, Grbl uses what we call an Adaptive Multi-Axis
   Step Smoothing (AMASS) algorithm, which does what the name implies. At lower step frequencies,
   AMASS artificially increases the Bresenham resolution without effecting the algorithm's 
   innate exactness. AMASS adapts its resolution levels automatically depending on the step
   frequency to be executed, meaning that for even lower step frequencies the step smoothing 
   level increases. Algorithmically, AMASS is acheived by a simple bit-shifting of the Bresenham
   step count for each AMASS level. For example, for a Level 1 step smoothing, we bit shift 
   the Bresenham step event count, effectively multiplying it by 2, while the axis step counts 
   remain the same, and then double the stepper ISR frequency. In effect, we are allowing the
   non-dominant Bresenham axes step in the intermediate ISR tick, while the dominant axis is 
   stepping every two ISR ticks, rather than every ISR tick in the traditional sense. At AMASS
   Level 2, we simply bit-shift again, so the non-dominant Bresenham axes can step within any 
   of the four ISR ticks, the dominant axis steps every four ISR ticks, and quadruple the 
   stepper ISR frequency. And so on. This, in effect, virtually eliminates multi-axis aliasing 
   issues with the Bresenham algorithm and does not significantly alter Grbl's performance, but 
   in fact, more efficiently utilizes unused CPU cycles overall throughout all configurations.
     AMASS retains the Bresenham algorithm exactness by requiring that it always executes a full
   Bresenham step, regardless of AMASS Level. Meaning that for an AMASS Level 2, all four 
   intermediate steps must be completed such that baseline Bresenham (Level 0) count is always 
   retained. Similarly, AMASS Level 3 means all eight intermediate steps must be executed. 
   Although the AMASS Levels are in reality arbitrary, where the baseline Bresenham counts can
   be multiplied by any integer value, multiplication by powers of two are simply used to ease 
   CPU overhead with bitshift integer operations. 
     This interrupt is simple and dumb by design. All the computational heavy-lifting, as in
   determining accelerations, is performed elsewhere. This interrupt pops pre-computed segments,
   defined as constant velocity over n number of steps, from the step segment buffer and then 
   executes them by pulsing the stepper pins appropriately via the Bresenham algorithm. This 
   ISR is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port
   after each pulse. The bresenham line tracer algorithm controls all stepper outputs
   simultaneously with these two interrupts.
   
   NOTE: This interrupt must be as efficient as possible and complete before the next ISR tick, 
   which for Grbl must be less than 33.3usec (@30kHz ISR rate). Oscilloscope measured time in 
   ISR is 5usec typical and 25usec maximum, well below requirement.
   NOTE: This ISR expects at least one step to be executed per segment.
*/
// TODO: Replace direct updating of the int32 position counters in the ISR somehow. Perhaps use smaller
// int8 variables and update position counters only when a segment completes. This can get complicated 
// with probing and homing cycles that require true real-time positions.
//ISR(TIMER1_COMPA_vect)
TIMER1A_OutputCompareInterruptHandler()
{        
  

   //This is necessary on e.g. PIC32, but not on AVR, so on AVR, it will
   //compile to nada. No extra instructions.
   TIMER1A_clearOutputCompareFlag();

   //meh: This was already commented-out... so not analyzed.
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT; // Debug: Used to time ISR
  if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt
  
  // Set the direction pins a couple of nanoseconds before we step the steppers
  //DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK);
  //writePORToutputs_masked(DIRECTION_PORT, DIRECTION_MASK, (st.dir_outbits & DIRECTION_MASK) );
   STEPPER_SetDirections(st.dir_outbits);

  // Then pulse the stepping pins
  #ifdef STEP_PULSE_DELAY
   
   #if( (STEPPER_INTERFACE != STEPPER_INTERFACE__STEP_DIR) \
        && defined(STEPPER_INTERFACE) )
    #error "STEP_PULSE_DELAY is NYI for stepper-interfaces other than step/dir"
   #endif
    //meh: NO!
   #ifndef __IGNORE_MEH_ERRORS__
    #error "Functionality-change, here, by meh..."
   #endif
    // What's to stop STEP_PORT's *other* pins from being changed between
    // this interrupt and the next...?!
   #ifdef __ORIGINAL_STEP_BITS__
    
    //st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // Store out_bits to prevent overwriting.
    st.step_bits = (readoutputsPORT(STEP_PORT) & ~STEP_MASK) | st.step_outbits; // Store out_bits to prevent overwriting.
   #else
    #error "This is UNVERIFIED, so either use __ORIGINAL_STEP_BITS__ or remove this error and see what happens..."
    st.step_bits = st.step_outbits; // Buffer outbits to prevent overwriting
   #endif


  #else  // Normal operation (NOT STEP_PULSE_DELAY)
    //STEP_PORT = (STEP_PORT & ~STEP_MASK) | st.step_outbits;
    //writePORToutputs_masked(STEP_PORT, STEP_MASK, st.step_outbits);
    STEPPER_SetStepBits(st.step_outbits);
  #endif  

  // Enable step pulse reset timer so that The Stepper Port Reset Interrupt can reset the signal after
  // exactly settings.pulse_microseconds microseconds, independent of the main Timer1 prescaler.
  //TODO: Confusing... it's not counting *down*, it's counting *up* from
  //here...
  //TCNT0 = st.step_pulse_time; // Reload Timer0 counter
  TIMER0_loadCount( st.step_pulse_time );

  //TCCR0B = (1<<CS01); // Begin Timer0. Full speed(?), 1/8 prescaler
  TIMER0_startDiv8();

  busy = true;
  sei(); // Re-enable interrupts to allow Stepper Port Reset Interrupt to fire on-time. 
         // NOTE: The remaining code in this ISR will finish before returning to main program.
    
  // If there is no step segment, attempt to pop one from the stepper buffer
  if (st.exec_segment == NULL) {
    // Anything in the buffer? If so, load and initialize next step segment.
    if (segment_buffer_head != segment_buffer_tail) {
      // Initialize new step segment and load number of steps to execute
      st.exec_segment = &segment_buffer[segment_buffer_tail];

      #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS is disabled, set timer prescaler for segments with slow step frequencies (< 250Hz).
        //TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (st.exec_segment->prescaler<<CS10);
        TIMER1_setPrescaler(st.exec_segment->prescaler);
      #endif

      // Initialize step segment timing per step and load number of steps to execute.
      //OCR1A = st.exec_segment->cycles_per_tick;
      TIMER1A_setCompareMatchValue( st.exec_segment->cycles_per_tick );

      st.step_count = st.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.
      // If the new segment starts a new planner block, initialize stepper variables and counters.
      // NOTE: When the segment data index changes, this indicates a new planner block.
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];
        
        // Initialize Bresenham line and distance counters
        st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
      }
      st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask; 

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level.
        st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
        st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
        st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
      #endif
      
    } else {
      // Segment buffer empty. Shutdown.
      st_go_idle();
      bit_true_atomic(sys_rt_exec_state,EXEC_CYCLE_STOP); // Flag main program for cycle end
      return; // Nothing to do but exit.
    }  
  }
  
  
  // Check probing state.
  probe_state_monitor();
   
  // Reset step out bits.
  st.step_outbits = 0; 

  // Execute step displacement profile by Bresenham line algorithm
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_x += st.steps[X_AXIS];
  #else
    st.counter_x += st.exec_block->steps[X_AXIS];
  #endif  
  if (st.counter_x > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<X_STEP_BIT);
    st.counter_x -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<X_DIRECTION_BIT)) { sys.position[X_AXIS]--; }
    else { sys.position[X_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_y += st.steps[Y_AXIS];
  #else
    st.counter_y += st.exec_block->steps[Y_AXIS];
  #endif    
  if (st.counter_y > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Y_STEP_BIT);
    st.counter_y -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Y_DIRECTION_BIT)) { sys.position[Y_AXIS]--; }
    else { sys.position[Y_AXIS]++; }
  }
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    st.counter_z += st.steps[Z_AXIS];
  #else
    st.counter_z += st.exec_block->steps[Z_AXIS];
  #endif  
  if (st.counter_z > st.exec_block->step_event_count) {
    st.step_outbits |= (1<<Z_STEP_BIT);
    st.counter_z -= st.exec_block->step_event_count;
    if (st.exec_block->direction_bits & (1<<Z_DIRECTION_BIT)) { sys.position[Z_AXIS]--; }
    else { sys.position[Z_AXIS]++; }
  }  

  // During a homing cycle, lock out and prevent desired axes from moving.
  if (sys.state == STATE_HOMING) { st.step_outbits &= sys.homing_axis_lock; }   

  st.step_count--; // Decrement step events count 
  if (st.step_count == 0) {
    // Segment is complete. Discard current segment and advance segment indexing.
    st.exec_segment = NULL;
    if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
  }

  st.step_outbits ^= step_port_invert_mask;  // Apply step port invert mask    
  busy = false;
   //meh: This was already commented-out, so not analyzed...
// SPINDLE_ENABLE_PORT ^= 1<<SPINDLE_ENABLE_BIT; // Debug: Used to time ISR
}


/* The Stepper Port Reset Interrupt: Timer0 OVF interrupt handles the falling edge of the step
   pulse. This should always trigger before the next Timer1 COMPA interrupt and independently
   finish, if Timer1 is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is 
   added to Grbl.
*/
// This interrupt is enabled by ISR_TIMER1_COMPAREA when it sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds) 
// completing one step cycle.
//ISR(TIMER0_OVF_vect)
TIMER0_OverflowInterruptHandler()
{
   //This is necessary on e.g. PIC32, but not on AVR, so on AVR, it will
   //compile to nada. No extra instructions.
   TIMER0_clearOverflowFlag();

  // Reset stepping pins (leave the direction pins)
  // (Nothing is done here if NOT using the  STEP/DIR interface)
  // (in which case: TODO: There's really no reason to have Timer0 running
  //  at all)
  //STEP_PORT = (STEP_PORT & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK); 
  //writePORToutputs_masked(STEP_PORT, STEP_MASK, (step_port_invert_mask & STEP_MASK) );
  STEPPER_ClearStepBits();


  //TCCR0B = 0; // Disable Timer0 to prevent re-entering this interrupt when it's not needed. 
  TIMER0_stopCounting();
  //TODO: Should we not reset its value to zero...?
  //      Or are they counting-up from some value to overflow... OK...
}



#ifdef STEP_PULSE_DELAY
// This interrupt is used only when STEP_PULSE_DELAY is enabled. Here, the step pulse is
// initiated after the STEP_PULSE_DELAY time period has elapsed. The ISR TIMER2_OVF interrupt
// will then trigger after the appropriate settings.pulse_microseconds, as in normal operation.
// The new timing between direction, step pulse, and step complete events are setup in the
// st_wake_up() routine.
//ISR(TIMER0_COMPA_vect) 
TIMER0A_OutputCompareInterruptHandler()
{

   //This is necessary on e.g. PIC32, but not on AVR, so on AVR, it will
   //compile to nada. No extra instructions.
   TIMER0A_clearOutputCompareFlag();

    //meh: So there's no risk of STEP_PORT being used by other things
    // (e.g. the spindle) and having a change between the two interrupts on
    // the other port-pins, and being rewritten here...?
    //NO. Sorry... redefining st.step_bits to buffer st.step_outbits
    // The only reason I can imagine *not* to do the port-read/mask HERE is
    // if there's some risk the extra instructions will be too slow...
    // If that was the case, this entire code-scheme would be entirely
    // different, with notes all the way regarding such presumptions (that
    // the port *won't* change between interrupts).
#ifdef __ORIGINAL_STEP_BITS__
    //STEP_PORT = st.step_bits; // Begin step pulse.
    writeAllPORToutputs_unsafe(STEP_PORT, st.step_bits);
#else
    //Should be: 
    //STEP_PORT = (STEP_PORT & ~STEP_MASK) | st.step_bits;
    writePORToutputs_masked(STEP_PORT, STEP_MASK, st.step_bits);
#endif
}
#endif


// Generates the step and direction port invert masks used in the Stepper Interrupt Driver.
void st_generate_step_dir_invert_masks()
{  
  uint8_t idx;
  step_port_invert_mask = 0;
  dir_port_invert_mask = 0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (bit_istrue(settings.step_invert_mask,bit(idx))) { step_port_invert_mask |= get_step_pin_mask(idx); }
    if (bit_istrue(settings.dir_invert_mask,bit(idx))) { dir_port_invert_mask |= get_direction_pin_mask(idx); }
  }
}


// Reset and clear stepper subsystem variables
void st_reset()
{
  // Initialize stepper driver idle state.
  st_go_idle();
  
  // Initialize stepper algorithm variables.
  memset(&prep, 0, sizeof(st_prep_t));
  memset(&st, 0, sizeof(stepper_t));
  st.exec_segment = NULL;
  pl_block = NULL;  // Planner block pointer used by segment buffer
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;
  
  st_generate_step_dir_invert_masks();
  
#ifndef __IGNORE_MEH_ERRORS__  
#error "Is this ever called while interrupts are running? If so, they need to be disabled around here..."
#endif
  // Initialize step and direction port pins. 
  //STEP_PORT = (STEP_PORT & ~STEP_MASK) | step_port_invert_mask; 
  writePORToutputs_masked(STEP_PORT, STEP_MASK, step_port_invert_mask);
  //DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | dir_port_invert_mask;
  writePORToutputs_masked(DIRECTION_PORT, DIRECTION_MASK, dir_port_invert_mask);
}


// Initialize and start the stepper motor subsystem
void stepper_init()
{
  // Configure step and direction interface pins
  //meh: This is now handled, below...
  //STEP_DDR |= STEP_MASK;
  configurePORToutputs_masked(STEP_PORT, STEP_MASK);

  //meh: ditto
  //STEPPERS_DISABLE_DDR |= 1<<STEPPERS_DISABLE_BIT;
  configurePORToutput(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_BIT);
  
  //meh: ditto
  //DIRECTION_DDR |= DIRECTION_MASK;
  configurePORToutputs_masked(DIRECTION_PORT, DIRECTION_MASK);


 #if (STEPPER_INTERFACE == STEPPER_INTERFACE__PHASE_AB_PWM)
  #ifndef __XC32__ //PIC32...
   #error "This is not particularly architecture-independent! See notes."
  //NOTE...
  // The output registers are set-up (AND CHANGED!) despite the PWM mode.
  // PIC32, this should be OK, LATn outputs don't have different effects
  // when peripherals are on... 
  // OTHER ARCHITECTURES may respond differently!
  #endif
  Stepper_initPWM();
 #endif


  // Configure Timer 1: Stepper Driver Interrupt
/*
  TCCR1B &= ~(1<<WGM13); // waveform generation = 0100 = CTC
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~((1<<WGM11) | (1<<WGM10)); 
  TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0)); // Disconnect OC1 output
  // TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // Set in st_go_idle().
  // TIMSK1 &= ~(1<<OCIE1A);  // Set in st_go_idle().
*/
  TIMER1_initClearTimerOnCompare();

  // Configure Timer 0: Stepper Port Reset Interrupt
/*
  TIMSK0 &= ~((1<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0)); // Disconnect OC0 outputs and OVF interrupt.
  TCCR0A = 0; // Normal operation
  TCCR0B = 0; // Disable Timer0 until needed
  TIMSK0 |= (1<<TOIE0); // Enable Timer0 overflow interrupt
*/
  TIMER0_initNormalCountingAndOverflowInterrupt();
  #ifdef STEP_PULSE_DELAY
    //TIMSK0 |= (1<<OCIE0A); // Enable Timer0 Compare Match A interrupt
    TIMER0A_enableCompareMatchInterrupt();
  #endif
}
  

// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters()
{ 
  if (pl_block != NULL) { // Ignore if at start of a new block.
    prep.flag_partial_block = true;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed; // Update entry speed.
    pl_block = NULL; // Flag st_prep_segment() to load new velocity profile.
  }
}


/* Prepares step segment buffer. Continuously called from main program. 

   The segment buffer is an intermediary buffer interface between the execution of steps
   by the stepper algorithm and the velocity profiles generated by the planner. The stepper
   algorithm only executes steps within the segment buffer and is filled by the main program
   when steps are "checked-out" from the first block in the planner buffer. This keeps the
   step execution and planning optimization processes atomic and protected from each other.
   The number of steps "checked-out" from the planner buffer and the number of segments in
   the segment buffer is sized and computed such that no operation in the main program takes
   longer than the time it takes the stepper algorithm to empty it before refilling it. 
   Currently, the segment buffer conservatively holds roughly up to 40-50 msec of steps.
   NOTE: Computation units are in steps, millimeters, and minutes.
*/
void st_prep_buffer()
{

  if (sys.state & (STATE_HOLD|STATE_MOTION_CANCEL|STATE_SAFETY_DOOR)) { 
    // Check if we still need to generate more segments for a motion suspend.
    if (prep.current_speed == 0.0) { return; } // Nothing to do. Bail.
  }
  
  while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer.

    // Determine if we need to load a new planner block or if the block has been replanned. 
    if (pl_block == NULL) {
      pl_block = plan_get_current_block(); // Query planner for a queued block
      if (pl_block == NULL) { return; } // No planner blocks. Exit.
                      
      // Check if the segment buffer completed the last planner block. If so, load the Bresenham
      // data for the block. If not, we are still mid-block and the velocity profile was updated. 
      if (prep.flag_partial_block) {
        prep.flag_partial_block = false; // Reset flag
      } else {
        // Increment stepper common data index to store new planner block data. 
        if ( ++prep.st_block_index == (SEGMENT_BUFFER_SIZE-1) ) { prep.st_block_index = 0; }
        
        // Prepare and copy Bresenham algorithm segment data from the new planner block, so that
        // when the segment buffer completes the planner block, it may be discarded when the 
        // segment buffer finishes the prepped block, but the stepper ISR is still executing it. 
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;
        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          st_prep_block->steps[X_AXIS] = pl_block->steps[X_AXIS];
          st_prep_block->steps[Y_AXIS] = pl_block->steps[Y_AXIS];
          st_prep_block->steps[Z_AXIS] = pl_block->steps[Z_AXIS];
          st_prep_block->step_event_count = pl_block->step_event_count;
        #else
          // With AMASS enabled, simply bit-shift multiply all Bresenham data by the max AMASS 
          // level, such that we never divide beyond the original data anywhere in the algorithm.
          // If the original data is divided, we can lose a step from integer roundoff.
          st_prep_block->steps[X_AXIS] = pl_block->steps[X_AXIS] << MAX_AMASS_LEVEL;
          st_prep_block->steps[Y_AXIS] = pl_block->steps[Y_AXIS] << MAX_AMASS_LEVEL;
          st_prep_block->steps[Z_AXIS] = pl_block->steps[Z_AXIS] << MAX_AMASS_LEVEL;
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif
        
        // Initialize segment buffer data for generating the segments.
        prep.steps_remaining = pl_block->step_event_count;
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;
        
        prep.dt_remainder = 0.0; // Reset for new planner block

        if (sys.state & (STATE_HOLD|STATE_MOTION_CANCEL|STATE_SAFETY_DOOR)) {
          // Override planner block entry speed and enforce deceleration during feed hold.
          prep.current_speed = prep.exit_speed; 
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed; 
        }
        else { prep.current_speed = sqrt(pl_block->entry_speed_sqr); }
      }
     
      /* --------------------------------------------------------------------------------- 
         Compute the velocity profile of a new planner block based on its entry and exit
         speeds, or recompute the profile of a partially-completed planner block if the 
         planner has updated it. For a commanded forced-deceleration, such as from a feed 
         hold, override the planner velocities and decelerate to the target exit speed.
      */
      prep.mm_complete = 0.0; // Default velocity profile complete at 0.0mm from end of block.
      float inv_2_accel = 0.5/pl_block->acceleration;
      if (sys.state & (STATE_HOLD|STATE_MOTION_CANCEL|STATE_SAFETY_DOOR)) { // [Forced Deceleration to Zero Velocity]
        // Compute velocity profile parameters for a feed hold in-progress. This profile overrides
        // the planner block profile, enforcing a deceleration to zero speed.
        prep.ramp_type = RAMP_DECEL;
        // Compute decelerate distance relative to end of block.
        float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
        if (decel_dist < 0.0) {
          // Deceleration through entire planner block. End of feed hold is not in this block.
          prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
        } else {
          prep.mm_complete = decel_dist; // End of feed hold.
          prep.exit_speed = 0.0;
        }
      } else { // [Normal Operation]
        // Compute or recompute velocity profile parameters of the prepped planner block.
        prep.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp.
        prep.accelerate_until = pl_block->millimeters; 
        prep.exit_speed = plan_get_exec_block_exit_speed();   
        float exit_speed_sqr = prep.exit_speed*prep.exit_speed;
        float intersect_distance =
                0.5*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));
        if (intersect_distance > 0.0) {
          if (intersect_distance < pl_block->millimeters) { // Either trapezoid or triangle types
            // NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0.
            prep.decelerate_after = inv_2_accel*(pl_block->nominal_speed_sqr-exit_speed_sqr);
            if (prep.decelerate_after < intersect_distance) { // Trapezoid type
              prep.maximum_speed = sqrt(pl_block->nominal_speed_sqr);
              if (pl_block->entry_speed_sqr == pl_block->nominal_speed_sqr) { 
                // Cruise-deceleration or cruise-only type.
                prep.ramp_type = RAMP_CRUISE;
              } else {
                // Full-trapezoid or acceleration-cruise types
                prep.accelerate_until -= inv_2_accel*(pl_block->nominal_speed_sqr-pl_block->entry_speed_sqr); 
              }
            } else { // Triangle type
              prep.accelerate_until = intersect_distance;
              prep.decelerate_after = intersect_distance;
              prep.maximum_speed = sqrt(2.0*pl_block->acceleration*intersect_distance+exit_speed_sqr);
            }          
          } else { // Deceleration-only type
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            prep.maximum_speed = prep.current_speed;
          }
        } else { // Acceleration-only type
          prep.accelerate_until = 0.0;
          // prep.decelerate_after = 0.0;
          prep.maximum_speed = prep.exit_speed;
        }
      }  
    }

    // Initialize new segment
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // Set new segment to point to the current segment data block.
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        Compute the average velocity of this new segment by determining the total distance
      traveled over the segment time DT_SEGMENT. The following code first attempts to create 
      a full segment based on the current ramp conditions. If the segment time is incomplete 
      when terminating at a ramp state change, the code will continue to loop through the
      progressing ramp states to fill the remaining segment execution time. However, if 
      an incomplete segment terminates at the end of the velocity profile, the segment is 
      considered completed despite having a truncated execution time less than DT_SEGMENT.
        The velocity profile is always assumed to progress through the ramp sequence:
      acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
      may range from zero to the length of the block. Velocity profiles can end either at 
      the end of planner block (typical) or mid-block at the end of a forced deceleration, 
      such as from a feed hold.
    */
    float dt_max = DT_SEGMENT; // Maximum segment time
    float dt = 0.0; // Initialize segment time
    float time_var = dt_max; // Time worker variable
    float mm_var; // mm-Distance worker variable
    float speed_var; // Speed worker variable   
    float mm_remaining = pl_block->millimeters; // New segment distance from end of block.
    float minimum_mm = mm_remaining-prep.req_mm_increment; // Guarantee at least one step.
    if (minimum_mm < 0.0) { minimum_mm = 0.0; }

    do {
      switch (prep.ramp_type) {
        case RAMP_ACCEL: 
          // NOTE: Acceleration ramp only computes during first do-while loop.
          speed_var = pl_block->acceleration*time_var;
          mm_remaining -= time_var*(prep.current_speed + 0.5*speed_var);
          if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
            // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // Acceleration only. 
            prep.current_speed += speed_var;
          }
          break;
        case RAMP_CRUISE: 
          // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
          // NOTE: If maximum_speed*time_var value is too low, round-off can cause mm_var to not change. To 
          //   prevent this, simply enforce a minimum speed threshold in the planner.
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) { // End of cruise. 
            // Cruise-deceleration junction or end of block.
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
            prep.ramp_type = RAMP_DECEL;
          } else { // Cruising only.         
            mm_remaining = mm_var; 
          } 
          break;
        default: // case RAMP_DECEL:
          // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
          speed_var = pl_block->acceleration*time_var; // Used as delta speed (mm/min)
          if (prep.current_speed > speed_var) { // Check if at or below zero speed.
            // Compute distance from end of segment to end of block.
            mm_var = mm_remaining - time_var*(prep.current_speed - 0.5*speed_var); // (mm)
            if (mm_var > prep.mm_complete) { // Deceleration only.
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; // Segment complete. Exit switch-case statement. Continue do-while loop.
            }
          } // End of block or end of forced-deceleration.
          time_var = 2.0*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete; 
      }
      dt += time_var; // Add computed ramp time to total segment time.
      if (dt < dt_max) { time_var = dt_max - dt; } // **Incomplete** At ramp junction.
      else {
        if (mm_remaining > minimum_mm) { // Check for very slow segments with zero steps.
          // Increase segment time to ensure at least one step in segment. Override and loop
          // through distance calculations until minimum_mm or mm_complete.
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else { 
          break; // **Complete** Exit loop. Segment execution time maxed.
        }
      }
    } while (mm_remaining > prep.mm_complete); // **Complete** Exit loop. Profile complete.

   
    /* -----------------------------------------------------------------------------------
       Compute segment step rate, steps to execute, and apply necessary rate corrections.
       NOTE: Steps are computed by direct scalar conversion of the millimeter distance 
       remaining in the block, rather than incrementally tallying the steps executed per
       segment. This helps in removing floating point round-off issues of several additions. 
       However, since floats have only 7.2 significant digits, long moves with extremely 
       high step counts can exceed the precision of floats, which can lead to lost steps.
       Fortunately, this scenario is highly unlikely and unrealistic in CNC machines
       supported by Grbl (i.e. exceeding 10 meters axis travel at 200 step/mm).
    */
    float steps_remaining = prep.step_per_mm*mm_remaining; // Convert mm_remaining to steps
    float n_steps_remaining = ceil(steps_remaining); // Round-up current steps remaining
    float last_n_steps_remaining = ceil(prep.steps_remaining); // Round-up last steps remaining
    prep_segment->n_step = last_n_steps_remaining-n_steps_remaining; // Compute number of steps to execute.
    
    // Bail if we are at the end of a feed hold and don't have a step to execute.
    if (prep_segment->n_step == 0) {
      if (sys.state & (STATE_HOLD|STATE_MOTION_CANCEL|STATE_SAFETY_DOOR)) {
        // Less than one step to decelerate to zero speed, but already very close. AMASS 
        // requires full steps to execute. So, just bail.
        prep.current_speed = 0.0; // NOTE: (=0.0) Used to indicate completed segment calcs for hold.
        prep.dt_remainder = 0.0;
        prep.steps_remaining = n_steps_remaining;
        pl_block->millimeters = prep.steps_remaining/prep.step_per_mm; // Update with full steps.
        plan_cycle_reinitialize();         
        return; // Segment not generated, but current step data still retained.
      }
    }

    // Compute segment step rate. Since steps are integers and mm distances traveled are not,
    // the end of every segment can have a partial step of varying magnitudes that are not 
    // executed, because the stepper ISR requires whole steps due to the AMASS algorithm. To
    // compensate, we track the time to execute the previous segment's partial step and simply
    // apply it with the partial step distance to the current segment, so that it minutely
    // adjusts the whole segment rate to keep step output exact. These rate adjustments are 
    // typically very small and do not adversely effect performance, but ensures that Grbl
    // outputs the exact acceleration and velocity profiles as computed by the planner.
    dt += prep.dt_remainder; // Apply previous segment partial step execute time
    float inv_rate = dt/(last_n_steps_remaining - steps_remaining); // Compute adjusted step rate inverse
    prep.dt_remainder = (n_steps_remaining - steps_remaining)*inv_rate; // Update segment partial step time

    // Compute CPU cycles per step for the prepped segment.
    // NOTE: This is no longer CPU-cycles!
    //  E.G. PIC32 emulates AVR's timers by multiplying values by 3
    //       (Thankfully, it's a 32-bit timer!)
    //       So, cycles is essentially the number of periods of an
    //       imaginary/simulated 16MHz clock
    //  Except, of course, an AVR might be running at, say, 20MHz
    //       and then this stuff would still work out, but the PIC32 stuff
    //       wouldn't.
    // BIG WARNING: This overflows if F_CPU is too high! And that's
    // *before* the multiplication by inv_rate... So with inv_rate=1 this
    // is *right near* the limit of a uint32_t!
    uint32_t cycles = ceil( (TIMER_CLOCK_TICKS_PER_MICROSECOND*1000000*60)*inv_rate ); 
                     // (cycles/step)    

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING        
      // Compute step timing and multi-axis smoothing level.
      // NOTE: AMASS overdrives the timer with each level, so only one prescalar is required.
      if (cycles < AMASS_LEVEL1) { prep_segment->amass_level = 0; }
      else {
        if (cycles < AMASS_LEVEL2) { prep_segment->amass_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->amass_level = 2; }
        else { prep_segment->amass_level = 3; }    
        cycles >>= prep_segment->amass_level; 
        prep_segment->n_step <<= prep_segment->amass_level;
      }
      if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; } // < 65536 (4.1ms @ 16MHz)
      else { prep_segment->cycles_per_tick = 0xffff; } // Just set the slowest speed possible.
    #else 
      // Compute step timing and timer prescalar for normal step generation.
      if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
        prep_segment->prescaler = 1; // prescaler: 0
        prep_segment->cycles_per_tick = cycles;
      } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prep_segment->prescaler = 2; // prescaler: 8
        prep_segment->cycles_per_tick = cycles >> 3;
      } else { 
        prep_segment->prescaler = 3; // prescaler: 64
        if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
          prep_segment->cycles_per_tick =  cycles >> 6;
        } else { // Just set the slowest speed possible. (Around 4 step/sec.)
          prep_segment->cycles_per_tick = 0xffff;
        }
      }
    #endif

    // Segment complete! Increment segment buffer indices.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

    // Setup initial conditions for next segment.
    if (mm_remaining > prep.mm_complete) { 
      // Normal operation. Block incomplete. Distance remaining in block to be executed.
      pl_block->millimeters = mm_remaining;      
      prep.steps_remaining = steps_remaining;  
    } else { 
      // End of planner block or forced-termination. No more distance to be executed.
      if (mm_remaining > 0.0) { // At end of forced-termination.
        // Reset prep parameters for resuming and then bail. Allow the stepper ISR to complete
        // the segment queue, where realtime protocol will set new state upon receiving the 
        // cycle stop flag from the ISR. Prep_segment is blocked until then.
        prep.current_speed = 0.0; // NOTE: (=0.0) Used to indicate completed segment calcs for hold.
        prep.dt_remainder = 0.0;
        prep.steps_remaining = ceil(steps_remaining);
        pl_block->millimeters = prep.steps_remaining/prep.step_per_mm; // Update with full steps.
        plan_cycle_reinitialize(); 
        return; // Bail!
      } else { // End of planner block
        // The planner block is complete. All steps are set to be executed in the segment buffer.
        pl_block = NULL; // Set pointer to indicate check and load next planner block.
        plan_discard_current_block();
      }
    }

  } 
}      


// Called by realtime status reporting to fetch the current speed being executed. This value
// however is not exactly the current speed, but the speed computed in the last step segment
// in the segment buffer. It will always be behind by up to the number of segment blocks (-1)
// divided by the ACCELERATION TICKS PER SECOND in seconds. 
#ifdef REPORT_REALTIME_RATE
  float st_get_realtime_rate()
  {
     if (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_HOLD | STATE_MOTION_CANCEL | STATE_SAFETY_DOOR)){
       return prep.current_speed;
     }
    return 0.0f;
  }
#endif



#if (STEPPER_INTERFACE == STEPPER_INTERFACE__PHASE_AB_PWM)

 //16 is REALLY SLOW. (May no longer be true)
 #define MICROSTEPS_PER_STEP  4 //16 //4
 //The typical four phases in a bipolar stepper-motor's advancement cycle
 #define STEPS_PER_CYCLE      4

 //It's gotta fit in stepperPhase which is uint8_t (why?)
 #if( 256 % (MICROSTEPS_PER_STEP * STEPS_PER_CYCLE) != 0)
  #error "MICROSTEPS_PER_STEP * STEPS_PER_CYCLE * N != 256"
 #endif



 void Stepper_setPWMfromPhase(uint8_t axis, uint8_t stepperPhase)
 {
   float phaseA_radians =
      (2.0*M_PI)/(STEPS_PER_CYCLE)/(MICROSTEPS_PER_STEP) *
      (stepperPhase%(STEPS_PER_CYCLE * MICROSTEPS_PER_STEP));
   float phaseB_radians =
      (M_PI/2.0) + phaseA_radians;

   //Mighta used g_round, but it's *plausible* with floating-error
   // that we might get 255.5 -> 256 -> 0... or -.5 -> -1 -> 255... ugly.
   int32_t phaseA_power = g_lround((sin(phaseA_radians)*127.5) + 127.5);
   if(phaseA_power > 255)
      phaseA_power=255;
   if(phaseA_power < 0)
      phaseA_power=0;


   int32_t phaseB_power = g_lround((sin(phaseB_radians)*127.5) + 127.5);
   if(phaseB_power > 255)
      phaseB_power=255;
   if(phaseB_power < 0)
      phaseB_power=0;

   //And people wonder why I avoid floating point...

   //Note that this function should/will never be called with inactive
   //cases, so with these #if tests, we're just saving some program-space
   //and maybe making it a bit more clear
   switch(axis)
   {
     #if(PHASE_AB_PWM__AXES_MASK & (1<<X_AXIS) )
      case X_AXIS:
         StepperX_setPWM(phaseA_power, phaseB_power);
         break;
     #endif

     #if(PHASE_AB_PWM__AXES_MASK & (1<<Y_AXIS) )
      case Y_AXIS:
         StepperY_setPWM(phaseA_power, phaseB_power);
         break;
     #endif

     #if(PHASE_AB_PWM__AXES_MASK & (1<<Z_AXIS) )
      case Z_AXIS:
         StepperZ_setPWM(phaseA_power, phaseB_power);
         break;
     #endif
      default:
         break;
   }
 }

#endif // (STEPPER_INTERFACE == STEPPER_INTERFACE__PHASE_AB_PWM)
