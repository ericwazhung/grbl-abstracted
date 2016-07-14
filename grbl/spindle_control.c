/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon
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

#include CONCAT_HEADER(spindle_control_,__MCU_ARCH__)


void spindle_init()
{    
  // Configure variable spindle PWM and enable pin, if requried. On the Uno, PWM and enable are
  // combined unless configured otherwise.
  #ifdef VARIABLE_SPINDLE
    //SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
    configurePORToutput(SPINDLE_PWM_PORT, SPINDLE_PWM_BIT);
#ifndef __IGNORE_MEH_ERRORS__
    #error "is configuring a PWM pin as a digital-output common across all architectures...? Or at least non-invasive?"

    #error "Herein lie some funky #ifs that might need some architecture-specific-handling"
#endif
    #if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
      //SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
      configurePORToutput(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT);
    #endif     
  // Configure no variable spindle and only enable pin.
  #else  
    //SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    configurePORToutput(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT);
  #endif
  
  #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
    //SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
    configurePORToutput(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT);
  #endif
  spindle_stop();
}


void spindle_stop()
{
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  #ifdef VARIABLE_SPINDLE
    
   //TCCRA_REGISTER &= ~(1<<COMB_BIT); // Disable PWM. Output voltage is zero.
   //"Why not just set the OCR-value to zero?"
   // As an outsider looking in, my guess is (from experience):
   // Some AVRs would still spike for one timer-count from 0x00 -> 0x01
   // It's definitely dependent on the MCU, and usually not-well-documented
   // BUT NOTE: as-originally-implemented, the PWM_BIT is only enabled as
   // an output, but *not* set zero. So, we're relying on the boot-values.
   Spindle_disablePWMandOutputZero();


    #if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        //SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  
        // Set pin to high
        setPORTpin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT);
      #else
        //SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); 
        // Set pin to low
        clrPORTpin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT);
      #endif
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      //SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  
      // Set pin to high
      setPORTpin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT);
    #else
      //SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); 
      // Set pin to low
      clrPORTpin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT);
    #endif
  #endif  
}


void spindle_set_state(uint8_t state, float rpm)
{
  // Halt or set spindle direction and rpm. 
  if (state == SPINDLE_DISABLE) {

    spindle_stop();

  } else {

    #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
      if (state == SPINDLE_ENABLE_CW) {
        //SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
        clearPORTpin(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT);
      } else {
        //SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
        setPORTpin(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT);
      }
    #endif

    #ifdef VARIABLE_SPINDLE
      // TODO: Install the optional capability for frequency-based output for servos.

      Spindle_enablePWM();
      
      //current_pwm may be either 16bit or 8bit depending on the available
      //timer-counter
      Spindle_PWM_t  current_pwm;


      if (rpm <= 0.0) { spindle_stop(); } // RPM should never be negative, but check anyway.
      else {
        #define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
        if ( rpm < SPINDLE_MIN_RPM ) { rpm = 0; } 
        else { 
          rpm -= SPINDLE_MIN_RPM; 
          if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent integer overflow
        }
        current_pwm = floor( rpm*(PWM_MAX_VALUE/SPINDLE_RPM_RANGE) + 0.5);
        #ifdef MINIMUM_SPINDLE_PWM
          if (current_pwm < MINIMUM_SPINDLE_PWM) { current_pwm = MINIMUM_SPINDLE_PWM; }
        #endif
  
        // Set PWM pin output
        Spindle_setOutputCompareValue( current_pwm );


#ifndef __IGNORE_MEH_ERRORS__  
#error "This'd probably be a lot cleaner if we macroized the process specifically for INVERT_SPINDLE_ENABLE_PIN..."
#endif
        // On the Uno, spindle enable and PWM are shared, unless otherwise specified.
        #if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) 
          #ifdef INVERT_SPINDLE_ENABLE_PIN
            //SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
            clearPORTpin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT);
          #else
            //SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
            setPORTpin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT);
          #endif
        #endif
      }
      
    #else
      // NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
      // if the spindle speed value is zero, as its ignored anyhow.      
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        //SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
        clearPORTpin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT);
      #else
        //SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
        setPORTpin(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT);
      #endif
    #endif

  }
}


void spindle_run(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.  
  spindle_set_state(state, rpm);
}
