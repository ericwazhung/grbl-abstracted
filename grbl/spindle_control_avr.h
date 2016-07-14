#ifndef __SPINDLE_CONTROL_AVR_H__
#define __SPINDLE_CONTROL_AVR_H__


// This removes OC2A from the port-pin and returns it to normal digital
// functionality, which, I have yet to check, is probably output, low.
#define Spindle_disablePWMandOutputZero() \
           ( TCCRA_REGISTER &= ~(1<<COMB_BIT) )



//Great idea toward helping reduce the hardware-dependence of the code, but
//Kinda not ideal to abstract to this level... e.g. 'TCCRA_REGISTER' when,
//unfortunately, the register-definitions and bit-locations vary depending
//not only on the processor (AVR) used, but *also* on the actual timer#
//used within the same processors...
// (yeahp, I tried once to make a common-timer interface, it was a LOT of
// #ifdefs and a LOT of switch-statements and a LOT of things like this,
// which might as well just be specific to the particular timer type
// used... I have yet to determine whether specific timer *types* are
// identical across all AVRs, though... If so, then something like this
// might be doable.
// (e.g. Timer0 on *most* AVRs I've encountered are 8-bit and have the same
//  TCCRx bit-mappings. But not all AVRs' Timer0 *are* 8-bit. Those that
//  aren't *seem* to match Timer1's register/bit-locations.. but again, not
//  all. So, there may be a "Type A" timer that's 8-bit with a specific
//  register/bit mapping, a "Type B" that is 16-bit, etc... There might,
//  also, be a Type C that's also 16-bit but has different register/bit
//  mappings (e.g. one shocker I've encountered has a MUCH wider-range of
//  prescaler-values than most... I think it was the Tiny861... which of
//  course means the register/bit locations differ.)
//  The "worst-case" for a method like that originally used is where a bit
//  (e.g. a Clock-Select bit) is actually in a different register than
//  normal... while still following the typical naming-scheme.
//  E.G. Most of the time TCCRxB is used for clock-selects, but I'm almost
//  certain I've seen an AVR or two where clock-selects are in TCCRxA. In
//  which case, it would appear logical, there would be no compilation-
//  error, and yet random bits would be mangled.
//  It'd be handy to have that knowledge... (whether there's a handful of
//  standard interfaces for timers, and how to determine what interface is
//  associated with what timer in a programmatic way)
//  for moments like these)
//  In my opinion, it's probably less-misleading to potential porters
//  to use the actual register/bit names at this level, rather than
//  creating macros to point to the actual register/bit names...
// (e.g. ATMEGA328P's implementation, below, vs ATMEGA2560, here)

#ifdef CPU_MAP_ATMEGA2560
 
 //Do we really benefit from having 16bit PWM...? Don't forget that
 //doing-so slows the PWM-frequency by 256 times!
 //An alternative would be to use 8-bit and not have to change
 //Spindle_PWM_t, etc...
 #define Spindle_enablePWM() \
 ({ \
      TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) \
                        | (1<<WAVE0_REGISTER); \
      /* set to 1/8 Prescaler */ \
      TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02 \
                        | (1<<WAVE2_REGISTER) | (1<<WAVE3_REGISTER); \
      /* set the top 16bit value */ \
      OCR4A = 0xFFFF; \
      {}; \
 })

 #define Spindle_PWM_t  uint16_t

#else //This isn't particularly safe with all AVR's... Specifically, this
      // is for the ATMEGA328P
 
 #define Spindle_enablePWM() \
 ({ \
      TCCR2A =  \
      /*TCCRA_REGISTER =*/ \
                       /* Clear OC2A on Compare, set at BOTTOM */ \
                       (1<<COM2A1) /* COMB_BIT */ \
                       /* Assuming WGM22 is 0, power-up-default... */ \
                       /* Fast PWM, OCR2 value buffered until BOTTOM */ \
                       /* TOP = 0xFF (TOV flag set at 0xFF) */ \
                     | (1<<WGM21) /* WAVE1_REGISTER */ \
                     | (1<<WGM20); /* WAVE0_REGISTER */ \
      /* set to 1/8 Prescaler */ \
      TCCR2B =  \
      /* TCCRB_REGISTER = */ \
                        /* Clear Prescaler-Select bits */ \
                        /* (TCCRB_REGISTER & 0b11111000) */ \
                        (TCCR2B & 0b11111000) \
                        /* Leaving WGM22 (bit3) to power-up default 0? */ \
                        /* Set Clock-Select (Prescaler) to clk/8 */ \
                        /* (This starts the timer) */ \
                      | 0x02; \
                        /* If I have my math right, here... that's */ \
                        /* 2MHz/256 = 7.8KHz PWM?! Really? */ \
                        /* And the atmega2560 uses 16bit at clkdiv8...? */\
                        /* That'd be 30Hz PWM! No... */ \
      {}; \
 })

 #define Spindle_PWM_t  uint8_t

#endif

#define Spindle_setOutputCompareValue(val)   ( OCR_REGISTER = (val) )

#endif //__SPINDLE_CONTROL_AVR_H__
