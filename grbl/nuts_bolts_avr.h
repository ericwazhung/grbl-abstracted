#ifndef __NUTS_BOLTS_AVR_H__
#define __NUTS_BOLTS_AVR_H__


//The AVR timers are clocked (before the prescaler) by the CPU clock
//(But this isn't the case with all architectures)
#define TIMER_CLOCK_TICKS_PER_MICROSECOND    (CPU_TICKS_PER_MICROSECOND)

#endif
