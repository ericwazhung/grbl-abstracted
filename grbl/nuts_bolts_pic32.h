#ifndef __NUTS_BOLTS_PIC32_H__
#define __NUTS_BOLTS_PIC32_H__


//In order to simulate the AVR (running at 16MHz) there's a hack-job
//multiplying all timer-compare values by 3... (see stepper_pic32.c)
#define TIMER_FAKE_PRESCALER  3

//(NOTE: This assumes a Peripheral-Bus clock of 1:1)
#define TIMER_CLOCK_TICKS_PER_MICROSECOND \
            (CPU_TICKS_PER_MICROSECOND/TIMER_FAKE_PRESCALER)

#endif
