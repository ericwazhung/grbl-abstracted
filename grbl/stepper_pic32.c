//This is stolen and modified from grblPeripheralTesting-3
//As well as stepper_avr.h

#include "grbl.h"
#include <xc.h>
//#include <sys/attribs.h>   //for __ISR() though it compiled without.
                           //This IS necessary when using __ISR()
                           // but I didn't get it working that way...
                           // right?
#include <inttypes.h>

#ifndef __IGNORE_MEH_TODOS__
#warning "#pragmas should probably be moved to e.g. nuts_bolts_pic32?"
//Use the internal R/C oscillator:
//This should be 48MHz... (FRC+PLL = 4MHz, * MUL_24 / DIV2)
#warning "Using FPBDIV=DIV_1, there're some messages about setting registers when this happens, does xc32-gcc watch that?"
#endif
#pragma config FPLLMUL = MUL_24,\
               FPLLODIV = DIV_2,\
               FWDTEN = OFF
#pragma config POSCMOD = OFF,\
               FNOSC = FRCPLL,\
               FPBDIV = DIV_1

#pragma config OSCIOFNC = OFF
//This needs to be OFF to disable the oscillator output on RA3/OSCO
//#pragma config   OSCIOFNC = ON

//DISABLE the Secondary Oscillator
// (is this really necessary?!)
//This DOES seem to affect input on RA4 (?!)
// But still not fully-high.
// WAS: floating around 1V (ripple from PWM noticeable)
// NOW: pulled-high around 2.3V (?!)
// "When internal pull-ups are enabled on non-5V
//  tolerant pins, the level as measured on the pin and
//  available to external device inputs may not exceed
//  the minimum value of V IH , and therefore qualify as
//  a logic “high”. However, with respect to the PIC32
//  device, as long as V DD >3V and the load does not
//  exceed -50 μA, the internal pull-ups are
//  guaranteed to be recognized as a logic “high”
//  internally to the device." --???
//  PIC32MX170F256B:
//       256K FLASH  64K SRAM
//          Problem Exists on 5V-Tolerant Pins
//          And NOT on NON-5V-Tolerant Pins
//       RA4 IS NOT 5V-Tolerant, Problem Should Not Exist.
//          VIH-Min = 0.65*VDD
//             VDD=3.6V
//             .65*3.6 = 2.34V Which Is Dang-Near What I'm Seeing
//             Sooooo Thereyago.
#pragma config FSOSCEN = OFF


//Make sure we don't have difficulty reconfiguring the PPS for the UART
#pragma config IOL1WAY = OFF // Allow multiple reconfigurations



//per grbl: TIMER1_initClearTimerOnCompare() 
// (We'll use pic32's timer4/5 in 32-bit mode)
//Initialize the timer to clear-on-compare-match mode
//Do not use its output-compare pins
//Don't start counting, yet
//Don't initialize interrupts, yet
void TIMER1_initClearTimerOnCompare(void)
{
   //Much of this is from bipolarator-5 stepper_init...
   // The Period Register (PRn) is essentially exactly what we're looking
   // for as the "Compare" value to "clear" at...
   // TODO: Does the AVR clear *at* the compare-value or After?
   // (note, PRn is the vale *after* which the timer restarts at 0)
   //Start with a sane value... 
   PR4 = 0xffff;
   PR5 = 0x0000;

   //Set up the timer...
   //Timer4 is a Type B timer...
   //In 32-bit mode, this is the only TnCON register that needs to be set
   // as opposed to configuring T5CON, as well...
   T4CON =  (0<<15) //OFF (Not yet!)  // (1<<15) //ON
         |  (0<<13) //Continue while in idle mode...
         |  (0<<7)  //"Gated Time Accumulation is disabled" O...K...?
                    //value ignored due to TCS = 0...
         //Prescaler:
         // The peripheral-bus is currently set to 1x (48MHz)
         // I can't math, right now...
         // So, here's the chart, for later adjustments:
         //bit 6-4 TCKPS<2:0>: Timer Input Clock Prescale Select bits(3)
         //111 = 1:256 prescale value
         //110 = 1:64 prescale value
         //101 = 1:32 prescale value
         //100 = 1:16 prescale value
         //011 = 1:8 prescale value
         //010 = 1:4 prescale value
         //001 = 1:2 prescale value
         //000 = 1:1 prescale value
         |  (0<<6)
         |  (0<<5)
         |  (0<<4)
         |  (1<<3)   //Use timers4 and 5 in 32-bit mode
         |  (0<<1);  //Use the internal peripheral clock (bit: TCS)

   //And this is probably redundant, but set the start-value to 0
   TMR4 = 0;
   TMR5 = 0;


   //Timer5 is used for interrupt-setup...
   //T5 - Timer 5 (I guess that must be a period-match, what else?)
   //    IRQ 24
   //    Vector 20
   //    Flag: IFS0 bit 24
   //    Enable: IEC0 bit 24
   //    Priority: IPC5 bits 4-2 == 4
   //    Sub-priority: IPC5 bits 1-0


   //We should probably *prep* the interrupt...
   //Make sure it's disabled
   IEC0CLR = (1<<24);

   //Set the priority-level to 4
   IPC5 |= (4<<2);

   //Make sure the interrupt-flag is cleared before starting
   IFS0CLR = (1<<24);


   //Default "single-vector-mode" causes all interrupts to vector to 0x00
   //This is probably set-up in other interrupt-initializers, but doesn't
   //hurt, right?
   INTCONSET = (1<<12); //MVEC, right?

   //X=4, Y=5
   //TimerX is the master timer; TimerY is the slave timer
   //TMRx count register is least significant half word of the 32-bit 
   //    timer value
   //TMRy count register is most significant half word of the 32-bit 
   //    timer value
   //PRx period register is least significant half word of the 32-bit 
   //    period value
   //PRy period register is most significant half word of the 32-bit 
   //    period value
   //TimerX control bits (TxCON) configure the operation for the 32-bit 
   //    timer pair
   //TimerY control bits (TyCON) have no effect
   //TimerX interrupt and status bits are ignored
   //TimerY provides the interrupt enable, interrupt flag and interrupt 
   //    priority control bits


}


// Each value loaded to the counters, from the AVR-Counter-Realm, should
// be multiplied by 3
// ALL THREE:   Period-Register       (PR2  == "overflow" value)
//              Timer-Count-Value     (TMR2 == "start" value)
// TMR2[writ] < Compare-Match-Value   (OC2R == "delay" value)

//AVR's Timer0 maxes out at 0xff
#define T0MAX     (0xff)
//This is what gets loaded to PIC32's PR2
#define T2MAX     ((T0MAX) * 0x0003)
//This gets loaded via TIMER0_loadCount()
//#define T0START   (0x00)
//#define T0DELAY   (0x0002 * (T0MAX) / 3)
//#define T0DELAY   (0x0f )
//So, this is to be the delay-time INCLUDING the start-time
// The actual value that would be loaded to the AVR's OCR
// (and modified for PIC32 via TIMER2_setOutputCompareValue(value) )
//#define T0DELAYED (T0START + T0DELAY)


//per grbl: TIMER0_initNormalCountingAndOverflowInterrupt():
// (We'll use pic32's timer2)
//Initialize Timer0 to normal-counting (count to 0xFF, overflow to 0)
//AVR's Timer0 is 8-bit!
//Do not use its output-compare pins
//Don't start counting, yet
//DO INITIALIZE overflow-interrupt
//Don't initialize compare-match interrupt, yet
void TIMER0_initNormalCountingAndOverflowInterrupt(void)
{
   //Essentially, we'll force PIC32's default Clear-On-Compare setup
   // to work like the AVR's overflow
   //AVR's Timer0 is 8-bit, so will overflow at 0xff
   //Except that the PIC32 clock is 3x faster than the AVR...
   PR2 = T2MAX; //0x00ff*3;

      
   //Set up the timer...
   //Timer2 is a Type B timer...
   T2CON =  (0<<15) //OFF (Not yet!)  // (1<<15) //ON
         |  (0<<13) //Continue while in idle mode...
         |  (0<<7)  //"Gated Time Accumulation is disabled" O...K...?
                    //value ignored due to TCS = 0...
         //Prescaler:
         // The peripheral-bus is currently set to 1x (48MHz)
         // I can't math, right now...
         // So, here's the chart, for later adjustments:
         //bit 6-4 TCKPS<2:0>: Timer Input Clock Prescale Select bits(3)
         //111 = 1:256 prescale value
         //110 = 1:64 prescale value
         //101 = 1:32 prescale value
         //100 = 1:16 prescale value
         //011 = 1:8 prescale value
         //010 = 1:4 prescale value
         //001 = 1:2 prescale value
         //000 = 1:1 prescale value
         |  (0<<6)
         |  (0<<5)
         |  (0<<4)
         |  (0<<3)   //Use timers in 16-bit mode
         |  (0<<1);  //Use the internal peripheral clock (bit: TCS)


   //And this is probably redundant, but set the start-value to som'n sane
   //TMR2 = (0x0003 * T0START);
   // (e.g. don't load the timer value with a value larger than the
   //  period-register, or it'll take forever to overflow and wrap around)
   TIMER0_loadCount(0);

   //Set up the overflow interrupt
   //Interrupt configuration for TIMER2 is handled in interrupt registers
   
   //T2 - Timer 2 (I guess that must be a period-match, what else?)
   //    IRQ 9
   //    Vector 8
   //    Flag: IFS0 bit 9
   //    Enable: IEC0 bit 9
   //    Priority: IPC2 bits 4-2 == 3
   //    Sub-priority: IPC2 bits 1-0
   
   IEC0 |= (1<<9);
   IPC2 |= (3 << 2); //set the priority-level to 3 

   //Clear the interrupt flag... 
   IFS0CLR = (1<<9);

#ifdef STEP_PULSE_DELAY
   //Should probably also *prep* the Output-Compare Unit...
   //Make sure the compare-value is reasonable
   //OC2R = 0x007f*3;
   TIMER0A_setOutputCompareValue(0x7f);

//Looks like this is only in PWM mode...
//#warning "Are we going to have issues writing OC2R rather'n OC2RS?"
   // Dun see why it can't be *on* already...
   // In Fact: In bipolarator-5, stepper_init, I did it this way
   //   enabling the timer after the output-compare-unit

   OC2CON =    (1<<15)  //ON
            |  (0<<13)  //Continue when the device is idle
            |  (0<<5)   //Use 16-bit (rather than 32-bit)
            |  (0<<3)   //Use Timer2 (1=Timer3)
            //"Output compare peripheral is disabled but continues to draw
            //current"
            //All other options look to require involving an actual pin
            // Although, I suppose, I could just not PPS that shizzle...?
#warning "This could be a problem... Set a mode!"
//            |  (0<<2)
//            |  (0<<1)
//            |  (0<<0);
            //101 = Initialize OCx pin low; generate continuous output 
            //      pulses on OCx pin... NO. This compares OCRn and OCRnS
            //INSTEAD
            //011 ="Compare toggles OCx pin. Toggle event is continuous and
            //      ## an interrupt is generated for each toggle event. ##"
            |  (0<<2)
            |  (1<<1)
            |  (1<<0);
            // (The other 0xx modes only interrupt *once*, as I understand.
            //  We don't care about the pin-toggling, just the repeated
            //  interrupts, as, otherwise, I think, each time the timer is
            //  enabled, it would *also* have to reset the OC-system, or
            //  something...?)


   //Should probably also *prep* the Compare-Match interrupt
   // "output-compare" despite not intending to wire to an output...
   
   //OC2 - Output Compare 2 (I guess that must be a match, what else?)
   //    IRQ 12
   //    Vector 10
   //    Flag: IFS0 bit 12
   //    Enable: IEC0 bit 12
   //    Priority: IPC2 bits 20-18 == 5
   //    Sub-priority: IPC2 bits 17-16
   
   //Make sure it's disabled, it will be enabled later
   IEC0CLR = (1<<12);
   IPC2 |=   (5<<18);  //set the priority level to 5

   //Clear the interrupt flag...
   IFS0CLR = (1<<12);
#endif //STEP_PULSE_DELAY

   //Default "single-vector-mode" causes all interrupts to vector to 0x00
   //This is probably set-up in other interrupt-initializers, but doesn't
   //hurt, right?
   INTCONSET = (1<<12); //MVEC, right?
  
}


//AVR Timer1 prescaler chart: (for timer45)
// Note: 328P: CS10 is bit 0, so <<CS10 is redundant
//       but maybe for another AVR...?
// The PIC32 doesn't use this same bit-scheme so will 
// require a switch-statement. Fine... But ALSO: the timer doesn't
// run at the same speed!
//bit: 2     1     0
//     CS12  CS11  CS10
//     0     0     0       Counter Stopped  (untested)
//     0     0     1       clk/1      (tested)
//     0     1     0       clk/8      (tested)
//     0     1     1       clk/64     (tested)
//     1     0     0       clk/256    (tested)
//     1     0     1       clk/1024   (tested. Aws!)
//     1     1     X       external clock (should be unused)
//
//NOTE: It appears only clk/1,8,64 are used by stepper.c...(?)

//THREE PROBLEMS:
// They definitely don't align... switch-statement necessary
//  but worse:
// clkdiv1024 is not available
// The peripheral-bus is 48MHz, 3x faster than the AVR's at 16MHz
//  Because no comparisons seem to be made with TCNT1
//  (the only use of the timer is for the interrupt)
//  We should be able to math-it...
//  EXCEPT:
//   The AVR-Timer1 is 16-bit
//   So e.g. multiplying a compare-match value by 3 *might* overflow a
//   16-bit timer... so set these up as 32-bit
//  So, this'll need to be changed if/when clkDiv1024 is requested.
unsigned int timer45_additionalPrescaler = (48/16);

//Alright, now a potential problem:
//  If the compare-match-value is set *before* vs *after* the prescaler!
//  TIMER1_setPrescaler is *only* called immediately before
//  setCompareMatchValue. FHWEW
// Another: MAKE SURE to ONLY ever set the prescaler with this function!
// (dun see how it would slip-by as I've already grepped it to only one
// location, and the grbl code won't compile for PIC32 if referencing AVRs'
// registers... right?)


//NOTE:
// AVR's setting of the prescaler inherently *starts* the timer
// While it doesn't appear that should be a problem, I'll add that to this,
// as well...
void TIMER1_setPrescaler(unsigned int prescalerConfigBits)
{
   unsigned int t4conTemp = T4CON;  

   //Per AVR, start the timer when setting the prescaler
   // (if prescalerConfigBits == 0, it will be redisabled)
   t4conTemp |= (1<<15);

   switch(prescalerConfigBits)
   {
      //Counter Stopped (probably shouldn't be called, right?)
      case 0x0:
         t4conTemp &= ~(1<<15);
         break;
      //<AVR's 16MHz> / 1 (full speed = 16MHz)
      case 0x1:
         //1:1 prescaler, requires additionalPrescaler=3 for 1:3
         t4conTemp &= ~(7<<4);
         timer45_additionalPrescaler = 3;
         break;
      //<AVR's 16MHz> / 8 (2MHz)
      case 0x2:
         //1:8 prescaler, requires additionalPrescaler=3 for 1:24
         t4conTemp &= ~(7<<4);
         t4conTemp |= (3<<4);
         timer45_additionalPrescaler = 3;
         break;
      //<AVR's 16MHz> / 64 (250KHz)
      case 0x3:
         //1:64 prescaler, requires additionalPrescaler=3 for 1:192
         t4conTemp &= ~(7<<4);
         t4conTemp |= (6<<4);
         timer45_additionalPrescaler = 3;
         break;
      //<AVR's 16MHz> / 256 (62.5KHz)
      case 0x4:
         //1:256 prescaler, requires additionalPrescaler=3 for 1:768
         t4conTemp |= (7<<4);
         timer45_additionalPrescaler = 3;
         break;
      //<AVR's 16MHz> / 1024 (15.625KHz)
      case 0x5:
         //1:1024 prescaler
         t4conTemp |= (7<<4); //PIC32's Max Prescaler: 1:256
         //So we need 1024/256 * 3...
         timer45_additionalPrescaler = (1024*3/256);
         break;
      default:
         //WTF?
         break;
   }

   T4CON = t4conTemp;
}

#if (STEPPER_INTERFACE == STEPPER_INTERFACE__PHASE_AB_PWM)

//This note left here only for my reminder...
#ifdef STEP_PULSE_DELAY
#error "STEP_PULSE_DELAY can't be used with PHASE_AB"
//Namely, here, right... because the OCR is used for PWM...?
#endif


//This is near-identical to Spindle_enablePWM()
// SEE THERE FOR LOTS OF NOTES.
// We will share the Timer3 for this, as well...
//
// Things here that are duplicates should be acceptably-duplicated
// (rewriting timer-config-registers with the same values should be OK)
void Stepper_initPWM(void)
{
   ppsUNLOCK();
#if(PHASE_AB_PWM__AXES_MASK & (1<<X_AXIS))
      RPB4R = 0x5; //0101 = OC1 (X_STEP)
      RPB5R = 0x5; //0101 = OC2 (X_DIR)
#endif

#if(PHASE_AB_PWM__AXES_MASK & (1<<Y_AXIS))
      RPB13R = 0x5; //0101 = OC4 (Y_STEP)
      RPB14R = 0x5; //0101 = OC3 (Y_DIR)
#endif
   ppsLOCK();

   //It's wise to set the OCnR value to something sane before starting PWM
   OC1R = 0x7f;
   OC2R = 0x7f;
   OC3R = 0x7f;
   OC4R = 0x7f;

   //From here-on, the compare-register OC5R is read-only...
   // must write to OC5RS, which is buffered until the next timer reset

   //Configure the PWM output:
   OC1CON =    (1<<15)  //ON
            |  (0<<13)  //Continue when the device is idle
            |  (0<<5)   //Use 16-bit (rather than 32-bit)
            |  (1<<3)   //Use Timer3 (0=Timer2)
            // PWM Mode, Fault Pin Disabled:
            |  (1<<2)
            |  (1<<1)
            |  (0<<0);
   asm("nop;");
   OC2CON = OC1CON;
   OC3CON = OC1CON;
   OC4CON = OC1CON;

   //We'll use 8-bit PWM, per Spindle_enablePWM() AND the rest of this code
   PR3 = 0xff;

   //Set up the timer...
   //Timer3 is a Type B timer...
   T3CON =     (1<<15) //ON
            |  (0<<13) //Continue while in idle mode...
            |  (0<<7)  //"Gated Time Accumulation is disabled" O...K...?
                       //value ignored due to TCS = 0...
            //Prescaler:
            // The peripheral-bus is currently set to 1x (48MHz)
            // I can't math, right now...
            // So, here's the chart, for later adjustments:
            //bit 6-4 TCKPS<2:0>: Timer Input Clock Prescale Select bits(3)
            //111 = 1:256 prescale value
            //110 = 1:64 prescale value
            //101 = 1:32 prescale value
            //100 = 1:16 prescale value
            //011 = 1:8 prescale value
            //010 = 1:4 prescale value
            //001 = 1:2 prescale value
            //000 = 1:1 prescale value
            //This was giving about 23KHz PWM
            |  (0<<6)
            |  (1<<5)
            |  (1<<4)

            |  (0<<3)   //Use timer in 16-bit mode
            |  (0<<1);  //Use the internal peripheral clock (bit: TCS)

   Stepper_setPWMfromPhase(X_AXIS, 0);
 //  StepperX_setPWM(0x7f, 0x7f);
   Stepper_setPWMfromPhase(Y_AXIS, 0);

}

#endif



