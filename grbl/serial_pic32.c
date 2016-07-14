/*
  serial_pic32.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Modified from the original serial.c by Esot.Eric 6-2016

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
//#include "serial.h" //Included in grbl.h...


void serial_init(void)
{
   //This is redundant... interrupts are not enabled until long after
   //serial_init is called, in main...
   //__builtin_disable_interrupts();
   cli();
   U1MODEbits.UARTEN = 0;  //Disable UART1 for reconfiguration


   //Use UART1 for stdio (_mon_putc/getc)
   __XC_UART = 1;

  // Set baud rate
#if 0 //From AVR...
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
#endif //0

#ifndef __IGNORE_MEH_TODOS__
#error "This could stand to be more sophisticated... see the AVR example"
#endif
  U1MODEbits.BRGH = 1; //High Baud Rate Enable
  U1BRG = F_CPU/(4*BAUD_RATE) - 1; //Baud Rate Generator.
   
  //UBRR0H = UBRR0_value >> 8;
  //UBRR0L = UBRR0_value;
            
  // enable rx and tx
  //UCSR0B |= 1<<RXEN0;
  U1STAbits.URXEN = 1;
  //UCSR0B |= 1<<TXEN0;
  U1STAbits.UTXEN = 1;

  // enable the UART peripheral
  U1MODEbits.UARTEN = 1;

  //Configure the Peripheral-Pin-Select system to handle my UART pinout
  // U1TX: RA0
  // U1RX: RA2
  //The PPS system has to be unlocked and relocked...
/*
   SYSKEY = 0x0; //ensure OSCCON is locked
   SYSKEY = 0xAA996655; //Write Key1 to SYSKEY 
   SYSKEY = 0x556699AA; //Write Key2 to SYSKEY
   // OSCCON is now unlocked, and it's time to make the change... 
   CFGCON = CFGCON & ~(1<<13);
*/
  ppsUNLOCK();
   //Set PPS for RPA0 to be the UART1 Tx output.
   RPA0R = 0x1; //RPA0R<3:0> = 0001 = U1TX

   //Set PPS for RPA2 to be the UART1 Rx input.
   // (whoa, digity, prior to v4, I forgot about this, and apparently just
   //  happened to luck-out that it was default.)
   // (Also happened to luck-out that there's no analog circuitry on this
   // pin, otherwise that'd've had to've been disabled)
   U1RXR = 0x0; //U1RXR<3:0> = 0000 = RPA2

  //And relock it...
  // SYSKEY = 0x0;
  ppsLOCK();



  // enable interrupt on complete reception of a byte
  //UCSR0B |= 1<<RXCIE0;
  IEC1 |= (1<<8); //Enable the Rx interrupt
  IPC8 |= (2<<2); //Give it a priority of 2 (this is the same for Tx)

  //The Tx interrupt is enabled on an as-needed basis

  //Clear both interrupt flags before enabling interrupts
  IFS1CLR = (3<<8);

  //Enable multi-vector interrupts
  INTCONSET = (1<<12); //MVEC, right?

  //Interrupts are enabled, globally, after this is called, in main
  // So don't do it here...
  //__builtin_enable_interrupts();

  // defaults to 8-bit, no parity, 1 stop bit
}

//Normally the only function in an architecture-specific serial_xxx.c would
//be serial_init()
//But...
//PIC32 (unlike the AVR) has a single interrupt-vector for multiple
//interrupt-sources...
// The Rx/Tx interrupts are both at the same vector
// Thus, there needs to be a test for the source, and a function-call to
// handle it appropriately.
// Because SERIAL_RxDataReceivedInterrupt() is merely a macro creating a
// function-name for the associated code in serial.c, we can redirect it
// from an ISR-"function" to a regular-ol' function...
// Then we can call these functions from our interrupt-vector...
// It'll be a lot more instructions, but we've got a lot more
// processing-power... right?
// TODO: PIC32 allows(?) interrupts to interrupt interrupts
//       Priorities can be set to allow/disallow as-preferred
//       I don't think AVR allows it, at all, unless explicitly sei()'d
//       during an interrupt... so maybe it makes sense to cli() in this
//       interrupt...
void __attribute__ ((interrupt(IPL2SOFT))) __attribute__ ((vector(32))) 
     _uartInterruptHandler(void)
{
   unsigned int ifsTemp = IFS1;

   //Clearing the flag is handled in these functions

   //Rx Data Received:
   if( ifsTemp & (1<<8) )
   {
      serial_RxDataReceivedInterrupt();
   }
   //Tx Register is empty:
   else if( ifsTemp & (1<<9) )
   {
      serial_TxRegEmptyInterrupt();
   }
}





