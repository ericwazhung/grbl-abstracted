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

#ifndef __SERIAL_PIC32_H__
#define __SERIAL_PIC32_H__


//This is declared in serial.h but defined in serial_pic32.c
//void serial_init(void)

// Tx Data Register Empty Interrupt handler
//See note below...
//#define SERIAL_TxRegEmptyInterrupt() ISR(SERIAL_UDRE)

// Enable Data Register Empty Interrupt to make sure tx-streaming is running
#define SERIAL_enableTxRegEmptyInterrupt()   ( IEC1SET = (1<<9) )

// Disable Data Register Empty Interrupt when the serial_tx_buffer is empty
#define SERIAL_disableTxRegEmptyInterrupt()  ( IEC1CLR = (1<<9) )







// Load the Tx register with a new byte and begin transmission of that byte
// pic32-note: Surely this could be handled with nothing more than a
// register-write or two, but this is how I got it workin'
// Because this is called only when the Tx buffer (built into the PIC32) is
// empty, it should never block...
#define SERIAL_loadTxRegAndBeginTransmission(byte)  ( _mon_putc(byte) )


// Clear the TxRegEmptyFlag
// This is unnecessary on AVR's... But I think it's necessary on PIC32
#define SERIAL_clearTxRegEmptyFlag()  ( IFS1CLR = (1<<9) ) 



// Serial Data Received Interrupt Handler
//NOGO, here... see note, below...
#if 0
#define SERIAL_RxDataReceivedInterrupt() \
   void __attribute__ ((interrupt(IPL2SOFT))) \
         __attribute__ ((vector(32))) \
        _uartRxHandler(void)
#endif //0

//PIC32 (unlike the AVR) has a single interrupt-vector for multiple
//interrupt-sources...
// The Rx/Tx interrupts are both at the same vector
// Thus, there needs to be a test for the source, and a function-call to
// handle it appropriately.
// Because SERIAL_RxDataReceivedInterrupt() is merely a macro creating a
// function-name for the associated code in serial.c, we can redirect it
// from an ISR-"function" to a regular-ol' function...
// Then we can call these functions from our interrupt-vector...
#define SERIAL_RxDataReceivedInterrupt() \
   void serial_RxDataReceivedInterrupt(void)

//Similarly for SERIAL_TxRegEmptyInterrupt()
#define SERIAL_TxRegEmptyInterrupt() \
   void serial_TxRegEmptyInterrupt(void)

//HOWEVER: Since they are, now, here, regular 'ol functions, we need their
//declarations available...
SERIAL_RxDataReceivedInterrupt();
SERIAL_TxRegEmptyInterrupt();


// Grab the received-data from the serial Rx register
//  pic32-note: Because this is only called from within the Rx-interrupt
//  there should be zero-risk of _mon_getc(DONT_BLOCK=0) from returning
//  that data was not available.
//  Surely this whole thing could be handled with nothing more than a
//  register-read or two, but this is how I got it workin'
#define SERIAL_readRxReg()    ( _mon_getc(0) )

// Clear the RxDataReceivedFlag
// (This is unnecessary on AVR's... But I don't think so for PIC32)
#define SERIAL_clearRxDataReceivedFlag()  ( IFS1CLR = (1<<8) )


#endif //__SERIAL_AVR_H__
