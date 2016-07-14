/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl


  This file is now for general-purpose serial handling
  e.g. for the buffers, etc. 
  See e.g. serial_avr.c for architecture-specific code

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


#ifndef serial_h
#define serial_h

//Essentially: #include "serial_avr.h" or "serial_pic32.h", as appropriate
#include CONCAT_HEADER(serial_,__MCU_ARCH__)

//WTF... this was at the end of the file...
//Typically it's considered "bad practice" to include actual
//function-definitions in a header-file.
//I think this is a reasonable exception...
//There's only one small function (the rest being macros)
//WTF... this shoulda been handled long ago with CONCAT_HEADER...
//#include "serial_avr.h"



#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 64
#endif

#define SERIAL_NO_DATA 0xff

#ifdef ENABLE_XONXOFF
  #define RX_BUFFER_FULL 96 // XOFF high watermark
  #define RX_BUFFER_LOW 64 // XON low watermark
  #define SEND_XOFF 1
  #define SEND_XON 2
  #define XOFF_SENT 3
  #define XON_SENT 4
  #define XOFF_CHAR 0x13
  #define XON_CHAR 0x11
#endif


// Initialize the serial peripheral
// This is architecture-specific, so its definition is in e.g. serial_avr.h
// Generally:
//   Sets BAUD_RATE
//   Enable BOTH Tx and Rx
//   Enable the Rx interrupt
//   (Use 8-bits, No-parity, 1 stop-bit... this is generally default)
void serial_init();

// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data);

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read();

// Reset and empty data in read buffer. Used by e-stop and reset.
void serial_reset_read_buffer();

// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count();

// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count();




//Internal-ish stuff, these are used in serial.c and e.g. serial_avr.c...

#if 0
// We mightn't need these, here... they may only be necessary in serial.c
// not e.g. serial_avr.c!


//These variables are used regardless of the MCU architecture
// Their definitions are in serial.c
// but they are referenced in multiple locations, including e.g. serial_avr.c
extern uint8_t serial_rx_buffer[];
extern uint8_t serial_rx_buffer_head;
extern volatile uint8_t serial_rx_buffer_tail;

extern uint8_t serial_tx_buffer[];
extern uint8_t serial_tx_buffer_head;
extern volatile uint8_t serial_tx_buffer_tail;


#ifdef ENABLE_XONXOFF
  extern volatile uint8_t flow_ctrl; // Flow control state variable
#endif
#endif //0

//These "functions" are macros, defined in your specific architecture file,
//e.g. serial_avr.h
// Doing it this way assures that no extra instructions are wasted in
// function-calls, pushing/popping registers, etc.
// An alternative might be to use inline functions, but that gets a bit
// difficult, at this low-level, across different implementations of C


// Your architecture-specific header-file (e.g. serial_avr.h)
// should have the following macros...

// Serial Tx Register Empty Interrupt
// (The actual interrupt-handling code is in serial.c)
// e.g.
//  #define SERIAL_TxRegEmptyInterrupt() ISR(SERIAL_UDRE)


// Enable an interrupt for when there's free-space in the Tx register
// e.g.
//  #define SERIAL_enableTxRegEmptyInterrupt()   ( UCSR0B |=  (1 << UDRIE0) )

// And disable that interrupt when there's no data in serial_tx_buffer[]
// e.g. 
//  #define SERIAL_disableTxRegEmptyInterrupt()  ( UCSR0B &=  ~(1 << UDRIE0) )

// Load the Tx register with a new byte 
// (and begin transmission if not already)
// e.g. 
//  #define SERIAL_loadTxRegAndBeginTransmission(byte)  ( UDR0 = (byte) )

// Serial Data Received Interrupt
// e.g.
//  #define SERIAL_RxDataReceivedInterrupt() ISR(SERIAL_RX)

// Grab the received-data from the serial Rx register
// e.g.
//  #define SERIAL_readRxReg()    ( UDR0 )



#endif
