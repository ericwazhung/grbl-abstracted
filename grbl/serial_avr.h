/*
  serial_avr.c - Low level functions for sending and recieving bytes via the serial port
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

#ifndef __SERIAL_AVR_H__
#define __SERIAL_AVR_H__


//This is declared in serial.h but defined in serial_avr.c
//void serial_init(void)

// Tx Data Register Empty Interrupt handler
#define SERIAL_TxRegEmptyInterrupt() ISR(SERIAL_UDRE)

// Enable Data Register Empty Interrupt to make sure tx-streaming is running
#define SERIAL_enableTxRegEmptyInterrupt()   ( UCSR0B |=  (1 << UDRIE0) )

// Disable Data Register Empty Interrupt when the serial_tx_buffer is empty
#define SERIAL_disableTxRegEmptyInterrupt()  ( UCSR0B &=  ~(1 << UDRIE0) )

// Load the Tx register with a new byte and begin transmission of that byte
#define SERIAL_loadTxRegAndBeginTransmission(byte)  ( UDR0 = (byte) )

// Clear the TxRegEmptyFlag
// This is unnecessary on AVR's...
#define SERIAL_clearTxRegEmptyFlag()  {}


// Serial Data Received Interrupt Handler
#define SERIAL_RxDataReceivedInterrupt() ISR(SERIAL_RX)

// Grab the received-data from the serial Rx register
#define SERIAL_readRxReg()    ( UDR0 )

// Clear the RxDataReceivedFlag
// This is unnecessary on AVR's...
#define SERIAL_clearRxDataReceivedFlag()  {}


#endif //__SERIAL_AVR_H__
