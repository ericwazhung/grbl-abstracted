// This file made courtesy Esot.Eric
//  and is a piece from his _commonCode

// This file handles the ability to concatenate filenames for #includes
// such that, e.g. 
//  #include CONCAT_HEADER(serial_,__MCU_ARCH__)
//   would essentially be the same as:
//  #include "serial_avr.h"

//QUOTE_THIS( x ) -> "x"
#define QUOTE_THIS( x) #x

//QUOTE_THIS_VALUE( x ) -> "x_value"
// (when #define x x_value)
#define QUOTE_THIS_VALUE(x) QUOTE_THIS(x)

//add ".h" to the end of the tokens... (without quotes)
#define APPEND_DOT_H(a) a.h

#define APPEND_DOT_C(a) a.c

//GLUE_THESE(a,b) -> ab
#define GLUE_THESE( a, b) a##b
//GLUE_THESE_VALUES(a,b) ->a_valb_val
// (when #define a a_val #define b b_val)
#define GLUE_THESE_VALUES( a, b) GLUE_THESE( a, b)

//Take two pieces of a header-file's name and create a string for #include
// e.g.:
// #define THING  thing_
// #define MCU    attiny861
// #define _THING_HEADER_  CONCAT_HEADER(THING,MCU)
// #include _THING_HEADER_
//   is the equivalent of:
// #include "thing_attiny861.h"
#define CONCAT_HEADER(a,b) \
   QUOTE_THIS_VALUE( \
         APPEND_DOT_H( \
            GLUE_THESE_VALUES(a,b) \
         ) \
   )

#define CONCAT_CFILE(a,b)  \
   QUOTE_THIS_VALUE( \
         APPEND_DOT_C( \
            GLUE_THESE_VALUES(a,b) \
         ) \
   )
