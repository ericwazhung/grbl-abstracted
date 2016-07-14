//There are a few avr-specific operations 'round here, that are somewhat
//difficult to mimic on other architectures.
//These are methods to achieve the same functionality, but plausibly at
//great-cost...


//pgm_read_byte[_near] reads a byte from program-memory (rather than RAM)
//In architectures with large amounts of RAM, it's easy-enough (though
//memory-hungry) to access those data-blocks directly from RAM
// "easy-enough" means... *using normal means* (as opposed to AVR's special
// pgm_read... functions.)
// But, of course, going the *other* direction takes a bit of work...
#define pgm_read_byte_near(address)    ( *((const char *)(address)) )

#define PSTR(string)                   ( string )


