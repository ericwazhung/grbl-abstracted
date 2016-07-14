#  Part of Grbl
#
#  Copyright (c) 2009-2011 Simen Svale Skogsrud
#  Copyright (c) 2012-2015 Sungeun K. Jeon
#
#  Grbl is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  Grbl is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.


#IF THIS IS DEFINED=1, then the new abstracted version of grbl should
#compile to a byte-for-byte identical compilation of the original
#grbl-master
# NOTE that this requires all the options in grbl.h to be enabled!
# Comment this out (don't set it to zero) if you don't want this.
#BYTE_IDENTICAL_TEST=1



#This and other options are now in grbl.h... is that reliable...?
CFLAGS += -D__IGNORE_MEH_ERRORS__
CFLAGS += -D__IGNORE_MEH_TODOS__

#This is only a test (with below) to check for missed 
# uint8_t -> gpioPortWidth_t modifications...
# It's not a *reliable* test, but it is A test...
# Convert gpioPortWidth_t to uint16_t for AVR for comparison purposes...
#CFLAGS += -D__AVR_PORT_WIDTH_16__

#As part of that, check for conversion between e.g. a 16-bit value into an
# 8-bit variable...
#Bad Idea... Too many implicit conversions to clear up
# BUT... It can be diffed... Duh.
#CFLAGS += -Wconversion

#I can't imagine an intended-case where an overflow warning should not
#result in an ERROR.
#This came about because bumping F_CPU to 48MHz caused an overflow in
# stepper.c. That has since been fixed, ish.
# BUT OF COURSE this only shows overflows that are detected during
# compile-time! Just because, e.g., that case no longer results in an
# overflow-error does NOT mean that any value can be multiplied by it
# safely! (is inv_rate always less than 1?)
CFLAGS +=-Werror=overflow



# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for

# MCU_ARCH...... The Microcontroller (MCU) architecture
#                E.G. if using an AVR Atmega328p, this will be 'avr'
#                (no quotes)
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.


#MCU_ARCH   = avr

MCU_ARCH   = pic32


ifeq ($(MCU_ARCH),avr)
DEVICE     ?= atmega328p
CLOCK      = 16000000
# CPU_MAP used to be #defined in config.h
# But that's the *only* architecture-specific bit in there...
# (the rest being, e.g. timings)
# So, let's do it here, instead.
# This is *just* the MCU/Pinout-specific part of the name...
# (And, frankly, it would make more sense for it to correspond directly
# with the corresponding filename: e.g. cpu_map_atmega328p.h -> atmega328p
# Then we could use CONCAT_HEADER()
#
# Note from cpu_map.h:
#
# These *could* be handled via CONCAT_HEADER
# But, one thing about the CPU_MAP is that there *might* be multiple
# pinouts for the same CPU... This, too, might be a worthy consideration
# in the other cpu-specific files... e.g. serial_pic32.c/h is, currently,
# specific to UART1, on specific pins... So, yahknow, it depends on what
# level of abstract we want to go to.
# Also, it gets a bit difficult with the cpu (vs architecture)
# specific-ness... e.g. the MCU argument for xc32-gcc uses "32MX170F256B"
# (all caps, lacking the "PIC" prefix) whereas avr-gcc uses "atmega328p"
# (all lower-case, prefixed with the architecture-specific "atmega")
# Of Course, there's ways, but I'm not there yet, with this project.
#
# TODO: Grab notes from config.h
CPU_MAP    = ATMEGA328P
PROGRAMMER ?= -c avrisp2 -P usb
else
ifeq ($(MCU_ARCH),pic32)
DEVICE     ?= 32MX170F256B
CLOCK      = 48000000
CPU_MAP	  = PIC32MX170F256B
else
#Put your MCU-specific stuff here
endif
endif



SOURCE    = main.c motion_control.c gcode.c spindle_control.c \
				coolant_control.c serial.c protocol.c stepper.c \
				eeprom.c settings.c planner.c nuts_bolts.c limits.c \
            print.c probe.c report.c system.c



ifeq ($(BYTE_IDENTICAL_TEST),1)
CFLAGS    += -D__BYTE_IDENTICAL_TEST__
else
#It's a bit ridiculous to use MCU_ARCH, here, since the BYTE_IDENTICAL_TEST
# is basically only for comparing with the original AVR version
SOURCE    += serial_$(MCU_ARCH).c
SOURCE    += stepper_$(MCU_ARCH).c
SOURCE    += spindle_control_$(MCU_ARCH).c
SOURCE    += system_$(MCU_ARCH).c
SOURCE    += eeprom_$(MCU_ARCH).c
SOURCE	 += nuts_bolts_$(MCU_ARCH).c
endif




BUILDDIR = build
SOURCEDIR = grbl
# FUSES      = -U hfuse:w:0xd9:m -U lfuse:w:0x24:m
FUSES      = -U hfuse:w:0xd2:m -U lfuse:w:0xff:m

# Tune the lines below only if you know what you are doing:

ifeq ($(MCU_ARCH),pic32)
TOOLS_PRE = xc32
#Assuming you've only got the "free" xc32-gcc, then we can only use -O1
CFLAGS += -mprocessor=$(DEVICE) -O1
else
ifeq ($(MCU_ARCH),avr)
TOOLS_PRE = avr
CFLAGS += -mmcu=$(DEVICE) -Os
else
TOOLS_PRE = $(MCU_ARCH)
endif
endif



# This makes __MCU_ARCH__ available as a macro
CFLAGS += -D__MCU_ARCH__=$(MCU_ARCH)
CFLAGS += -DCPU_MAP_$(CPU_MAP)
AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE) -B 10 -F
COMPILE = $(TOOLS_PRE)-gcc $(CFLAGS) -Wall -DF_CPU=$(CLOCK) -I. -ffunction-sections -fdata-sections

OBJECTS = $(addprefix $(BUILDDIR)/,$(notdir $(SOURCE:.c=.o)))

# symbolic targets:
all:	grbl.hex

.Phony: lss
lss:
	$(TOOLS_PRE)-objdump --disassemble-zeroes -h -S $(BUILDDIR)/main.elf > main.lss

# I'm using my own tools for testing Byte-identical
.Phony: lssDiff
lssDiff:
	rm -f main.lss vsM.lss.diff && make clean && make && make lss && lssDiff.sh main.lss ../grbl-master/build/main.lss >vsM.lss.diff && view vsM.lss.diff	


$(BUILDDIR)/%.o: $(SOURCEDIR)/%.c
	$(COMPILE) -MMD -MP -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $(BUILDDIR)/$@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

#.c.s:
	$(COMPILE) -S $< -o $(BUILDDIR)/$@

flash:	all
	$(AVRDUDE) -U flash:w:grbl.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID grbl.hex

clean:
	rm -f grbl.hex $(BUILDDIR)/*.o $(BUILDDIR)/*.d $(BUILDDIR)/*.elf \
   $(BUILDDIR)/main.hex
#main.hex is for PIC32, currently...

# file targets:
$(BUILDDIR)/main.elf: $(OBJECTS)
	$(COMPILE) -o $(BUILDDIR)/main.elf $(OBJECTS) -lm -Wl,--gc-sections


ifeq ($(MCU_ARCH),avr)
grbl.hex: $(BUILDDIR)/main.elf
	rm -f grbl.hex
	$(TOOLS_PRE)-objcopy -j .text -j .data -O ihex $(BUILDDIR)/main.elf grbl.hex
	$(TOOLS_PRE)-size --format=berkeley $(BUILDDIR)/main.elf
else
ifeq ($(MCU_ARCH),pic32)

grbl.hex: $(BUILDDIR)/main.elf
	rm -f grbl.hex
	xc32-bin2hex $(BUILDDIR)/main.elf
	mv $(BUILDDIR)/main.hex grbl.hex
	$(TOOLS_PRE)-size --format=berkeley $(BUILDDIR)/main.elf

run:
	~/_commonCode/_make/rC3-scripts/pic32_flash3.sh \
	grbl.hex 32MX170F256B

.PHONY: reset
reset:
	~/_commonCode/_make/rC3-scripts/pic32_reset3.sh 32MX170F256B

endif
endif
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	$(TOOLS_PRE)-objdump -d $(BUILDDIR)/main.elf

cpp:
	$(COMPILE) -E $(SOURCEDIR)/main.c

# include generated header dependencies
-include $(BUILDDIR)/$(OBJECTS:.o=.d)
