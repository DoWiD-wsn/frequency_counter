################################################################################
# USAGE:
# make              .... build the program image
# make flash        .... build and flash the application
# make clean        .... remove intermediate and generated files
# make erase        .... erase program on target device
# make factoryreset .... flash original Arduino image (incl. bootloader)


################################################################################
# define the name of the generated output file
#
## Project/Application name ##
TARGET        = frequency_counter
PORT          = /dev/ttyACM0

## Target specifics ##
MCU           = atmega8
ARCH          = AVR8
F_CPU         = 4096000

## Tools
CC            = avr-gcc
OBJCP         = avr-objcopy
OBJDMP        = avr-objdump
SIZE          = avr-size

# Output optimization
OPTIMIZATION  = s


################################################################################
# below only edit with care
#
BUILDDIR      = ./build
SRCDIR        = .
TARGET_ARCH   = -mmcu=$(MCU)
SRC           = $(wildcard $(SRCDIR)/*.c)
VPATH         = $(SRCDIR)
INCS          = -I$(SRCDIR)
#
CPPFLAGS      = $(TARGET_ARCH)
#
CFLAGS        = -I. $(TARGET_ARCH)
CFLAGS       += -Wall -Wextra -Wno-unused-parameter
CFLAGS       += -DF_CPU=$(F_CPU)
CFLAGS       += -O$(OPTIMIZATION)
#
LDFLAGS       = $(TARGET_ARCH)
LDFLAGS      += -lm
LDFLAGS      += -Wl,--gc-sections
LDFLAGS      += -O$(OPTIMIZATION)

# Add all dirs with source files to make's VPATH
$(foreach d,$(call uniq,$(dir $(SRC))),$(eval VPATH+=:$(d)))

## Programmer ##
TARGETPROG    = $(BUILDDIR)/$(TARGET).hex
# AVRDUDE #
DUDE          = avrdude
#DUDE         = /usr/bin/avrdude
DUDE_DEVICE   = $(MCU)
DUDE_PORT     = $(PORT)
DUDE_FLASH    = -U flash:w:
DUDE_FUSES    = -U lfuse:w:0xff:m -U hfuse:w:0xc9:m
DUDE_ERASE    = -e
# AVR ISP programmer
DUDE_ISP      = $(DUDE) -p $(DUDE_DEVICE) -c avrispv2 -P $(DUDE_PORT) -v

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

AFLAGS = -x assembler-with-cpp

COBJS := $(SRC:.c=.o)
AOBJS := $(COBJS:.S=.o)
OBJS  := $(addprefix $(BUILDDIR)/,$(notdir $(AOBJS)))

-include $(OBJS:.o=.d)

#### Rules ####
all: $(OBJS) $(BUILDDIR)/$(TARGET).elf $(BUILDDIR)/$(TARGET).hex size

$(BUILDDIR)/%.o: %.c | $(BUILDDIR)
	$(CC) -c $(CFLAGS) $(INCS) $< -o $@

$(BUILDDIR)/%.o: %.S | $(BUILDDIR)
	$(CC) -c $(CFLAGS) $(INCS) $(AFLAGS) $< -o $@

$(BUILDDIR)/$(TARGET).elf: $(OBJS) | $(BUILDDIR)
	$(CC) $(LDFLAGS) $(CFLAGS) -o $(BUILDDIR)/$(TARGET).elf $(OBJS)

$(BUILDDIR)/$(TARGET).hex: $(BUILDDIR)/$(TARGET).elf
	$(OBJCP) -O ihex $(HEX_FLASH_FLAGS)  $< $@

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

flash: $(BUILDDIR)/$(TARGET).hex
	$(DUDE_ISP) $(DUDE_FLASH)$(TARGETPROG)

setfuses:
	$(DUDE_ISP) $(DUDE_FUSES)

erase:
	$(DUDE_ISP) $(DUDE_ERASE)

size: $(BUILDDIR)/$(TARGET).elf
	$(SIZE) -C --mcu=$(MCU) $(BUILDDIR)/$(TARGET).elf

clean:
	rm -rf $(BUILDDIR)

.DEFAULT_GOAL := all
.PHONY: all flash setfuses erase size clean
.SUFFIXES: .c .o .S .d .h .su
