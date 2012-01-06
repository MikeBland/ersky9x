#
#       !!!! Do NOT edit this makefile with an editor which replace tabs by spaces !!!!    
#
##############################################################################################
# 
# On command line:
#
# make all = Create project
#
# make clean = Clean project files.
#
# To rebuild project do "make clean" and "make all".
#

##############################################################################################
# Start of default section
#

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
BIN  = $(CP) -O ihex 
BINX = $(CP) -O binary 

# Object files directory
OBJDIR = obj

MCU  = cortex-m3

# List all default C defines here, like -D_DEBUG=1
DDEFS =

# List all default ASM defines here, like -D_DEBUG=1
DADEFS = 

# List all default directories to look for include files here
DINCDIR = 

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS = 

#
# End of default section
##############################################################################################

##############################################################################################
# Start of user section
#

# 
# Define project name and Ram/Flash mode here
PROJECT        = ersky9x
RUN_FROM_FLASH = 1

#
# Define linker script file here
#
ifeq ($(RUN_FROM_FLASH), 0)
LDSCRIPT = sam3s2c_ram.ld
FULL_PRJ = $(PROJECT)_ram
else
LDSCRIPT = sam3s2c_flash.ld
FULL_PRJ = $(PROJECT)_rom
endif

# List all user C define here, like -D_DEBUG=1
UDEFS = -Dat91sam3s4

# Define ASM defines here
UADEFS = 

# List C source files here
SRC  = core_cm3.c \
       board_lowlevel.c \
       crt.c \
       vectors_sam3s.c

CPPSRC = drivers.cpp \
         sound.cpp \
         lcd.cpp \
         debug.cpp \
         file.cpp \
         templates.cpp \
         pers.cpp \
         menus.cpp \
         ersky9x.cpp

# List ASM source files here
ASRC =

# List all user directories here
UINCDIR = ./inc
# \
#          ./cmsis/core \
#          ./cmsis/device

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = 

# Define optimisation level here
OPT = -O1

#
# End of user defines
##############################################################################################


INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))

ifeq ($(RUN_FROM_FLASH), 0)
DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0
else
DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=1
endif

ADEFS   = $(DADEFS) $(UADEFS)
OBJS    = $(ASRC:.s=.o) $(SRC:.c=.o) $(CPPSRC:.cpp=.o)
LIBS    = $(DLIBS) $(ULIBS)
MCFLAGS = -mcpu=$(MCU)

ASFLAGS = $(MCFLAGS) -g -gdwarf-2 -Wa,-amhls=$(<:.s=.lst) $(ADEFS)
CPFLAGS = $(MCFLAGS) $(OPT) -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -Wstrict-prototypes -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
LDFLAGS = $(MCFLAGS) -mthumb -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(FULL_PRJ).map,--cref,--no-warn-mismatch $(LIBDIR)
CPPFLAGS = $(MCFLAGS) $(OPT) -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -fverbose-asm -Wa,-ahlms=$(<:.cpp=.lst) $(DEFS)

# Generate dependency information
CPFLAGS += -MD -MP -MF .dep/$(@F).d
CPPFLAGS += -MD -MP -MF .dep/$(@F).d

#
# makefile rules
#

all: $(OBJS) $(FULL_PRJ).elf $(FULL_PRJ).hex $(FULL_PRJ).lss $(FULL_PRJ).bin



%.o : %.cpp
	$(CC) -c $(CPPFLAGS) -fno-exceptions -I . $(INCDIR) $< -o $@

%.o : %.c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

%.o : %.s
	$(AS) -c $(ASFLAGS) $< -o $@

%elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
  
# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	arm-none-eabi-objdump -h -S $< > $@

%hex: %elf
	$(BIN) $< $@

%bin: %elf
	$(BINX) $< $@

clean:
	-rm -f $(OBJS)
	-rm -f $(FULL_PRJ).elf
	-rm -f $(FULL_PRJ).map
	-rm -f $(FULL_PRJ).hex
	-rm -f $(SRC:.c=.c.bak)
	-rm -f $(SRC:.c=.lst)
	-rm -f $(SRC:.c=.lss)
	-rm -f $(ASRC:.s=.s.bak)
	-rm -f $(ASRC:.s=.lst)
	-rm -fR .dep

# 
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***