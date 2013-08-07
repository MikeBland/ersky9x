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
CLSS = $(TRGT)objdump
AS   = $(TRGT)as
#gcc -x assembler-with-cpp
BIN  = $(CP) -O ihex 
BINX = $(CP) -O binary 

# Object files directory
ifeq ($(PCB), X9D)
OBJDIR = x9dobj
else
OBJDIR = skyobj
endif

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


EXT = STD

ifeq ($(STAMP), 1)
AREV = $(shell sh -c "cat .svn/entries | sed -n '4p'")
REV = $(shell echo $$(( $(AREV) + 1 )))
#REV = $(shell echo $$(( $(AREV) )))
endif

#
# End of default section
##############################################################################################

##############################################################################################
# Start of user section
#

# 
# Define project name and Ram/Flash mode here
ifeq ($(PCB), X9D)
PROJECT        = x9d
else
PROJECT        = ersky9x
endif

RUN_FROM_FLASH = 1

#
# Define linker script file here
#
ifeq ($(RUN_FROM_FLASH), 0)
 ifeq ($(SAM3S2), 1)
 LDSCRIPT = sam3s2c_ram.ld
 else
   ifeq ($(PCB), X9D)
#     ARCH = ARM
     LDSCRIPT = x9d/stm32_flash.ld
#     TRGT = arm-none-eabi-
#     MCU  = cortex-m3
     CPPDEFS += -DHSE_VALUE=12000000
	  CPPDEFS += -DPCBX9D 
   else
     LDSCRIPT = sam3s4c_ram.ld
	  CPPDEFS += -DPCBSKY 
     UDEFS = -Dat91sam3s4
	endif
 endif
 FULL_PRJ = $(PROJECT)_ram
else
 ifeq ($(SAM3S2), 1)
 CPPDEFS += -DPCBSKY 
 LDSCRIPT = sam3s2c_flash.ld
 else
   ifeq ($(PCB), X9D)
#     ARCH = ARM
     LDSCRIPT = x9d/stm32_flash.ld
#     TRGT = arm-none-eabi-
#     MCU  = cortex-m3
     CPPDEFS += -DHSE_VALUE=12000000
	  CPPDEFS += -DPCBX9D 
   else
    CPPDEFS += -DPCBSKY 
    LDSCRIPT = sam3s4c_flash.ld
    UDEFS = -Dat91sam3s4
	endif
 endif
 FULL_PRJ = $(PROJECT)_rom
endif

# List all user C define here, like -D_DEBUG=1

ifeq ($(PCB), X9D)
UDEFS = 
endif

ifeq ($(SAM3S2), 1)
UDEFS = -Dat91sam3s2
endif

ifeq ($(LANG), FR)
 CPPDEFS += -DLANG_FR
else
 ifeq ($(LANG), DE)
  CPPDEFS += -DLANG_DE
 endif
endif

# Define ASM defines here
UADEFS = 

ifeq ($(PCB), X9D)

# List C source files here
SRC  = x9d/system_stm32f2xx.c \
       x9d/stm32f2xx_gpio.c \
       x9d/stm32f2xx_spi.c \
       x9d/stm32f2xx_rtc.c \
       x9d/stm32f2xx_pwr.c \
       coos.c \
       x9d/stm32f2xx_rcc.c


CPPSRC = x9d.cpp \
         x9d/lcd_driver.cpp \
         x9d/aspi.cpp \
         x9d/diskio.cpp \
         drivers.cpp \
         x9d/sound.cpp \
         x9d/power.cpp \
         x9d/watchdog.cpp \
       	x9d/i2c_ee.cpp \
         x9d/rtc_driver.cpp \
         x9d/eeprom_rlc.cpp \
         timers.cpp \
         logicio.cpp \
         analog.cpp \
         debug.cpp \
		   lcd.cpp

#         audio.cpp \
#         ff.cpp \

# List ASM source files here
ASRC = x9d/startup_stm32f2xx.s

else

# List C source files here
SRC  = core_cm3.c \
       board_lowlevel.c \
       crt.c \
       vectors_sam3s.c \
       coos.c \
       usb/device/core/USBD_UDP.c usb/device/core/USBDDriver.c usb/device/core/USBDCallbacks.c \
       usb/device/massstorage/MSDDriver.c usb/device/massstorage/MSDDStateMachine.c usb/device/massstorage/MSDLun.c usb/device/massstorage/MSDDriverDescriptors.c usb/device/massstorage/SBCMethods.c \
       usb/common/core/USBEndpointDescriptor.c usb/common/core/USBGenericRequest.c usb/common/core/USBFeatureRequest.c usb/common/core/USBInterfaceRequest.c usb/common/core/USBGetDescriptorRequest.c \
       usb/common/core/USBSetAddressRequest.c usb/common/core/USBSetConfigurationRequest.c \
       usb/common/core/USBConfigurationDescriptor.c usb/common/core/USBGenericDescriptor.c \
       MEDSdcard.c

CPPSRC = stamp.cpp \
         drivers.cpp \
         diskio.cpp \
         sdcard_driver.cpp \
         ff.cpp \
         gtime.cpp \
         sound.cpp \
         lcd.cpp \
         debug.cpp \
         file.cpp \
         templates.cpp \
         pers.cpp \
         menus.cpp \
         frsky.cpp \
         audio.cpp \
         ersky9x.cpp \
         timers.cpp \
         logicio.cpp \
         pulses.cpp \
         massstorage.cpp

# List ASM source files here
ASRC =

endif


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
OPT = -Os

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
OBJS    = $(ASRC:%.s=$(OBJDIR)/%.o) $(SRC:%.c=$(OBJDIR)/%.o) $(CPPSRC:%.cpp=$(OBJDIR)/%.o)
LIBS    = $(DLIBS) $(ULIBS)
MCFLAGS = -mcpu=$(MCU)

#ASFLAGS = $(MCFLAGS) -g -gdwarf-2 -Wa,-amhls=$(<:.s=.lst) $(ADEFS)
CPFLAGS = $(MCFLAGS) $(OPT) -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -Wstrict-prototypes -fverbose-asm -Wa,-ahlms=$(<:.c=.lst) $(DEFS)
LDFLAGS = $(MCFLAGS) -mthumb -nostartfiles -T$(LDSCRIPT) -Wl,-Map=$(FULL_PRJ).map,--cref,--no-warn-mismatch $(LIBDIR)
CPPFLAGS = $(MCFLAGS) $(OPT) -gdwarf-2 -mthumb -fomit-frame-pointer -Wall -fverbose-asm -Wa,-ahlms=$(<:.cpp=.lst) $(DEFS)

CPPFLAGS += $(CPPDEFS)
CPFLAGS += $(CPPDEFS)

#   ifeq ($(STAMP), 1)
AVER = ${shell sh -c "grep \"SUB_VERS\" stamp-ersky9x.h | egrep -o \"[[:digit:]]*\""}
VER = $(shell echo $$(( $(AVER) + 1 )))
CPPDEFS += -DSTAMP
#   endif

# If REVB board
ifeq ($(REVB), 1)
 CPPDEFS += -DREVB
 EXT_MOD=REVB
endif

# If REVX board
ifeq ($(REVX), 1)
 CPPDEFS += -DREVX
 EXT_MOD=REVB-X
endif

# If Flight Phases
ifeq ($(PHASES), 1)
 CPPDEFS += -DPHASES
endif

CPPDEFS += -DFRSKY
CPPDEFS += -DCPUARM

# If serial Debug required
ifeq ($(DEBUG), 1)
 CPPDEFS += -DDEBUG
endif

# Generate dependency information
CPFLAGS += -MD -MP -MF .dep/$(@F).d
CPPFLAGS += -MD -MP -MF .dep/$(@F).d

#
# makefile rules
#

all: size_before stamp $(OBJS) $(FULL_PRJ).elf $(FULL_PRJ).hex $(FULL_PRJ).lss $(FULL_PRJ).bin size

size_before:
#	arm-none-eabi-size $(FULL_PRJ).elf
	@if test -f $(FULL_PRJ).elf; then arm-none-eabi-size $(FULL_PRJ).elf; fi

size:
#	arm-none-eabi-size $(FULL_PRJ).elf
	@if test -f $(FULL_PRJ).elf; then arm-none-eabi-size $(FULL_PRJ).elf; fi

# Build stamp-file
stamp:
	@echo
ifeq ($(STAMP), 1)
	@echo "Generate Version-stamp:"
	@echo "//Automatically generated file - do not edit" > stamp-ersky9x.h
	@echo "#define DATE_STR \"`date +%d.%m.%Y`\"" >> stamp-ersky9x.h
	@echo "#define TIME_STR \"`date +%H:%M:%S`\"" >> stamp-ersky9x.h
	@echo "#define SUB_VERS $(VER)-$(USER)" >> stamp-ersky9x.h
 ifeq ($(PCB), X9D)
	@echo "#define SVN_VERS \"erx9d-r$(REV)\"" >> stamp-ersky9x.h
 else	
	@echo "#define SVN_VERS \"ersky9x-r$(REV)\"" >> stamp-ersky9x.h
 endif
	@echo "#define MOD_VERS \"$(EXT_MOD)\"" >> stamp-ersky9x.h
#	@cat stamp-ersky9x.h
endif

$(OBJDIR)/%.o : %.cpp
	$(CC) -c $(CPPFLAGS) -fno-exceptions -I . $(INCDIR) $< -o $@

$(OBJDIR)/%.o : %.c
	$(CC) -c $(CPFLAGS) -I . $(INCDIR) $< -o $@

$(OBJDIR)/%.o : %.s
	$(AS) -c $(ASFLAGS) $< -o $@

%elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LIBS) -o $@
  
# Create extended listing file from ELF output file.
%.lss: %.elf
	$(CLSS) -h -S $< > $@

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