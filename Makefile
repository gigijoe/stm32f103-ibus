# Please add all the appropriate src files, (*.o) targets
SRCS= main.c system_stm32f10x.c delay.c usart.c

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJ_NAME=main

# Location of the stm32f10x periph library
STD_PERIPH_LIB = StdPeriph_lib
CMSIS = $(STD_PERIPH_LIB)/CMSIS/CM3

# Location of the linker scripts
LDSCRIPT_INC=Device/ldscripts

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size

BIN=$(CP) -O ihex 

# Select the appropriate option for your device, the available options are listed below
# with a description copied from stm32f10x.h
# Make sure to set the startup code file to the right device family, too!
#
# STM32F10X_LD 		STM32F10X_LD: STM32 Low density devices
# STM32F10X_LD_VL	STM32F10X_LD_VL: STM32 Low density Value Line devices
# STM32F10X_MD		STM32F10X_MD: STM32 Medium density devices
# STM32F10X_MD_VL	STM32F10X_MD_VL: STM32 Medium density Value Line devices 
# STM32F10X_HD		STM32F10X_HD: STM32 High density devices
# STM32F10X_HD_VL	STM32F10X_HD_VL: STM32 High density value line devices
# STM32F10X_XL		STM32F10X_XL: STM32 XL-density devices
# STM32F10X_CL		STM32F10X_CL: STM32 Connectivity line devices 
#
# - Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 16 and 32 Kbytes.
# 
# - Low-density value line devices are STM32F100xx microcontrollers where the Flash
#   memory density ranges between 16 and 32 Kbytes.
# 
# - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 64 and 128 Kbytes.
# 
# - Medium-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 64 and 128 Kbytes.   
# 
# - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 256 and 512 Kbytes.
# 
# - High-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 256 and 512 Kbytes.   
# 
# - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 512 and 1024 Kbytes.
# 
# - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
#
# HSE_VALUE sets the value of the HSE clock, 8MHz in this case 

# PLEASE CHOOSE THE CORRECT MCU FOR YOUR APPLICATION
MCU_FAMILY = STM32F10X_HD
HSE_VALUE = 8000000

# Default to selecting the STD_PERIPH_DRIVER
DEFS = -DUSE_STDPERIPH_DRIVER -D$(MCU_FAMILY) -DHSE_VALUE=$(HSE_VALUE)

vpath %.c src

MCU = cortex-m3
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mthumb-interwork

STM32_INCLUDES = -I$(CMSIS)/DeviceSupport/ST/STM32F10x/ \
	-I$(CMSIS)/CoreSupport/ \
	-Iinc -I$(STD_PERIPH_LIB) -I$(STD_PERIPH_LIB)/inc

OPTIMIZE       = -Os

CFLAGS	= $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(STM32_INCLUDES)
CFLAGS += -lm -lc -lnosys -specs=nosys.specs -Wl,-Map=$(PROJ_NAME).map -Wl,-T,stm32_flash.ld
AFLAGS	= $(MCFLAGS) 

SRCS += stm32f10x_it.c 
SRCS += ./Device/startup_stm32f10x_hd.s

OBJDIR = .
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) 
OBJ += Startup.o

.PHONY: lib proj

all: lib proj 

lib:
	$(warning Please check that the MCU in the makefile matches what you expect.)
	$(MAKE) -C $(STD_PERIPH_LIB)
	
proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	@echo Please check $(LDSCRIPT_INC) to ensure that the linker script is correct.
	@echo
	@echo $(value STARTUP)
	$(CC) $(CFLAGS) $^ -o $@ -L$(STD_PERIPH_LIB) -lstm32f1 -L$(LDSCRIPT_INC)
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -St $(PROJ_NAME).elf >$(PROJ_NAME).lst
	$(SIZE) $(PROJ_NAME).elf

clean:
	find ./ -name '*~' | xargs rm -f	
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).map
	rm -f $(PROJ_NAME).lst
	rm -f $(STD_PERIPH_LIB)/*.o

reallyclean: clean
	$(MAKE) -C $(STD_PERIPH_LIB) clean
