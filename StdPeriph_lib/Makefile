CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

vpath %.c src

# Please make sure this matches your project folder as well!
MCU_FAMILY = STM32F10X_HD

CFLAGS  = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m3 -march=armv7-m
CFLAGS += -ffreestanding -nostdlib
CFLAGS += -includestm32f10x_conf.h -ICMSIS/CM3/CoreSupport -ICMSIS/CM3/DeviceSupport/ST/STM32F10x -Iinc -I.
CFLAGS += -D$(MCU_FAMILY)
SRCS = stm32f10x_adc.c stm32f10x_bkp.c stm32f10x_can.c stm32f10x_cec.c \
	stm32f10x_crc.c stm32f10x_dac.c stm32f10x_dbgmcu.c stm32f10x_dma.c \
	stm32f10x_exti.c stm32f10x_flash.c stm32f10x_fsmc.c stm32f10x_gpio.c \
	stm32f10x_i2c.c stm32f10x_iwdg.c stm32f10x_pwr.c stm32f10x_rcc.c \
	stm32f10x_rtc.c stm32f10x_sdio.c stm32f10x_spi.c stm32f10x_tim.c \
	stm32f10x_usart.c stm32f10x_wwdg.c misc.c

OBJS = $(SRCS:.c=.o)

.PHONY: libstm32f1.a

all: libstm32f1.a

%.o : %.c 
	$(CC) $(CFLAGS) -c -o $@ $^

libstm32f1.a: $(OBJS)
	@echo Target MCU Family: $(value MCU) 
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) libstm32f1.a
