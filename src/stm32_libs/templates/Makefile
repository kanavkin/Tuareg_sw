####################################
# stm32 boctok standard makefile
####################################

# developers note: use "cat -etv Makefile" to check if all required tabs are present

PROGRAM = stm32f103vet6_primer

# select linker script according to cpu type:
LKR_SCRIPT = stm32_libs/stm32f10x/linker_scripts/stm32f10x_f512_r64.ld

# set defines for SPL configuration
DEFINES = -DSTM32F10X_HD
DEFINES += -DUSE_STDPERIPH_DRIVER

# compiler options
CFLAGS  = -c
CFLAGS += -march=armv7-m -mcpu=cortex-m3 -mthumb
CFLAGS += -fno-common -nostdlib -fno-builtin -ffreestanding
CFLAGS += -Wall -O0 -g
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += $(DEFINES)

## Library headers
CFLAGS += -I./
CFLAGS += -I/stm32_libs/stm32f10x/spl
CFLAGS += -I/stm32_libs/stm32f10x/cmsis
CFLAGS += -I/stm32_libs/stm32f10x/boctok

# linker options
LFLAGS  = -nostartfiles -Wl,--gc-sections -specs=nano.specs
LFLAGS += $(DEFINES)

# toolchain setting
CC 	= arm-none-eabi-gcc
LD      = arm-none-eabi-gcc
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE 	= arm-none-eabi-size

# internal variables
TARGET_BIN = $(PROGRAM).bin
TARGET_ELF = $(PROGRAM).elf

#################################
## code objects
#################################

# app code
OBJS  = main.o

# startup file
OBJS += stm32_libs/stm32f10x/startup_files/startup_stm32f10x_hd.o

# core support
OBJS += stm32_libs/stm32f10x/cmsis/system_hsi64_stm32f10x.o
OBJS += stm32_libs/stm32f10x/cmsis/core_cm3.o

# drivers
OBJS += stm32_libs/stm32f10x/boctok/stm32f10x_gpio_boctok.o
OBJS += stm32_libs/stm32f10x/spl/stm32f10x_rcc.c
OBJS += stm32_libs/stm32f10x/spl/stm32f10x_flash.c


#################################
## RULES
#################################
.PHONY: all clean write

all: $(TARGET_BIN)

#%.bin: %.elf
#    $(OBJCOPY) --strip-unneeded -O binary $< $@

$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) --strip-unneeded -O binary $(TARGET_ELF) $(TARGET_BIN) 

$(TARGET_ELF): $(OBJS) 
	$(LD) $(LFLAGS) -T$(LKR_SCRIPT) -o $(TARGET_ELF) $(OBJS) 

#%.o: %.c
#	$(CC) $(CFLAGS) -c $< -o $@

clean:
#	rm -rf *.o *.elf *.bin
	find . -type f -name '*.o' -delete
	find . -type f -name '*.elf' -delete
	find . -type f -name '*.bin' -delete

write:
	/opt/stlink/st-upload.sh $(TARGET_BIN)

