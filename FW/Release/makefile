# Makefile for Sonochan mkII

# Optimizations - write as parameter. Eq: OPT=-O2
OPT=-O2
# Debug level - write as parameter (can be void). Eq: DBG=-gdwarf-2
# or: DBG=-g2
# or can be void (no debug informations). Eq.: DBG=
DBG=-gdwarf-2



# DO NOT TOUCH!
# Name of text file containing build number.
include build_counter.mak

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include src/subdir.mk
-include src/SOFTWARE_FRAMEWORK/UTILS/STARTUP_FILES/GCC/subdir.mk
-include src/SOFTWARE_FRAMEWORK/UTILS/DEBUG/subdir.mk
-include src/SOFTWARE_FRAMEWORK/SERVICES/USB/CLASS/HID/subdir.mk
-include src/SOFTWARE_FRAMEWORK/SERVICES/USB/CLASS/AUDIO/subdir.mk
-include src/SOFTWARE_FRAMEWORK/SERVICES/FREERTOS/Source/subdir.mk
-include src/SOFTWARE_FRAMEWORK/SERVICES/FREERTOS/Source/portable/MemMang/subdir.mk
-include src/SOFTWARE_FRAMEWORK/SERVICES/FREERTOS/Source/portable/GCC/AVR32_UC3/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/WDT/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/USBB/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/USBB/ENUM/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/USBB/ENUM/DEVICE/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/USART/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/TWIM/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/TC/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/SSC/I2S/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/RTC/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/PM/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/PDCA/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/INTC/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/GPIO/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/FLASHC/subdir.mk
-include src/SOFTWARE_FRAMEWORK/DRIVERS/EIC/subdir.mk
-include src/SOFTWARE_FRAMEWORK/BOARDS/SDRwdgtLite/subdir.mk
-include src/SOFTWARE_FRAMEWORK/ASM/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: elf hex build_cnt

# Build elf file
elf: $(OBJS) $(USER_OBJS)
	@echo Link $@
	@avr32-gcc -nostartfiles -Wl,--gc-sections -Wl,-e,_trampoline \
	 -mpart=uc3a3256 --rodata-writable -Wl,--direct-data \
	 -Wl,--stack,0x2000 \
	 -o"Sonochan_mkII.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@avr32-size -A "Sonochan_mkII.elf"
	@echo "  -> RAM usage (without malloc and stack) is data + bss"
	@echo "  -> Flash usage is text + data"


hex: hex_program hex_user

hex_program:
# Create hex file, which is important for DFU tools
# Grab elf file and create hex
# basically we have to skip
# .userpage
# Everything else is OK
	@echo Creating program HEX file
	@objcopy -O ihex --only-section=.reset \
                --only-section=.rela.got \
                --only-section=.text \
                --only-section=.exception \
                --only-section=.rodata \
                --only-section=.dalign \
                --only-section=.data \
                --only-section=.balign \
                --only-section=.bss \
                --only-section=.comment \
                --only-section=.debug_aranges \
                --only-section=.debug_info \
                --only-section=.debug_abbrev \
                --only-section=.debug_line \
                --only-section=.debug_frame \
                --only-section=.stack \
                --only-section=.debug_ranges \
                --only-section=.shstrtab \
                --only-section=.symtab \
                --only-section=.strtab \
                Sonochan_mkII.elf \
                Sonochan_mkII_prog.hex
hex_user:
	@echo Creating user flash HEX file
	@objcopy -O ihex --only-section=.userpage \
                Sonochan_mkII.elf \
                Sonochan_mkII_user.hex
                
# Just clean project. No more binary files :)
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) Sonochan_mkII.elf Sonochan_mkII.hex

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
