# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/tinyprintf.c \
../src/tinyprintf_extra.c \
../src/bootloader_cfg.c \
../src/brd_driver_hw_03.c \
../src/cs2200_HAL_AVR32_UC3A3_HW_interface.c \
../src/cs2200.c \
../src/tlv320aic33_HAL_AVR32_UC3A3_HW_interface.c \
../src/tlv320aic33.c \
../src/LCD_5110_HAL_AVR32_UC3A3_HW_interface.c \
../src/LCD_5110.c \
../src/display.c \
../src/usb_descriptors.c \
../src/usb_specific_request.c \
../src/generic_driver.c \
../src/HW.c \
../src/HW_bridge_uniprot.c \
../src/uniprot.c \
../src/ssc.c \
../src/composite_widget.c \
../src/device_audio_task.c \
../src/device_generic_hid.c \
../src/host_audio_task.c \
../src/image.c \
../src/taskAK5394A.c \
../src/taskEXERCISE.c \
../src/uac1_device_audio_task.c \
../src/uac1_image.c \
../src/uac1_taskAK5394A.c \
../src/uac1_usb_descriptors.c \
../src/uac1_usb_specific_request.c \
../src/uac2_device_audio_task.c \
../src/uac2_image.c \
../src/uac2_taskAK5394A.c \
../src/uac2_usb_descriptors.c \
../src/uac2_usb_specific_request.c \



OBJS += \
./src/tinyprintf.o \
./src/tinyprintf_extra.o \
./src/bootloader_cfg.o \
./src/brd_driver_hw_03.o \
./src/cs2200_HAL_AVR32_UC3A3_HW_interface.o \
./src/cs2200.o \
./src/tlv320aic33_HAL_AVR32_UC3A3_HW_interface.o \
./src/tlv320aic33.o \
./src/LCD_5110_HAL_AVR32_UC3A3_HW_interface.o \
./src/LCD_5110.o \
./src/display.o \
./src/usb_descriptors.o \
./src/usb_specific_request.o \
./src/generic_driver.o \
./src/HW.o \
./src/HW_bridge_uniprot.o \
./src/uniprot.o \
./src/ssc.o \
./src/composite_widget.o \
./src/device_audio_task.o \
./src/device_generic_hid.o \
./src/host_audio_task.o \
./src/image.o \
./src/taskAK5394A.o \
./src/taskEXERCISE.o \
./src/uac1_device_audio_task.o \
./src/uac1_image.o \
./src/uac1_taskAK5394A.o \
./src/uac1_usb_descriptors.o \
./src/uac1_usb_specific_request.o \
./src/uac2_device_audio_task.o \
./src/uac2_image.o \
./src/uac2_taskAK5394A.o \
./src/uac2_usb_descriptors.o \
./src/uac2_usb_specific_request.o \



C_DEPS += \
./src/tinyprintf.d \
./src/tinyprintf_extra.d \
./src/bootloader_cfg.d \
./src/brd_driver_hw_03.d \
./src/cs2200_HAL_AVR32_UC3A3_HW_interface.d \
./src/cs2200.d \
./src/tlv320aic33_HAL_AVR32_UC3A3_HW_interface.d \
./src/tlv320aic33.d \
./src/LCD_5110_HAL_AVR32_UC3A3_HW_interface.d \
./src/LCD_5110.d \
./src/display.d \
./src/usb_descriptors.d \
./src/usb_specific_request.d \
./src/generic_driver.d \
./src/HW.d \
./src/HW_bridge_uniprot.d \
./src/uniprot.d \
./src/ssc.d \
./src/composite_widget.d \
./src/device_audio_task.d \
./src/device_generic_hid.d \
./src/host_audio_task.d \
./src/image.d \
./src/taskAK5394A.d \
./src/taskEXERCISE.d \
./src/uac1_device_audio_task.d \
./src/uac1_image.d \
./src/uac1_taskAK5394A.d \
./src/uac1_usb_descriptors.d \
./src/uac1_usb_specific_request.d \
./src/uac2_device_audio_task.d \
./src/uac2_image.d \
./src/uac2_taskAK5394A.d \
./src/uac2_usb_descriptors.d \
./src/uac2_usb_specific_request.d \


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo Compile $(CFLAGS) $<
	@avr32-gcc $(CFLAGS) \
	 -ffast-math \
	 -mfast-float \
	 -DBOARD=SDRwdgtLite \
	 -DFREERTOS_USED \
         -D__BUILD_DATE=`date +'%s'` \
         -D__BUILD_NUMBER_A=$(BUILD_NUM_A) \
	 -D__BUILD_NUMBER_B=$(BUILD_NUM_B) \
	 -D__BUILD_NUMBER_C=$(BUILD_NUM_C) \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/PDCA \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/TWIM \
	 -I../src/SOFTWARE_FRAMEWORK/UTILS/DEBUG \
	 -I../src/SOFTWARE_FRAMEWORK/SERVICES/USB/CLASS/AUDIO \
	 -I../src/SOFTWARE_FRAMEWORK/SERVICES/USB/CLASS/CDC \
	 -I../src/SOFTWARE_FRAMEWORK/SERVICES/FREERTOS/Source/portable/GCC/AVR32_UC3 \
	 -I../src/SOFTWARE_FRAMEWORK/SERVICES/FREERTOS/Source/include \
	 -I../src/SOFTWARE_FRAMEWORK/SERVICES/USB/CLASS/HID \
	 -I../src/SOFTWARE_FRAMEWORK/SERVICES/USB \
	 -I../src/CONFIG \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/USBB/ENUM/DEVICE \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/USBB/ENUM \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/USBB \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/USART \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/TC \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/WDT \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/CPU/CYCLE_COUNTER \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/EIC \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/RTC \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/PM \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/GPIO \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/FLASHC \
	 -I../src/SOFTWARE_FRAMEWORK/UTILS/LIBS/NEWLIB_ADDONS/INCLUDE \
	 -I../src/SOFTWARE_FRAMEWORK/UTILS/PREPROCESSOR \
	 -I../src/SOFTWARE_FRAMEWORK/UTILS \
	 -I../src/SOFTWARE_FRAMEWORK/DRIVERS/INTC \
	 -I../src/SOFTWARE_FRAMEWORK/BOARDS \
	 -I../src \
	$(OPT) $(DBG) -fdata-sections -Wall -c -fmessage-length=0 \
	 -mpart=uc3a3256 -ffunction-sections -masm-addr-pseudos -MMD -MP \
	 -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"


