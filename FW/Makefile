##
## make both the widget
## and the widget-control that matches
## the features in the widget
##
## assumes that you've set the AVR32BIN environment
## to point to the directory containing avr32-gcc

# [Martin] Let's give AVR32BIN path directly thru Makefile to make it clear
# AVR32BIN=/opt/non-portage/avr32-tools/bin/
#AVR32BIN=C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR32 GCC\Native\3.4.2.1002\avr32-gnu-toolchain\avr32\bin

all:: Release/widget.elf

Release/widget.elf::
	./make-widget

audio-widget::
	rm -f Release/widget.elf Release/src/features.o
	CFLAGS=-DFEATURE_DEFAULT_BOARD=feature_board_dib ./make-widget

sdr-widget::
	rm -f Release/widget.elf Release/src/features.o
	CFLAGS=-DFEATURE_DEFAULT_BOARD=feature_board_usbi2s ./make-widget

clean::
	cd Release && make clean
	rm -f widget-control
