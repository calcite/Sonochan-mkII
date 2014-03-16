@echo off

REM Clear screen
cls

REM make clean
REM make


SET PATH=%PATH%;c:\Program Files (x86)\Atmel\Atmel Studio 6.1\atbackend\

REM Switch "-p" options: f (flash)/ a (all)
REM avrone -d AT90USB1287 -e -pa -J 1000000 -ia CodecKit.elf
REM atprogram -t avrdragon -i JTAG -d at32uc3a0512 program -f file.hex

REM atprogram -v -t jtagicemkii -i JTAG -d at32uc3a0512 erase

echo ---------------------

atprogram -v -t jtagicemkii -i JTAG -d at32uc3a32556 -cl 10khz --timeout 0 program -f widget.elf

