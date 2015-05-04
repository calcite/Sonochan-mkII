@echo off
REM Simple script for loading firmware through AVR Dragon

REM Clear screen 
cls

REM Here Should be defined path to "atprogram.exe" Because of 64 bit systems
REM there is this line 2x (just for case that someone use 64 bit and someone
REM else use 32 bit system)
SET PATH=%PATH%;c:\Program Files\Atmel\Atmel Studio 6.1\atbackend\
SET PATH=%PATH%;c:\Program Files (x86)\Atmel\Atmel Studio 6.1\atbackend\

echo ---------------------
REM Erase flash
atprogram -v -t avrdragon -i JTAG -d at32uc3a3256 chiperase
echo "
echo --- Memory erased ---
echo " 
REM Upload firmware 
atprogram -v -t avrdragon -i JTAG -d at32uc3a3256 program -f Sonochan_mkII.elf

