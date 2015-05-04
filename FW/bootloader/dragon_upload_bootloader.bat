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

REM Erase whole chip (also fuses)
atprogram -v -t avrdragon -i JTAG -d at32uc3a3256 chiperase
echo "
echo --- Memory erased ---
echo " 

REM Upload firmware 
atprogram -v -t avrdragon -i JTAG -d at32uc3a3256 program -f UC3A3256_DFU_flash.hex
echo "
echo --- Flash programmed ---
echo " 
REM Also upload "user page" (which is just another part of flash)
REM This part is commented because we want to start up bootloader no matter what.
REM So reprogramming is easy.
REM atprogram -v -t avrdragon -i JTAG -d at32uc3a3256 program -f UC3A3256_DFU_user_flash.hex
REM echo "
REM echo --- User page programmed ---
REM echo " 

REM Change fuses to correct ones
atprogram -t avrdragon -i JTAG -d at32uc3a3256 write -fs -o 0xFFFE1410 --values FFF7FFFF
echo "
echo --- Fuses changed ---
echo " 

echo ----------------------------------------------------------------
atprogram -t avrdragon -i JTAG -d at32uc3a3256 info
