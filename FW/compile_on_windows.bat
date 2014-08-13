@echo off
REM This file should help compile project on Windows
REM This allow find "make" and "avr32-gcc" program in case, that is not in your
REM path.
REM Because every user can have different path to binaries, this is the
REM easy way to solve problem with missing command.
REM Just change it to your needs.  

REM Define directory where is "make.exe" command  
SET PATH_TO_MAKE=c:\Program Files\Atmel\Atmel Studio 6.1\shellUtils\

REM Define directory where is "avr32-gcc" command
SET PATH_TO_GCC=c:\Program Files\Atmel\Atmel Toolchain\AVR32 GCC\Native\3.4.2.1002\avr32-gnu-toolchain\bin\

REM DO NOT TOUCH!
SET PATH=%PATH_TO_MAKE%;%PATH_TO_GCC%;%PATH%;
EXPORT PATH
cd Release && make all