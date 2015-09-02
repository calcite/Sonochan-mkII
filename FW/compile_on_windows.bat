@echo off
REM This file should help compile project on Windows
REM Path is little bit modifed.
REM This allow find "make", "avr32-gcc" and "objcopy" program in case,
REM that is not in your path.
REM Because every user can have different path to binaries, this is the
REM easy way to solve problem with missing command.
REM Just change it to your needs.  

REM Define directory where is "make.exe" command  
SET PATH_TO_MAKE=c:\Program Files\Atmel\Atmel Studio 6.1\shellUtils\
REM Again, but just for case for 64 bit
SET PATH_TO_MAKE=%PATH_TO_MAKE%;c:\Program Files (x86)\Atmel\Atmel Studio 6.1\shellUtils\

REM Define directory where is "avr32-gcc" command
SET PATH_TO_GCC=c:\Program Files\Atmel\Atmel Toolchain\AVR32 GCC\Native\3.4.2.1002\avr32-gnu-toolchain\bin\
REM Again, but just for case for 64 bit
SET PATH_TO_GCC=%PATH_TO_GCC%;c:\Program Files (x86)\Atmel\Atmel Toolchain\AVR32 GCC\Native\3.4.2.1002\avr32-gnu-toolchain\bin\

REM Define directory where is "objcopy" command
SET PATH_TO_OBJCOPY=c:\Program Files\Atmel\Atmel Toolchain\AVR32 GCC\Native\3.4.2.1002\avr32-gnu-toolchain\avr32\bin\
REM And once more for 64 bit systems
SET PATH_TO_OBJCOPY=%PATH_TO_OBJCOPY%;c:\Program Files (x86)\Atmel\Atmel Toolchain\AVR32 GCC\Native\3.4.2.1002\avr32-gnu-toolchain\avr32\bin\

REM DO NOT TOUCH!
SET PATH=%PATH_TO_MAKE%;%PATH_TO_GCC%;%PATH_TO_OBJCOPY%;%PATH%;
cd Release && make all
cd ..
