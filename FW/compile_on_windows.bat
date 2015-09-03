@echo off
REM This file should help compile project on Windows
REM Path is little bit modified.
REM This allow find "make", "avr32-gcc" and "objcopy" program in case,
REM that is not in your path.
REM Because every user can have different path to binaries, this is the
REM easy way to solve problem with missing command.
REM Just change it to your needs.  

REM Sonochan Mk.II Firmware.
REM Copyright (C) 2013-2015 Martin Stejskal,
REM ALPS Electric Czech, s.r.o.,
REM ALPS Electric Co., Ltd.  
REM  
REM This program is free software; you can redistribute it and/or
REM modify it under the terms of the GNU General Public License
REM as published by the Free Software Foundation; either version 2
REM of the License, or (at your option) any later version.
REM  
REM This program is distributed in the hope that it will be useful,
REM but WITHOUT ANY WARRANTY; without even the implied warranty of
REM MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
REM GNU General Public License for more details.
REM  
REM You should have received a copy of the GNU General Public License
REM along with this program; if not, write to the Free Software
REM Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
REM  
REM Contact information:
REM Martin Stejskal <martin.stejskal@jp.alps.com>,
REM Josef Nevrly <jnevrly@alps.cz>
REM ALPS Electric Czech, s.r.o., Sebranice 240, 679 31 Sebranice u Boskovic
REM Czech Republic


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
