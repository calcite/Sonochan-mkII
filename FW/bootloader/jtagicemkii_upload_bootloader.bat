@echo off
REM Simple script for loading firmware through AVR Dragon

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

REM Clear screen 
cls

REM Here Should be defined path to "atprogram.exe" Because of 64 bit systems
REM there is this line 2x (just for case that someone use 64 bit and someone
REM else use 32 bit system)
SET PATH=%PATH%;c:\Program Files\Atmel\Atmel Studio 6.1\atbackend\
SET PATH=%PATH%;c:\Program Files (x86)\Atmel\Atmel Studio 6.1\atbackend\


echo ---------------------

REM Erase whole chip (also fuses)
atprogram -v -t jtagicemkii -i JTAG -d at32uc3a3256 chiperase
echo "
echo --- Memory erased ---
echo " 

REM Upload firmware 
atprogram -v -t jtagicemkii -i JTAG -d at32uc3a3256 program -f UC3A3256_DFU_flash.hex
echo "
echo --- Flash programmed ---
echo " 
REM Also upload "user page" (which is just another part of flash)
REM This part is commented because we want to start up bootloader no matter what.
REM So reprogramming is easy.
REM atprogram -v -t jtagicemkii -i JTAG -d at32uc3a3256 program -f UC3A3256_DFU_user_flash.hex
REM echo "
REM echo --- User page programmed ---
REM echo " 

REM Change fuses to correct ones
atprogram -t jtagicemkii -i JTAG -d at32uc3a3256 write -fs -o 0xFFFE1410 --values 6D37FFFF
echo "
echo --- Fuses changed ---
echo " 

echo ----------------------------------------------------------------
atprogram -t jtagicemkii -i JTAG -d at32uc3a3256 info
