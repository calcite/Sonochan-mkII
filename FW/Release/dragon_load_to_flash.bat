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
REM Erase flash
atprogram -v -t avrdragon -i JTAG -d at32uc3a3256 chiperase
echo "
echo --- Memory erased ---
echo " 
REM Upload firmware 
atprogram -v -t avrdragon -i JTAG -d at32uc3a3256 program -f Sonochan_mkII.elf

