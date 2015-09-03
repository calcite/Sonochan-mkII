@echo off
REM This script should update firmware through DFU
REM Path is little bit modifed.
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


REM Path to batchisp
SET PATH_TO_BATCHISP=c:\Program Files\Atmel\Flip 3.4.7\bin\
REM And because at 64 bit windows is everything up side down, we also add this to path
SET PATH_TO_BATCHISP=%PATH_TO_BATCHISP%;c:\Program Files (x86)\Atmel\Flip 3.4.7\bin\

REM DO NOT TOUCH!
SET PATH=%PATH_TO_BATCHISP%;%PATH%
batchisp.exe -hardware usb -device at32uc3a3256 -operation erase f memory flash blankcheck loadbuffer Release\Sonochan_mkII.elf program verify start reset 0
