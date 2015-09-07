@echo off
REM This script take ELF, convert it to intel HEX and upload it to device.
REM
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

REM Path to DFU programmer binary
SET PATH_TO_DFU=C:\Program Files\dfu-programmer-win\
REM Path to DFU programmer binary for x64 systems
SET PATH_TO_DFU_x86=C:\Program Files (x86)\dfu-programmer-win\


REM DO NOT TOUCH!!!
SET PATH=%PATH_TO_DFU%;%PATH_TO_DFU_x86%;%PATH%

SET line=-------------------------------------------------------------------------------

REM There are some problems when writing to user flash memory programming
REM in older versions
echo %line%
echo Starting... Supporting only dfu-programmer 7.2 and newer

dfu-programmer at32uc3a3256 erase --debug 6 2> fw_update.log &&^
echo %line% &&^
echo Chip erased &&^
echo %line% &&^
dfu-programmer at32uc3a3256 flash --suppress-bootloader-mem ^
               Release/Sonochan_mkII_prog.hex --debug 6 2>> fw_update.log &&^
echo %line% &&^
echo Firmware sucessfuly updated :) &&^
echo %line% &&^
dfu-programmer at32uc3a3256 flash --force --user ^
  Release/Sonochan_mkII_user.hex --debug 6 2>> fw_update.log &&^
echo %line% &&^
echo User page updated. Ready to launch application. :B &&^
dfu-programmer at32uc3a3256 launch --debug 4 2>> fw_update.log &&^
echo %line% &&^
echo Running application. Have fun. &&^
echo %line%

if %ERRORLEVEL% == 0 echo All OK :)
if %ERRORLEVEL% NEQ 0 echo Something went wrong :( Check fw_update.log
