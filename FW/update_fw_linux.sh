#!/bin/bash
#
# This script should make updating FW a lot of easier
#
# Sonochan Mk.II Firmware.
# Copyright (C) 2013-2015 Martin Stejskal,
# ALPS Electric Czech, s.r.o.,
# ALPS Electric Co., Ltd.  
#  
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#  
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#  
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#  
# Contact information:
# Martin Stejskal <martin.stejskal@jp.alps.com>,
# Josef Nevrly <jnevrly@alps.cz>
# ALPS Electric Czech, s.r.o., Sebranice 240, 679 31 Sebranice u Boskovic
# Czech Republic

line="\n\
===============================================================================
\n"

# There are some problems when writing to user flash memory programming
# in older versions
echo "Starting... Supporting only dfu-programmer 7.2 and newer"

dfu-programmer at32uc3a3256 erase --debug 6 2> fw_update.log &&
echo -e ""$line"Chip erased"$line"" &&

dfu-programmer at32uc3a3256 flash --suppress-bootloader-mem \
               Release/Sonochan_mkII_prog.hex --debug 6 2>> fw_update.log &&
echo -e ""$line"Firmware sucessfuly updated :)"$line"" &&


#dfu-programmer at32uc3a3256 flash-user --suppress-validation
dfu-programmer at32uc3a3256 flash --force --user \
  Release/Sonochan_mkII_user.hex --debug 6 2>> fw_update.log &&
echo -e ""$line"User page updated. Ready to launch application. ^_^"$line"" &&

# For new dfu programmer. However still not in Ubuntu repository
# dfu-programmer at32uc3a3256 reset --debug 4 2>> fw_update.log &&
dfu-programmer at32uc3a3256 launch --debug 4 2>> fw_update.log &&
echo -e ""$line"Running application. Have fun."$line"" 


if [ $? -eq 0 ] ; then
  echo -e ""$line"All OK :)"$line""
else
  echo -e ""$line"Something gets wrong :(\n Check fw_update.log"$line""
fi
