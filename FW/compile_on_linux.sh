#!/bin/bash
#
# This allow find "avr32-gcc" program in case, that is not in your path.
# Because every user can have different path to AVR32 binary, this is the easy
# way to solve problem with missing command.
# Just change it to your needs.  
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

PATH=${AVR32BIN:=/opt/non-portage/avr32-tools/bin}:$PATH
# Example:
#XXXXXXXXXXXXXXXX| Just change this              |XXXXXX
#PATH=${AVR32BIN:=/opt/unofficial/avr32-tools/bin}:$PATH

# DO NOT TOUCH!
export PATH
cd Release && make all
