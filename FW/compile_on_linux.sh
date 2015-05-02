#!/bin/bash
#
# This allow find "avr32-gcc" program in case, that is not in your path.
# Because every user can have different path to AVR32 binary, this is the easy
# way to solve problem with missing command.
# Just change it to your needs.  
#
PATH=${AVR32BIN:=/opt/non-portage/avr32-tools/bin}:$PATH
# Example:
#XXXXXXXXXXXXXXXX| Just change this              |XXXXXX
#PATH=${AVR32BIN:=/opt/unofficial/avr32-tools/bin}:$PATH

# DO NOT TOUCH!
export PATH
cd Release && make all
