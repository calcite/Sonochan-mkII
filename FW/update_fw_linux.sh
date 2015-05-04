#!/bin/bash
#
# This script take ELF, convert it to intel HEX and upload it to device.

line="\n\
===============================================================================
\n"

echo "Starting..."

dfu-programmer at32uc3a3256 erase --debug 6 2> fw_update.log &&
echo -e ""$line"Chip erased"$line"" &&

dfu-programmer at32uc3a3256 flash --suppress-bootloader-mem \
               Release/Sonochan_mkII_prog.hex --debug 6 2>> fw_update.log &&
echo -e ""$line"Firmware sucessfuly updated! ^_^"$line"" &&

dfu-programmer at32uc3a3256 flash --force --user \
               Release/Sonochan_mkII_user.hex --debug 6 2>> fw_update.log &&
echo -e ""$line"User page updated. Ready to launch application." &&

dfu-programmer at32uc3a3256 launch --debug 4 2>> fw_update.log &&
echo -e ""$line"Running application. Have fun."$line"" 


if [ $? -eq 0 ] ; then
  echo -e ""$line"All OK :)"$line""
else
  echo -e ""$line"Something gets wrong :(\n Check fw_update.log"$line""
fi