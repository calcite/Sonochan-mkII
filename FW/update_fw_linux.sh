#!/bin/bash
#
# This script take ELF, convert it to intel HEX and upload it to device.

output_dir="Release"
elf_file="Sonochan_mkII"

line="\n\
===============================================================================
\n"

dfu-programmer at32uc3a3256 erase --debug 6 2> fw_update.log &&
echo -e ""$line"Chip erased (it is necessary, sorry)"$line"" &&

dfu-programmer at32uc3a3256 flash --suppress-bootloader-mem \
               ""$output_dir"/"$elf_file".hex" --debug 6 2>> fw_update.log &&
echo -e ""$line"Firmware sucessfuly updated! ^_^"$line"" &&

dfu-programmer at32uc3a3256 reset --debug 4 2>> fw_update.log &&
echo -e ""$line"Chip restarted. Have fun."$line"" 

if [ $? -eq 0 ] ; then
  echo -e ""$line"All OK :)"$line""
else
  echo -e ""$line"Something gets wrong :(\n Check fw_update.log"$line""
fi