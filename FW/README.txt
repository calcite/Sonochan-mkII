# Compilation

## Compilation under Windows
 Basicly project need only "make" (and basic programs like "rm", "cp" and so
 on), "objcopy", "avr32gcc" and standard AVR32 libraries.
 So easiest way is install Atmel Studio 6 and just set correct paths. Or you
 can install just basic programs (MinGW, avr32-gnu-toolchain) and set correct
 paths. It is up to you.

### With AVR Studio 6.1
 * Prepare yourself few coffees, get ready and install AVR Studio 6.1
 * If you using old WinXP, install Service Pack 3 first. It is required anyway
 * To make your journey easy install it to default location (C:\Program files\)
 * Run "compile_on_windows.bat" in console. If fails, then open this file and
   edit variables "PATH_TO_MAKE" (where is make.exe) and "PATH_TO_GCC"
   (avr32-gcc.exe).
 
### MinGW and avr32-gnu-toolchain
 * Open "compile_on_windows.bat" in your favorite text editor
 * Edit variables "PATH_TO_MAKE" (where is make.exe), "PATH_TO_GCC"
   (avr32-gcc.exe) and "PATH_TO_OBJCOPY" (objcopy.exe)
 * Run "compile_on_windows.bat" in console

## Compilation under Linux
 * As first make sure, that you have installed "make" (Try type "make" without
   quotas in console). If appears something like "command not found", then you
   need to install "make" (please use uncle Google)
 * Open file "compile_on_linux.sh" and edit line begun with
   "PATH=${AVR32BIN:=" . Just change path to "avr32-bin" command.
 * If needed "chmod 755 compile_on_linux.sh" (script will be executable)
 * run "./compile_on_linux.sh" 



# Updating firmware
 * Because this device is still under development and probably for a long time
   will be, it is good to know how to update firmware.

## Updating under Linux
 * Because avrdude does not support AVR32 (yet), it is not possible to update
   firmware through JTAG. However AVR32 chips are usually selled with
   bootloader, so you can update firmware witout need HW AVR programmer.
   
### Using DFU programmer
 * You will probably need "dfu-programmer". It is usually part of package
   system, so there should not be problem. At Ubuntu like system just type:
   sudo apt-get install dfu-programmer
   If you really like compiling source codes and you are using Gentoo, then:
   sudo emerge -v dfu-programmer
 * On board connect signal "PROG" (TP12) to "GND". In newer versions there
   probably will be some button, or extra pin on case. If you are not sure,
   just check schematics.
 * Now connect device through USB cable to PC.
 * Run script "./update_fw_linux.sh"
   Script should do everything automatically. If there is some problem,
   script will tell you.

## Updating under Windows
 * There are 2 options: JTAG and DFU. JTAG allow debug and do awesome stuff,
   but sometimes stepping cause side effects, so debugging is useless. Also
   some AVR programmer is needed. If you use DFU option, you just need usb
   cable, but debugging is quite hell. However for updating firmware it is
   enough.

### Using DFU programmer
 * The "less evil way" is to get Flip application (hopefully still on Atmel
   site).
 * On board connect signal "PROG" (TP12) to "GND". In newer versions there
   probably will be some button, or extra pin on case. If you are not sure,
   just check schematics.
 * Now connect device through USB cable to PC.
 * Windows probably does not know device, so you have to install drivers
   manually. Luckily developers at Atmel think about this. Drivers are part of
   Flip application (usb folder).
 * Check path variable in update_fw_windows.bat file.
   Now you can run script "update_fw_windows.bat". Script should do all
   annoying routines.

### Using JTAG programmer
 * Allow easy debugging, but reprogramming by JTAG can cause erasing whole
   flash memory (include bootloader!!!). In FW/Release there are .bat scripts
   for some most famous Atmel programmers, so you can use/modify them.
      
# Restoring bootloader
 * As mentioned above, if you use JTAG programmer, bootloader can be eased.
   Of course Atmel provive many documents about this, but they are very old
   and refer to old tools, which are not available. Furthemore their bootloader
   image files did not worked for me (for unknown reason). Long story short:
   there is prepared script which recover bootloader.
   Surprisingly it is located in "bootloader" folder.
 * Just be sure, that paths in "dragon_upload_bootloader.bat" are correct.
 * Run "dragon_upload_bootloader.bat"