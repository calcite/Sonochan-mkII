# Compilation

## Compilation under Windows
 Basicly project need only "make" (and basic programs like "rm", "cp" and so
 on), "avr32gcc" and standard AVR32 libraries.
 So easiest way is install Atmel Studio 6 and just set correct paths. Or you
 can install just basic programs (MinGW, avr32-gnu-toolchain) and set correct
 paths.

### With AVR Studio 6.1
 * Prepare yourself few coffees, get ready and install AVR Studio 6.1
 * If you using old WinXP, install Service Pack 3 first. It is required anyway
 * To make your journey easy install it to default location (C:\Program files\)
 * Run "compile_on_windows.bat" in console. If fails, then open this file and
   edit variables "PATH_TO_MAKE" (where is make.exe) and "PATH_TO_GCC"
   (avr32-gcc.exe).
 
### MinGW and avr32-gnu-toolchain
 * Open "compile_on_windows.bat" in your favorite text editor
 * Edit variables "PATH_TO_MAKE" (where is make.exe) and "PATH_TO_GCC"
   (avr32-gcc.exe).
 * Run "compile_on_windows.bat" in console

## Compilation under Linux
 * As first make sure, that you have installed "make" (Try type "make" without
   quotas in console). If appears something like "command not found", then you
   need to install "make" (please use uncle Google)
 * Open file "compile_on_linux.sh" and edit line begun with
   "PATH=${AVR32BIN:=" . Just change path to "avr32-bin" command.
 * If needed "chmod 755 compile_on_linux.sh" (script will be executable)
 * run "./compile_on_linux.sh" 
      
