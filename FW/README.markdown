# Preamble
 * IF YOU WANT MODIFY FW, PLEASE READ THIS PARAGRAPH !!!
 * Please, please and once more please DO NOT USE "TABS"! I spend on this
   project more than 2 years and tabs do only problems. Just use SPACES
   instead (2 SPACES for offsets).
 * Why spaces instead of tabs? Because tab interpretation can be at every
   editor different. So when somebody change code and someone other want
   to read it, it can be very hard to read. And because tab length is not
   strictly defined, line length could be over 80 characters. Maybe you do not
   realize, but even nowadays 80 characters length have meaning (specially if
   you develop on laptop/netbook, which have small display).
 * Please also try to keep line length UP TO 80 CHARACTERS. Keep compassion
   with those they do not have large display.
 * When you use spaces, wide is strictly defined and it displays to all users
   same. No doubt about it. Also in Python is recommended to use spaces instead
   of tabs, so it is good habit for Pythonists.
 * Bottom line: USE SPACES (2 for offset) and DO NOT WRITE MORE THAN 80
   CHARACTERS AT ONE LINE.
 * Thanks                         Martin Stejskal

# Compilation

## Compilation under Linux
 * As first make sure, that you have installed `make` (Try type `make`to
   console). If appears something like `command not found`, then you
   need to install `make` (please use uncle Google)
 * Open file `compile_on_linux.sh` and edit line begun with
   `PATH=${AVR32BIN:=` . Just change path to `avr32-bin` command. Example:
`PATH=${AVR32BIN:=/opt/non-portage/avr32-tools/bin}:$PATH`
 * Make sure, that heareds were copied to `avr32-tools/avr32/include/`
   else compilation will fail.
 * If needed `chmod 755 compile_on_linux.sh` (script will be executable)
 * run `./compile_on_linux.sh` 


## Compilation under Windows
 Basicly project need only `make` (and basic programs like `rm`, `cp` and so
 on), `objcopy`, `avr32gcc` and standard AVR32 libraries.
 So easiest way is install Atmel Studio 6 and just set correct paths in file
 `compile_on_windows.bat`. Or you can install just basic programs
 (MinGW, avr32-gnu-toolchain) and set correct paths. It is up to you.

### With AVR Studio 6
 * Prepare yourself few coffees, get ready and install AVR Studio 6
 * If you using old WinXP, install Service Pack 3 first. It is required anyway
 * To make your journey easy, install it to default location
   (`C:\Program files\`)
 * Check paths in `compile_on_windows.bat`. Probably with new versions it will
   be changed. So for example if you use old Atmel Studio 6.0, you will need
   to change paths for sure.
 * Run `compile_on_windows.bat` in console. If fails, then open this file and
   edit variables `PATH_TO_MAKE` (where is `make.exe`) and `PATH_TO_GCC`
   (`avr32-gcc.exe`).
 
### MinGW and avr32-gnu-toolchain
 * Open `compile_on_windows.bat` in your favorite text editor
 * Edit variables `PATH_TO_MAKE` (where is `make.exe`), `PATH_TO_GCC`
   (`avr32-gcc.exe`) and `PATH_TO_OBJCOPY` (`objcopy.exe`)
 * Run `compile_on_windows.bat` in console


# Updating firmware
 * This process delete all user settings! So please backup your device settings
   if you need it.
 * Because this device is still under development and probably for a long time
   will be, it is good to know how to update firmware.

## Updating under Linux
 * Because avrdude does not support AVR32 (yet), it is not possible to update
   firmware through JTAG. However AVR32 chips are usually sold with
   bootloader, so you can update firmware without need HW AVR programmer.
   It is more safe and faster.
   
### Updating by DFU programmer
 * You will probably need `dfu-programmer`. It is usually part of package
   system, so there should not be problem. At Ubuntu like system just type:
   `sudo apt-get install dfu-programmer`
   If you really like compiling source codes and you are using Gentoo, then:
   `sudo emerge -v dfu-programmer`
 * Be sure, that dfu-programmer version is at least `0.7.2`! Old versions have
   some issues with programming user flash and unfortunately this function is
   needed.
 * Connect device to PC.
 * Then hold `DAI Reset` (`I2S Reset` at old versions) button and press for
   brief moment `Reset` button.
 * Release `DAI Reset` button. Screen back light should be off and your PC
   should recognize new device. This you can verify by using `lsusb` command.
   You should see something like this:
   `03eb:2ff1 Atmel Corp.`
   This indicate that bootloader is ready. 
 * Run script `./update_fw_linux.sh`
   Script should do everything automatically. If there is some problem,
   script will tell you (and then please check log).


## Updating under Windows
 * There are 2 options: JTAG and DFU. JTAG allow debug and do awesome stuff,
   but sometimes stepping cause side effects, so debugging is useless. Also
   some AVR programmer is needed. If you use DFU option, you just need usb
   cable, but debugging is quite hell. However for updating firmware it is
   enough.
 * And one more IMPORTANT thing about debugging: bootloader HAVE TO be ERASED!
   So in Atmel Studio you will need to erase whole chip before starting
   debugging. Keep this in mind.

### Updating by Flip
 * The "less evil way" is to get Flip application (hopefully still on Atmel
   site).
 * Now connect device through USB cable to PC.
 * Then hold `DAI Reset` (`I2S Reset` at old versions) button and press for
   brief moment `Reset` button.
 * Release `DAI Reset` button. Screen back light should be off and your PC
   should recognize new device.
 * Windows probably does not know device, so you have to install drivers
   manually. Luckily developers at Atmel think about this. Drivers are part of
   Flip application (usb folder).
 * Check path variable in `update_fw_windows.bat` file.
   Now you can run script `update_fw_windows.bat`. Script should do all
   annoying routines.

### Updating by DFU programmer
 * This tool is usually also available for Windows. Just use Google and be sure
   you are downloaded at least version `0.7.2`.
 * It is recommended to unpack DFU programmer to
   `C:\Program Files\dfu-programmer-win\`
   So you can use default script without modifying it. Anyway if you put it to
   different folder, just be sure you changed path in
   `update_fw_windows_dfu-programmer.bat`
 * Connect device to PC.
 * Then hold `DAI Reset` (`I2S Reset` at old versions) button and press for
   brief moment `Reset` button.
 * Release `DAI Reset` button. Screen back light should be off and your PC
   should recognize new device. 
 * For the first time Windows probably did not recognize bootloader. So you
   have to manually point to drivers. But do not be afraid. Drivers are part
   of DFU programmer package. So install these drivers if it is necessary.
 * Then just run `update_fw_windows_dfu-programmer.bat` script and if necessary
   change paths.

### Using JTAG programmer
 * Allow easy debugging, but reprogramming by JTAG can cause erasing whole
   flash memory (include bootloader!!!). In FW/Release there are .bat scripts
   for some most famous Atmel programmers, so you can use/modify them.
 * Anyway, this method is NOT RECOMMENDED for non-developers.
      
# Restoring bootloader
 * As mentioned above, if you use JTAG programmer, bootloader can be eased.
   Of course Atmel provide many documents about this, but they are very old
   and refer to old tools, which are not available. Furthermore their
   bootloader image files did not worked for me (for unknown reason).
   Long story short: there is prepared script which recover bootloader.
   Surprisingly it is located in "bootloader" folder.
 * So far is tested only AVR dragon, but feel free to modify script for other
   programmers. It is not so difficult :)
 * Just be sure, that paths in `dragon_upload_bootloader.bat` are correct.
 * Run `dragon_upload_bootloader.bat`
