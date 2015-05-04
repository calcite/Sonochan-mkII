@echo off
REM This script should update firmware through DFU
REM Path is little bit modifed.
REM Because every user can have different path to binaries, this is the
REM easy way to solve problem with missing command.
REM Just change it to your needs.  

REM Path to batchisp
SET PATH_TO_BATCHISP=c:\Program Files\Atmel\Flip 3.4.7\bin\
REM And because at 64 bit windows is everything up side down, we also add this to path
SET PATH_TO_BATCHISP=%PATH_TO_BATCHISP%;c:\Program Files (x86)\Atmel\Flip 3.4.7\bin\

REM DO NOT TOUCH!
SET PATH=%PATH_TO_BATCHISP%;%PATH%
batchisp.exe -hardware usb -device at32uc3a3256 -operation erase f memory flash blankcheck loadbuffer Release\Sonochan_mkII.elf program verify start reset 0
