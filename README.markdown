# ==<| Sonochan mkII |>==
![alt tag](https://github.com/calcite/Sonochan-mkII/raw/master/DOC/photo/snchn_mkII-production.jpg)

## Description
 * Polymorph usb soundcard (USB <--> [I2S/DSP/LJF/RJF]) based on "SDR Widget"
   project
 * On board is already codec which can sniff I2S/DSP/RJF/LJF bus
  (RX or TX data)
 * For normal operation no extra drivers needed. Using standard drivers
   included in nearly all operating systems
 * Development is also possible at Windows and Linux
 * Updating FW is possible directly through USB interface (drivers are part of
   programmer SW). No external tool needed.

![alt tag](https://github.com/calcite/Sonochan-mkII/raw/master/DOC/thesis/thesis_doc_files/HW_block_scheme.jpg)


## Parameters
 * CPU: AVR32 UC3A3256
 * As clock source generator is used CS2200
 * On board is DAC TLV320AIC33 
 * Supported sampling frequencies: 8000, 11025, 16000, 22050, 24000, 32000,
   44100 and 48000 Hz
 * Supported digital audio interfaces: I2S, DSP, Left justified,
   Right justified
 * Bit precision: up to 24 bit
 * Control thru USB HID (HW bridge uniprot)
 * Device is highly configurable (master/slave, BCLK oversampling, MCLK
   oversampling, MCLK offset, word offset, word precision...)
 * Debug messages can be sent through UART interface (115200-8-N-1). This pin
   is wired out to 2.54 pin header -> easy to connect

