/* -*- mode: c; tab-width: 4; c-basic-offset: 4 -*- */
#ifndef _SDRwdgt_H_
#define _SDRwdgt_H_

 /*
 * Additions and Modifications to ATMEL AVR32-SoftwareFramework-AT32UC3 are:
 *
 * Copyright (C) Alex Lee
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "compiler.h"

#ifdef __AVR32_ABI_COMPILER__ // Automatically defined when compiling for AVR32, not when assembling.
#include "led.h"
#endif  // __AVR32_ABI_COMPILER__


/*! \name Oscillator Definitions
 */
//! @{

// RCOsc has no custom calibration by default. Set the following definition to
// the appropriate value if a custom RCOsc calibration has been applied to your
// part.
//#define FRCOSC          AVR32_PM_RCOSC_FREQUENCY              //!< RCOsc frequency: Hz.

#define FOSC32          32768                                 //!< Osc32 frequency: Hz.
#define OSC32_STARTUP   AVR32_PM_OSCCTRL32_STARTUP_8192_RCOSC //!< Osc32 startup time: RCOsc periods.

#define FOSC0           12000000                              //!< Osc0 frequency: Hz.
#define OSC0_STARTUP    AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC  //!< Osc0 startup time: RCOsc periods.

// Osc1 crystal is not mounted by default. Set the following definitions to the
// appropriate values if a custom Osc1 crystal is mounted on your board.
#define FOSC1           12288000                              //!< Osc1 frequency: Hz.
// Now in brd_driver_hw_*.c
//#define OSC1_STARTUP    AVR32_PM_OSCCTRL1_STARTUP_2048_RCOSC  //!< Osc1 startup time: RCOsc periods.

//! @}
/*! \name System Clock Frequencies
 */
//! @{
#define FMCK_HZ                       FOSC0
#define FCPU_HZ                       66000000
#define FHSB_HZ                       33000000
#define FPBB_HZ                       33000000
//#define FPBA_HZ                       33000000
#define FPBA_HZ                       66000000
//! @}


/*! \name USB Definitions
 */
//! @{

//! Multiplexed pin used for USB_ID: AVR32_USBB_USB_ID_x_x.
//! To be selected according to the AVR32_USBB_USB_ID_x_x_PIN and
//! AVR32_USBB_USB_ID_x_x_FUNCTION definitions from <avr32/uc3axxxx.h>.
#define USB_ID                      AVR32_USBB_USB_ID_0_3               // pin 124

//! Multiplexed pin used for USB_VBOF: AVR32_USBB_USB_VBOF_x_x.
//! To be selected according to the AVR32_USBB_USB_VBOF_x_x_PIN and
//! AVR32_USBB_USB_VBOF_x_x_FUNCTION definitions from <avr32/uc3axxxx.h>.
#define USB_VBOF                    AVR32_USBB_USB_VBOF_0_3             // pin 127

//! Active level of the USB_VBOF output pin.
#define USB_VBOF_ACTIVE_LEVEL       LOW


//! Number of LEDs.
#define LED_COUNT   4

/**
 * \name GPIO Connections of LEDs
 * @{
 */
#define LED0_GPIO       AVR32_PIN_PX20
#define LED1_GPIO       AVR32_PIN_PX46
#define LED2_GPIO       AVR32_PIN_PX50
#define LED3_GPIO       AVR32_PIN_PX57
//!@}

/*! \name Signals for ADC
 */
//! @{

// NOTE:: need to work on these pin assignments
/* [Martin] Some need to be defined because of UAC2, but in future this
 * definition will be removed to brd_driver_*.h file.
 */
///\todo [Martin] Remove commented definition when not needed
//#define AK5394_FSYNC                     AVR32_PIN_PX26    // with Jumper 1-2 in J302
#define AK5394_LRCK                      AVR32_PIN_PX36    // with Jumper 3-4 in J302
//#define AK5394_LRCK_IN                   AVR32_PIN_PX26    // with Jumper 2-3 in J302
//#define AK5394_SDATA                     AVR32_PIN_PX25
//#define AK5394_SCLK                      AVR32_PIN_PX28    // trace on board to PX34  ??
//#define AK5394_AD_MCLK                   AVR32_PIN_PC04    // clock from A/D board

//! @}


/*! \name USART Settings for the Sonochan mkII boards
 */
//! @{

#define USART               (&AVR32_USART1)
#define USART_RX_PIN        AVR32_USART1_RXD_0_2_PIN
#define USART_RX_FUNCTION   AVR32_USART1_RXD_0_2_FUNCTION
#define USART_TX_PIN        AVR32_USART1_TXD_0_2_PIN
#define USART_TX_FUNCTION   AVR32_USART1_TXD_0_2_FUNCTION
#define USART_CLOCK_MASK    AVR32_USART1_CLK_PBA
//! @}


// Inhale list of required modules
#include "SDR-Widget.h"

#endif  // _SDRwdgt_H_
