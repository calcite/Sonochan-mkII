/*
Sonochan Mk.II Firmware.
Copyright (C) 2013-2015 Martin Stejskal,
ALPS Electric Czech, s.r.o.,
ALPS Electric Co., Ltd.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

Contact information:
Martin Stejskal <martin.stejskal@jp.alps.com>,
Josef Nevrly <jnevrly@alps.cz>
ALPS Electric Czech, s.r.o., Sebranice 240, 679 31 Sebranice u Boskovic
Czech Republic
*/
/**
 * \file
 *
 * \brief Hardware Abstraction Layer for PLL CS2200 running on AVR32 UC3A3\n
 * architecture
 *
 * Contains basic low level functions which depend on user architecture and\n
 * hardware. Thanks to this layer is possible use common higher level driver\n
 * and apply it thru many architectures and devices. Also this allow easy\n
 * updates independent to higher layer (new features and so on).
 *
 * Created:  10.03.2014\n
 * Modified: 12.03.2014
 *
 * \version 0.1
 * \author Martin Stejskal
 */

#ifndef _CS2200_HAL_AVR32_UC3A3_HW_INTERFACE_H_
#define _CS2200_HAL_AVR32_UC3A3_HW_INTERFACE_H_
//===========================| Included libraries |============================
// Basic library (I/O)
#include <avr32/io.h>

// Data types
#include <inttypes.h>

// GPIO operations
///\todo Try to remove this dependency
#include "gpio.h"

//==============================| User settings |==============================
/**
 * \name Basic settings for TWI
 *
 * @{
 */

/**
 * \brief TWI interface
 *
 * Aka (*((volatile avr32_twi_t*)AVR32_TWIM0_ADDRESS)) or (&AVR32_TWIM0)\n
 * Example: CS2200_TWI_DEVICE (&AVR32_TWIM0)
 */
#define CS2200_TWI_DEVICE         (&AVR32_TWIM0)

/**
 * \brief Clock speed in Hz
 *
 * User should define at least approximately speed of clock on which is TWI\n
 * module connected. This value is used to calculate correct values in\n
 * registers. After reset, MCU use slow oscillator, which in case UC3A3256\n
 * runs at 115 kHz. If your clock source is variable, then is recommended\n
 * set this constant to maximum used frequency. Anyway, if you want use\n
 * maximum possible speed, just set CS2200_TWI_BAUDRATE to 0 and\n
 * preprocessor do the rest :)\n
 * Example: CS2200_TWI_CLK_SOURCE 115000UL
 */
#define CS2200_TWI_CLK_SOURCE     33000000UL

/**
 * \brief TWI address of CS2200
 *
 * Because this depend on hardware settings, it is up to user to set correct\n
 * address.\n
 * \note When AD0/CS=0 -> address is 0b1001110, else 0b1001111
 */
#define CS2200_TWI_ADDRESS        0b1001110
/**
 * @}
 */

/**
 * \brief Definition of TWI (I2C) pins
 *
 * There are defined pins and their functions
 * @{
 */
/**
 * \brief Pin for TWI data signal
 *
 * Example: CS2200_TWI_SDA_PIN AVR32_TWIMS0_TWD_0_0_PIN
 */
#define CS2200_TWI_SDA_PIN         AVR32_TWIMS0_TWD_0_0_PIN

/**
 * \brief Function for TWI data signal
 *
 * Example: CS2200_TWI_SDA_FUNCTION AVR32_TWIMS0_TWD_0_0_FUNCTION
 */
#define CS2200_TWI_SDA_FUNCTION    AVR32_TWIMS0_TWD_0_0_FUNCTION

/**
 * \brief Pin for TWI clock signal
 *
 * Example: CS2200_TWI_SCL_PIN AVR32_TWIMS0_TWCK_0_0_PIN
 */
#define CS2200_TWI_SCL_PIN         AVR32_TWIMS0_TWCK_0_0_PIN

/**
 * \brief Function for TWI clock signal
 *
 * Example: CS2200_TWI_SCL_FUNCTION AVR32_TWIMS0_TWCK_0_0_FUNCTION
 */
#define CS2200_TWI_SCL_FUNCTION    AVR32_TWIMS0_TWCK_0_0_FUNCTION
/**
 * @}
 */



/** \name Advanced settings TWI interface
 * @{
 */
/**
 * \brief Baudrate in Hz - in real should not exceed 100 kb/s
 *
 * Too low speed decrease whole application performance.\n
 * For \b maximum \b performance is \b recommended to set this value \b to\n
 * \b 0. Library itself will try to set maximum possible baudrate.\n
 * However it can not be ideal solution. When any other devices are connected\n
 * to this TWI bus, they may not support this speed! So please check maximum\n
 * baudrate at other devices connected to bus.
 * Example: CS2200_SPI_BAUDRATE 100000UL
 */
#define CS2200_TWI_BAUDRATE        0

//============================| Structures, enums |============================
/**
 * \brief Enumeration of states, which can function returns
 *
 * Generally if function return non-zero value then no problem occurs. This\n
 * enum also use CS2200 driver (higher layer of this driver).
 */
typedef enum{
  CS2200_ERROR = -1,       //!< Unspecified error
  CS2200_OK = 0,           //!< All OK
  CS2200_ERROR_TWI_TIMEOUT = 1,/**!< Timeout occurs - when waiting for\n
                                       * complete TX data
                                       */
  CS2200_ERROR_TWI_INVALID_PARAMETER = 2,/**!< Invalid parameter when
                                            * configure TWI interface
                                            */
  CS2200_ERROR_TWI_NACK = 3      //!< When NACK occurs
} cs2200_status_t;


//================| Some checks and preprocessor calculations |================
// Test TLV address
#if (CS2200_TWI_ADDRESS != 0b1001110) &&\
    (CS2200_TWI_ADDRESS != 0b1001111)
#warning "CS2200_TWI_ADDRESS is not set to one of known settings. It is\
  highly possible that communication with PLL fail!"
#endif

// Test address - can not be higher than 7bits
#if CS2200_TWI_ADDRESS > 127
#error "CS2200_TWI_ADDRESS can not be higher than 127, because\
  address in TWI is 7 bit long. Please set correct value."
#endif

// Test if user want auto maximum possible speed
#if CS2200_TWI_BAUDRATE == 0
#undef CS2200_TWI_BAUDRATE
  // Check if clock frequency is lower than 200 kHz (baudrate *2)
#if CS2200_TWI_CLK_SOURCE < (200000UL)
  // Set maximum possible baudrate at this sloooow clock
  #define CS2200_TWI_BAUDRATE     (CS2200_TWI_CLK_SOURCE/2)
#else
  // Else we got clock fast enough to get maximum speed
#define CS2200_TWI_BAUDRATE     100000UL
#endif

#endif

// Check baudrate for too high value
#if CS2200_TWI_BAUDRATE > (100000UL)
#warning "CS2200_TWI_BAUDRATE is higher than 100 000, so communication with\
  PLL may fail!"
#endif

// Check baudrate for too low value
#if CS2200_TWI_BAUDRATE < (10000UL)
#warning "CS2200_TWI_BAUDRATE is lower than 10 000. This may cause, that\
  performance of whole application will be low! Please consider to use higher\
  speed of TWI."
#endif

// Check buaudrate if is lower then CS2200_CLK_SOURCE
#if CS2200_TWI_BAUDRATE > (CS2200_TWI_CLK_SOURCE/2)
#error "CS2200_TWI_BAUDRATE can not be higher than (input clock/2) for TWI\
  module. Please set CS2200_TWI_BAUDRATE to lower value, or set it to 0 and\
  preprocessor try to set maximum possible speed."
#endif


// Calculate prescaller
#if (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/2)<256
#define CS2200_TWI_CKDIV        0
#define CS2200_TWI_CLDIV        (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/2)

#elif (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/4)<256
#define CS2200_TWI_CKDIV        1
#define CS2200_TWI_CLDIV        (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/4)

#elif (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/8)<256
#define CS2200_TWI_CKDIV        2
#define CS2200_TWI_CLDIV        (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/8)

#elif (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/16)<256
#define CS2200_TWI_CKDIV        3
#define CS2200_TWI_CLDIV        (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/16)

#elif (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/32)<256
#define CS2200_TWI_CKDIV        4
#define CS2200_TWI_CLDIV        (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/32)

#elif (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/64)<256
#define CS2200_TWI_CKDIV        5
#define CS2200_TWI_CLDIV        (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/64)

#elif (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/128)<256
#define CS2200_TWI_CKDIV        6
#define CS2200_TWI_CLDIV        (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/128)

#elif (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/256)<256
#define CS2200_TWI_CKDIV        7
#define CS2200_TWI_CLDIV        (CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE/256)

#else
  // When maximum division is not enough
#error "TWI baudrate can not be set this low. Please set\
  CS2200_TWI_BAUDRATE to higher value, or set it  to 0 and\
  preprocessor try set maximum possible speed."
#endif


//================================| Functions |================================
cs2200_status_t cs2200_HAL_init(void);

cs2200_status_t cs2200_HAL_write_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes);

cs2200_status_t cs2200_HAL_read_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes);

#endif
