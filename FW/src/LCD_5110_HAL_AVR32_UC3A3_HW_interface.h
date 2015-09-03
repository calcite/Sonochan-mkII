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
 * \brief Hardware Abstraction Layer for AVR32 and LCD used in nokia 5110.\n
 * Use hardware SPI module on AVR.
 *
 * Contains basic low level functions which depend on user architecture and\n
 * hardware. Thanks to this layer is possible use common higher level driver\n
 * and apply it thru many architectures and devices. Also this allow easy\n
 * updates independent to higher layer (new features and so on).
 *
 * Created:  05.05.2014\n
 * Modified: 05.05.2014
 *
 * \version 0.1
 * \author Martin Stejskal
 */

#ifndef _LCD_5110_HAL_AVR32_UC3A3_HW_INTERFACE_H_
#define _LCD_5110_HAL_AVR32_UC3A3_HW_INTERFACE_H_

//===========================| Included libraries |============================
// Basic library (I/O)
#include <avr32/io.h>

// Data types
#include <inttypes.h>

// Some basic operations with I/O
///\todo Try remove this dependency
#include "gpio.h"

//==============================| User settings |==============================
/** \name Basic settings SPI interface
 * @{
 */

/**
 * \brief Enable (1) or disable (0) Chip Select support
 *
 * When Chip Select support is enabled (set to 1), then CS signal is set by\n
 * hardware. Else CS signal on target LCD must be connected to GND and SPI\n
 * device can be used only for this display.\n
 * Example:
 * \code
 * #define LCD_5110_CS_support 1        // Enable chip select support
 * \endcode
 */
#define LCD_5110_CS_support        1

/**
 * \brief Enable (1) or disable (0) Reset pin support
 *
 * When power supply is connected to LCD there should be some time reset pin\n
 * in low and then is possible to set reset pin back to high. After that\n
 * it is possible do initialization process. This process can be driven by\n
 * MCU or externally (for example by RC components). When reset is done by\n
 * external components, software just wait some time when initialization\n
 * function is called and then do initialize routine without any checks.\n
 * So it is up to external circuit to do reset before initialization function\n
 * is called. Or user can simply enable reset support and reset LCD is done\n
 * when needed. But it request one more wire to LCD module. Recommended is\n
 * enabled this feature.\n
 * Example:
 * \code
 * #define LCD_5110_RESET_support 1     // Enable support for reset
 * \endcode
 */
#define LCD_5110_RESET_support     1

/**
 * \brief Enable (1) or disable (0) support for backlight
 *
 * In some low power applications may be useful turn off backlight. If this\n
 * option is enabled user can simply control backlight thru this driver.\n
 * Example:
 * \code
 * #define LCD_5110_LIGHT_support 0     // Disable support for backlight
 * \endcode
 */
#define LCD_5110_LIGHT_support     1

/**
 * \brief SPI interface
 *
 * Aka (*((volatile avr32_spi_t*)AVR32_SPI0_ADDRESS)) or (&AVR32_SPI0)\n
 * Example: LCD_5110_SPI_DEVICE (&AVR32_SPI0)
 */
#define LCD_5110_SPI_DEVICE        (&AVR32_SPI0)

/**
 * \brief Chip/Slave select pin
 *
 * Should be defined even if LCD_5110_CS_support is disabled. If You doubt, just\n
 * set it to 0. Anyway please remember: inside SPI module is CS set according\n
 * to this settings!\n
 * Example:
 * \code
 * #define LCD_5110_SPI_CHIP_SELECT 1   // Chip select number: 1
 * \endcode
 */
#define LCD_5110_SPI_CHIP_SELECT   0

/**
 * \brief Clock speed in Hz
 *
 * User should define at least approximately speed of clock on which is SPI\n
 * module connected. This value is used to calculate correct values in\n
 * registers. After reset, MCU use slow oscillator, which in case UC3A3256\n
 * runs at 115 kHz. If your clock source is variable, then is recommended\n
 * set this constant to maximum used frequency. Anyway, if you want use\n
 * maximum possible speed, just set LCD_5110_SPI_BAUDRATE to 0 and\n
 * preprocessor do the rest :)\n
 * Example:
 * \code
 * #define LCD_5110_SPI_CLK_SOURCE 115000UL
 * \endcode
 */
#define LCD_5110_SPI_CLK_SOURCE    66000000UL

//! @}


/**
 * \name Definition of SPI pins
 *
 * There are defined pins and their functions
 * @{
 */
/**
 * \brief SCK pin - must be defined!
 *
 * Example:
 * \code
 * #define LCD_5110_SPI_SCK_PIN AVR32_SPI0_SCK_0_0_PIN
 * \endcode
 */
#define LCD_5110_SPI_SCK_PIN               AVR32_SPI0_SCK_0_0_PIN

/**
 * \brief SCK function - must be defined!
 *
 * Example:
 * \code
 * #define LCD_5110_SPI_SCK_FUNCTION AVR32_SPI0_SCK_0_0_FUNCTION
 * \endcode
 */

#define LCD_5110_SPI_SCK_FUNCTION          AVR32_SPI0_SCK_0_0_FUNCTION

/**
 * \brief MISO pin - must be defined!
 *
 * Example:
 * \code
 * #define LCD_5110_SPI_MOSI_PIN AVR32_SPI0_MOSI_0_0_PIN
 * \endcode
 */
#define LCD_5110_SPI_MOSI_PIN              AVR32_SPI0_MOSI_0_0_PIN

/**
 * \brief MISO function - must be defined!
 *
 * Example:
 * \code
 * #define LCD_5110_SPI_MOSI_FUNCTION AVR32_SPI0_MOSI_0_0_FUNCTION
 * \endcode
 */
#define LCD_5110_SPI_MOSI_FUNCTION         AVR32_SPI0_MOSI_0_0_FUNCTION

/**
 * \brief Chip select pin
 *
 * Should be at least defined. In real pin will not be connect to GPIO.\n
 * This also\n
 * depend which CS user set! So please make sure that LCD_5110_SPI_CHIP_SELECT\n
 * match this CS!\n
 * Example:
 * \code
 * #define LCD_5110_SPI_CS_PIN AVR32_SPI0_NPCS_2_0_PIN
 * \endcode
 */
#define LCD_5110_SPI_CS_PIN                AVR32_SPI0_NPCS_0_0_PIN

/**
 * \brief Chip select function (CS function)
 *
 * Should be at least defined. In real function will not be used. This also\n
 * depend which CS user set! So please make sure that LCD_5110_SPI_CHIP_SELECT\n
 * match this CS!\n
 * Example:
 * \code
 * #define LCD_5110_SPI_CS_FUNCTION AVR32_SPI0_NPCS_2_0_FUNCTION
 * \endcode
 */
#define LCD_5110_SPI_CS_FUNCTION           AVR32_SPI0_NPCS_0_0_FUNCTION
/// @}




/** \name Another connections for LCD 5110 module
 * @{
 */

/**
 * \brief Data/!Command used pin - must be defined!
 *
 * Example:
 * \code
 * #define LCD_5110_SPI_DC_PIN AVR32_PIN_PB30
 * \endcode
 */
#define LCD_5110_SPI_DC_PIN                AVR32_PIN_PA00

/**
 * \brief Reset pin - active LOW
 *
 * When LCD_5110_RESET_support is enabled, this must be defined!
 * Example:
 * \code
 * #define LCD_5110_SPI_RST_PIN AVR32_PIN_PB31
 * \endcode
 */
#define LCD_5110_RST_PIN                   AVR32_PIN_PA05

/**
 * \brief Backlight pin - active LOW
 *
 * When LCD_5110_LIGHT_support is enabled, this must be defined!
 * Example:
 * \code
 * #define LCD_5110_BACKLIGHT_PIN AVR32_PIN_PX21
 * \endcode
 */
#define LCD_5110_LIGHT_PIN                 AVR32_PIN_PX14
//! @}









/** \name Advanced settings SPI interface
 * @{
 */
/**
 * \brief Baudrate in Hz - in real should not exceed 4 Mbit/s
 *
 * It is possible use higher baudrate than 4 Mbit/s, but this depend on\n
 * used module and designed PCB and connection to module. Also too low speed\n
 * decrease whole application performance.\n
 * For \b maximum \b performance is \b recommended to set this value \b to\n
 * \b 0. Library itself will try to set maximum possible baudrate.\n
 * Example:
 * \code
 * #define LCD_5110_SPI_BAUDRATE 4000000UL
 * \endcode
 */
#define LCD_5110_SPI_BAUDRATE      0UL

/**
 * \brief Define timeout of SPI
 *
 * Well, sometimes can be SPI interface busy, so there can be limit for that\n
 * software will wait. After that return something like "timeout error" and\n
 * it is up to higher layers to deal with it. Anyway, here user can set own\n
 * delay. Delay is approximately in CPU cycles, so it depends.... However if\n
 * you are not sure, just set this value to 0 and preprocessor do the rest.\n
 * Example:
 * \code
 * #define LCD_5110_SPI_TIMEOUT 7500UL
 * \endcode
 */
#define LCD_5110_SPI_TIMEOUT               0UL

/**
 * \brief Delay between two chip select
 *
 * When one SPI module is shared between many devices there should be some\n
 * time between two chip select. For example: deselect device1, time delay\n
 * (because of logic in devices), select device2. Anyway, if SPI module\n
 * handle just one device (this LCD module) then this value can be set to 0.\n
 * Else there should be some delay. Delay is in SPI clock periods.\n
 * Example:
 * \code
 * #define LCD_5110_SPI_DELAY_BETWEEN_CS 0
 * \endcode
 */
#define LCD_5110_SPI_DELAY_BETWEEN_CS      0

/**
 * \brief Delay between SCK and CS at start condition
 *
 * CS enable -> this delay -> SCK start\n
 * This delay should be set to 0 for optimal performance. Anyway in some\n
 * special cases can be used different value if needed.\n
 * Example:
 * \code
 * #define LCD_5110_SPI_DELAY_SCK_START 0
 * \endcode
 */
#define LCD_5110_SPI_DELAY_SCK_START       0

/**
 * \brief Delay between CS and SCK at stop condition
 *
 * SCK stop -> this delay -> CS disable\n
 * This delay should be set to 0 for optimal performance. Anyway in some\n
 * special cases can be used different value if needed.\n
 * Example:
 * \code
 * #define LCD_5110_SPI_DELAY_CS_STOP 0
 * \endcode
 */
#define LCD_5110_SPI_DELAY_CS_STOP         0
//! @}



/**
 * \name Other advanced settings that is not recommended to change
 * @{
 */
/**
 * \brief Number of cycles when reset support is disabled
 *
 * When LCD_5110_RESET_support is disabled MCU do not know if reset was done\n
 * or not. So wait. This number define how many cycles must MCU do in loop.\n
 * So this value should depend also on used clock. It is recommended to\n
 * let preprocessor calculate this value (set let there 0).\n
 * Anyway if You like hardcore You can set it manually.\n
 * Example: (LCD_5110_SPI_CLK_SOURCE/100)
 */
#define LCD_5110_RESET_NUM_OF_CYCLES    0


//! @}


//============================| Structures, enums |============================

/**
 * \brief Enumeration of states, which can function returns
 *
 * Generally if function return non-zero value then no problem occurs. This\n
 * enum also use LCD_5110 driver (higher layer of this driver).
 */
typedef enum{
  LCD_5110_ERROR = -1,       //!< Unspecified error
  LCD_5110_OK = 0,           //!< All OK
  LCD_5110_ERROR_TIMEOUT = 1,//!< Timeout occurs
  LCD_5110_ERROR_SPI_INVALID_PARAMETER = 2  /**<//!< LCD_5110_ERROR_SPI_INVALID_PARAMETER
  * Invalid parameter for SPI driver
  */
} e_lcd_5110_status;



//=================================| Macros |==================================
#define LCD_5110_IO_LOW_simple(PIN)                                     \
    gpio_port =                                                         \
          &AVR32_GPIO.port[PIN >> 5];                                   \
    gpio_port->ovrc  = 1 << (PIN & 0x1F);                               \
    gpio_port->oders = 1 << (PIN & 0x1F);                               \
    gpio_port->gpers = 1 << (PIN & 0x1F)


#define LCD_5110_IO_LOW(PIN)                    LCD_5110_IO_LOW_simple(PIN)


#define LCD_5110_IO_HIGH_simple(PIN)                                    \
    gpio_port =                                                         \
          &AVR32_GPIO.port[PIN >> 5];                                   \
    gpio_port->ovrs  = 1 << (PIN & 0x1F);                               \
    gpio_port->oders = 1 << (PIN & 0x1F);                               \
    gpio_port->gpers = 1 << (PIN & 0x1F)

#define LCD_5110_IO_HIGH(PIN)                   LCD_5110_IO_HIGH_simple(PIN)

//================| Some checks and preprocessor calculations |================


#if (LCD_5110_CS_support != 0) && (LCD_5110_CS_support != 1)
#error "Please enable (set 1) or disable (set 0) LCD_5110_CS_support"
#endif

#if (LCD_5110_RESET_support != 0) && (LCD_5110_RESET_support != 1)
#error "Please enable (set 1) or disable (set 0) LCD_5110_RESET_support"
#endif

#if (LCD_5110_LIGHT_support != 0) && (LCD_5110_LIGHT_support != 1)
#error "Please enable (set 1) or disable (set 0) LCD_5110_LIGHT_support"
#endif

// If user let preprocessor define number of cycles
#if LCD_5110_RESET_NUM_OF_CYCLES == 0
#undef LCD_5110_RESET_NUM_OF_CYCLES
#define LCD_5110_RESET_NUM_OF_CYCLES    (LCD_5110_SPI_CLK_SOURCE/100)
#endif

#if LCD_5110_SPI_TIMEOUT == 0
#undef LCD_5110_SPI_TIMEOUT
#define LCD_5110_SPI_TIMEOUT 10000
#endif

// Try to set maximum baudrate if it is wish of user
#if LCD_5110_SPI_BAUDRATE == 0
#undef LCD_5110_SPI_BAUDRATE
  // Test if actual clock is lower than 4 MHz
#if LCD_5110_SPI_CLK_SOURCE < 4000000
#define LCD_5110_SPI_BAUDRATE      LCD_5110_SPI_CLK_SOURCE
#else
  // If actual clock is higher than 4 MHz -> try to set maximum baudrate
#define LCD_5110_SPI_BAUDRATE      4000000UL
#endif
#endif


#if LCD_5110_SPI_BAUDRATE > LCD_5110_SPI_CLK_SOURCE
#error "SPI baudrate can not be higher than input clock for SPI module. Please\
  set LCD_5110_SPI_BAUDRATE to lower value, or set it to 0 and preprocessor\
  try set maximum possible speed."
#endif

#if (LCD_5110_SPI_CLK_SOURCE / LCD_5110_SPI_BAUDRATE) > 255
  // If calculated SCBR > 255 - can not set this low
#error "SPI baudrate can not be set this low! Please set higher baudrate or\
  lower CLK source for SPI module."
#else
  // Baudrate can be set
#define LCD_5110_SPI_SCBR          ((LCD_5110_SPI_CLK_SOURCE / LCD_5110_SPI_BAUDRATE))
#endif

// Anyway, check LCD_5110_SPI_SCBR if is 0 -> change to 1
#if LCD_5110_SPI_SCBR == 0
#undef LCD_5110_SPI_SCBR
#define LCD_5110_SPI_SCBR = 1
#warning "Value for SCBR was calculated as 0, but this is invalid value. So\
  SCBR was set to 1. Means that baudrate will be lower then defined by user."
#endif

// test LCD_5110_SPI_DELAY_BETWEEN_CS
#if LCD_5110_SPI_DELAY_BETWEEN_CS > 255
#error "Sorry, but LCD_5110_SPI_DELAY_BETWEEN_CS can not be higher than 255.\
  Please set this constant lower. If You need higher delay please consider\
  lower SPI clock source."
#endif

//================================| Functions |================================
/**
 * \brief Initialize necessary HW and do initialize routine for LCD
 *
 * Initialize SPI, GPIO and send initialization commands to LCD. This\n
 * function must be called first.
 *
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_HAL_init(void);

/**
 * \brief Send one data Byte to LCD module
 *
 * @param i_Byte Data byte which will be send to LCD module
 *
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_HAL_send_data_byte(uint8_t i_data);

/**
 * \brief Send one command Byte to LCD module
 *
 * @param i_cmd Command which will be send to LCD module
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_HAL_send_command_byte(uint8_t i_cmd);

/**
 * \brief Turn on backlight on LCD module (set pin to high)
 *
 * LED should be controlled thru transistor, so that is reason why logic is\n
 * opposite to hardware design of LCD module.
 *
 * @return LCD_5110_OK (0) if support for backlight is enabled
 */
e_lcd_5110_status LCD_5110_HAL_backlight_on(void);

/**
 * \brief Turn off backlight on LCD module (set pin to low)
 *
 * LED should be controlled thru transistor, so that is reason why logic is\n
 * opposite to hardware design of LCD module.
 *
 * @return LCD_5110_OK (0) if support for backlight is enabled
 */
e_lcd_5110_status LCD_5110_HAL_backlight_off(void);


#endif
