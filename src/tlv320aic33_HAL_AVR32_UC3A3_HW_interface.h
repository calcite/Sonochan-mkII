/**
 * \file
 *
 * \brief Hardware Abstraction Layer for codec TLV320AIC33 running on AVR32\n
 *  UC3A3 architecture
 *
 * Contains basic low level functions which depend on user architecture and\n
 * hardware. Thanks to this layer is possible use common higher level driver\n
 * and apply it thru many architectures and devices. Also this allow easy\n
 * updates independent to higher layer (new features and so on).
 *
 * Created:  02.04.2014\n
 * Modified: 23.04.2014
 *
 * \version 0.1
 * \author Martin Stejskal
 */


#ifndef _TLV320AIC33_HAL_AVR32_UC3A3_HW_INTERFACE_H_
#define _TLV320AIC33_HAL_AVR32_UC3A3_HW_INTERFACE_H_
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
 * Example: TLV320AIC33_TWI_DEVICE (&AVR32_TWIM0)
 */
#define TLV320AIC33_TWI_DEVICE         (&AVR32_TWIM0)

/**
 * \brief Clock speed in Hz
 *
 * User should define at least approximately speed of clock on which is TWI\n
 * module connected. This value is used to calculate correct values in\n
 * registers. After reset, MCU use slow oscillator, which in case UC3A3256\n
 * runs at 115 kHz. If your clock source is variable, then is recommended\n
 * set this constant to maximum used frequency. Anyway, if you want use\n
 * maximum possible speed, just set TLV320AIC33_TWI_BAUDRATE to 0 and\n
 * preprocessor do the rest :)\n
 * Example: TLV320AIC33_TWI_CLK_SOURCE 115000UL
 */
#define TLV320AIC33_TWI_CLK_SOURCE     33000000UL

/**
 * \brief TWI address of TLV320AIC33
 *
 * Because this depend on hardware settings, it is up to user to set correct\n
 * address.\n
 * \note Depend on...
 * \todo Complete doc
 */
#define TLV320AIC33_TWI_ADDRESS        0b0011000
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
 * Example: TLV320AIC33_TWI_SDA_PIN AVR32_TWIMS0_TWD_0_0_PIN
 */
#define TLV320AIC33_TWI_SDA_PIN         AVR32_TWIMS0_TWD_0_0_PIN

/**
 * \brief Function for TWI data signal
 *
 * Example: TLV320AIC33_TWI_SDA_FUNCTION AVR32_TWIMS0_TWD_0_0_FUNCTION
 */
#define TLV320AIC33_TWI_SDA_FUNCTION    AVR32_TWIMS0_TWD_0_0_FUNCTION

/**
 * \brief Pin for TWI clock signal
 *
 * Example: TLV320AIC33_TWI_SCL_PIN AVR32_TWIMS0_TWCK_0_0_PIN
 */
#define TLV320AIC33_TWI_SCL_PIN         AVR32_TWIMS0_TWCK_0_0_PIN

/**
 * \brief Function for TWI clock signal
 *
 * Example: TLV320AIC33_TWI_SCL_FUNCTION AVR32_TWIMS0_TWCK_0_0_FUNCTION
 */
#define TLV320AIC33_TWI_SCL_FUNCTION    AVR32_TWIMS0_TWCK_0_0_FUNCTION
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
 * Example: TLV320AIC33_SPI_BAUDRATE 100000UL
 */
#define TLV320AIC33_TWI_BAUDRATE        0

//============================| Structures, enums |============================
/**
 * \brief Enumeration of states, which can function returns
 *
 * Generally if function return non-zero value then no problem occurs. This\n
 * enum also use TLV320AIC33 driver (higher layer of this driver).
 */
typedef enum{
  TLV320AIC33_ERROR = -1,       //!< Unspecified error
  TLV320AIC33_OK = 0,           //!< All OK
  TLV320AIC33_ERROR_TWI_TIMEOUT = 1,/**!< Timeout occurs - when waiting for\n
                                       * complete TX data
                                       */
  TLV320AIC33_ERROR_TWI_INVALID_PARAMETER = 2,/**!< Invalid parameter when
                                            * configure TWI interface
                                            */
  TLV320AIC33_ERROR_TWI_NACK = 3      //!< When NACK occurs
} TLV320AIC33_status_t;

//================| Some checks and preprocessor calculations |================
///\todo Check device address (1 of 4)

// Test address - can not be higher than 7bits
#if TLV320AIC33_TWI_ADDRESS > 127
#error "TLV320AIC33_TWI_ADDRESS can not be higher than 127, because\
  address in TWI is 7 bit long. Please set correct value."
#endif

// Test if user want auto maximum possible speed
#if TLV320AIC33_TWI_BAUDRATE == 0
#undef TLV320AIC33_TWI_BAUDRATE
  // Check if clock frequency is lower than 200 kHz (baudrate *2)
#if TLV320AIC33_TWI_CLK_SOURCE < (200000UL)
  // Set maximum possible baudrate at this sloooow clock
  #define TLV320AIC33_TWI_BAUDRATE     (TLV320AIC33_TWI_CLK_SOURCE/2)
#else
  // Else we got clock fast enough to get maximum speed
#define TLV320AIC33_TWI_BAUDRATE     100000UL
#endif

#endif

// Check baudrate for too high value
#if TLV320AIC33_TWI_BAUDRATE > (100000UL)
#warning "TLV320AIC33_TWI_BAUDRATE is higher than 100 000, so communication with\
  PLL may fail!"
#endif

// Check baudrate for too low value
#if TLV320AIC33_TWI_BAUDRATE < (10000UL)
#warning "TLV320AIC33_TWI_BAUDRATE is lower than 10 000. This may cause, that\
  performance of whole application will be low! Please consider to use higher\
  speed of TWI."
#endif

// Check buaudrate if is lower then TLV320AIC33_CLK_SOURCE
#if TLV320AIC33_TWI_BAUDRATE > (TLV320AIC33_TWI_CLK_SOURCE/2)
#error "TLV320AIC33_TWI_BAUDRATE can not be higher than (input clock/2) for TWI\
  module. Please set TLV320AIC33_TWI_BAUDRATE to lower value, or set it to 0 and\
  preprocessor try to set maximum possible speed."
#endif


// Calculate prescaller
#if (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/2)<256
#define TLV320AIC33_TWI_CKDIV        0
#define TLV320AIC33_TWI_CLDIV        (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/2)

#elif (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/4)<256
#define TLV320AIC33_TWI_CKDIV        1
#define TLV320AIC33_TWI_CLDIV        (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/4)

#elif (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/8)<256
#define TLV320AIC33_TWI_CKDIV        2
#define TLV320AIC33_TWI_CLDIV        (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/8)

#elif (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/16)<256
#define TLV320AIC33_TWI_CKDIV        3
#define TLV320AIC33_TWI_CLDIV        (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/16)

#elif (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/32)<256
#define TLV320AIC33_TWI_CKDIV        4
#define TLV320AIC33_TWI_CLDIV        (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/32)

#elif (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/64)<256
#define TLV320AIC33_TWI_CKDIV        5
#define TLV320AIC33_TWI_CLDIV        (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/64)

#elif (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/128)<256
#define TLV320AIC33_TWI_CKDIV        6
#define TLV320AIC33_TWI_CLDIV        (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/128)

#elif (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/256)<256
#define TLV320AIC33_TWI_CKDIV        7
#define TLV320AIC33_TWI_CLDIV        (TLV320AIC33_TWI_CLK_SOURCE/TLV320AIC33_TWI_BAUDRATE/256)

#else
  // When maximum division is not enough
#error "TWI baudrate can not be set this low. Please set\
  TLV320AIC33_TWI_BAUDRATE to higher value, or set it  to 0 and\
  preprocessor try set maximum possible speed."
#endif


//================================| Functions |================================
TLV320AIC33_status_t tlv320aic33_HAL_init(void);

TLV320AIC33_status_t tlv320aic33_HAL_write_data(
    uint8_t i_register_number,
    uint8_t i_data);

TLV320AIC33_status_t tlv320aic33_HAL_read_data(
    uint8_t i_register_number,
    uint8_t *p_data);



#endif
