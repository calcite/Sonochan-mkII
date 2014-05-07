/**
 * \file
 *
 * \brief Board driver for "Sonochan mkII" HW version 03
 *
 * Allow set basic settings on board. Also support "generic driver"
 *
 * Created:  23.04.2014\n
 * Modified: 06.05.2014
 *
 * \version 0.1
 * \author  Martin Stejskal
 */

#ifndef _BRD_DRIVER_H_
#define _BRD_DRIVER_H_

//=================================| Options |=================================

/**
 * \brief Set default headphones volume in dB
 *
 * Recommended value is about -40, but it is up to you if you want destroy \n
 * your ears or headphones...
 */
#define BRD_DRV_DEFAULT_HEADPHONES_VOLUME_IN_DB         -45

/**
 * \brief Option for generic driver support
 *
 * Options: 0 (support disabled) or 1 (support enabled)
 */
#define BRD_DRV_SUPPORT_GENERIC_DRIVER          1


/**
 * \brief Option for FreeRTOS support
 *
 * Options: 0 (support disabled) or 1 (support enabled)
 */
#define BRD_DRV_SUPPORT_RTOS                    1




/**
 * \brief Option allow create ADCmutex when brd_drv_init() is called
 *
 * Mutexes is needed when in RTOS two functions share same hardware. This\n
 * option allow create mutex mutexADC, so any other functions just can use\n
 * xSemaphoreTake and xSemaphoreGive functions.\n
 * Options: 0 (disabled) or 1 (enabled).\n
 *
 * \note This option is valid only if RTOS support is enabled.
 */
#define BRD_DRV_RTOS_CREATE_MUTEX               1



/**
 * \brief Address to memory where are ADC registers
 *
 * Example: AVR32_ADC_ADDRESS or (&AVR32_ADC) - it is the same
 */
#define BDR_DRV_ADC_ADDRESS                     AVR32_ADC_ADDRESS



/**
 * \brief Clock speed for ADC module in Hz
 *
 * User should define at least approximately speed of clock on which is ADC\n
 * module connected. This value is used to calculate correct values in\n
 * registers. After reset, MCU use slow oscillator, which in case UC3A3256\n
 * runs at 115 kHz. If your clock source is variable, then is recommended\n
 * set this constant to maximum used frequency.\n
 * Example:\n
 * \code
 * #define BRD_DRV_ADC_CLK_SOURCE 115000UL
 * \endcode
 */
#define BRD_DRV_ADC_CLK_SOURCE     33000000UL



/**
 * \brief Wanted clock speed for ADC module in Hz
 *
 * Because ADC module have own prescaller, it is possible set lower frequency\n
 * for ADC and other components can run on high speed bus. Also for 10 bit\n
 * resolution prescalled CLK should not exceed 5 MHz and for 8 bit resolution\n
 * should not exceed 8 MHz.\n
 * If you unsure, just set this value to 0 and preprocessor do the rest.\n
 * Example:\n
 * \code
 * #define BRD_DRV_ADC_PRESCALLED_CLK 0UL
 * \endcode
 */
#define BRD_DRV_ADC_PRESCALLED_CLK      0



/**
 * \name Pin mapping for BRD_DRV driver
 * @{
 */
///\brief ADC volume signal pin
#define BRD_DRV_ADC_VOLUME_CONTROL_PIN          AVR32_ADC_AD_5_PIN

///\brief ADC volume signal pin function
#define BRD_DRV_ADC_VOLUME_CONTROL_FUNCTION     AVR32_ADC_AD_5_FUNCTION

///\brief Used ADC channel by volume signal
#define BRD_DRV_ADC_VOLUME_CONTROL_CHANNEL      5

///\brief ADC con voltage signal pin
#define BRD_DRV_ADC_CON_VOLTAGE_PIN             AVR32_ADC_AD_0_PIN

///\brief ADC con voltage signal pin function
#define BRD_DRV_ADC_CON_VOLTAGE_FUNCTION        AVR32_ADC_AD_0_FUNCTION

///\brief Used ADC channel by con voltage signal
#define BRD_DRV_ADC_CON_VOLTAGE_CHANNEL         0

///\brief Mute enable on MCU side
#define BRD_DRV_MUTE_EN_A_PIN                   AVR32_PIN_PB02

///\brief Reset enable on MCU side
#define BRD_DRV_RST_EN_A_PIN                    AVR32_PIN_PA27

///\brief Bit clock enable on MCU side
#define BRD_DRV_BCLK_EN_A_PIN                   AVR32_PIN_PX42

///brief Frame sync enable on MCU side
#define BRD_DRV_FS_EN_A_PIN                     AVR32_PIN_PX03

///\brief Master clock enable on MCU side
#define BRD_DRV_MCLK_EN_A_PIN                   AVR32_PIN_PX34

///\brief Mute enable on CON side
#define BRD_DRV_MUTE_EN_B_PIN                   AVR32_PIN_PB01

///\brief Reset enable on CON side
#define BRD_DRV_RST_EN_B_PIN                    AVR32_PIN_PB03

///\brief TX enable on CON side
#define BRD_DRV_TX_EN_B_PIN                     AVR32_PIN_PX58

///\brief Master clock enable on CON side
#define BRD_DRV_MCLK_EN_B_PIN                   AVR32_PIN_PX41

///\brief Bit clock enable on CON side
#define BRD_DRV_BCLK_EN_B_PIN                   AVR32_PIN_PX33

///\brief Frame sync enable on CON side
#define BRD_DRV_FS_EN_B_PIN                     AVR32_PIN_PX45

///\brief Mute button
#define BRD_DRV_MUTE_BTN_PIN                    AVR32_PIN_PX10

///\brief Reset I2S button
#define BRD_DRV_RESET_I2S_BTN_PIN               AVR32_PIN_PA25

///\brief Reset I2S signal
#define BRD_DRV_RESET_I2S_PIN                   AVR32_PIN_PX07

///\brief Mute signal
#define BRD_DRV_MUTE_PIN                        AVR32_PIN_PX38

///\brief Error indication
#define BRD_DRV_ERROR_INDICATION_PIN            AVR32_PIN_PC00

/// @}



/**
 * \brief Define when volume is updated (how much must be value on ADC\n
 * different from previous value on ADC)
 */
#define BRD_DRV_ADC_UPDATE_TRESHOLD             2

/**
 * \brief Define "period" in which will be input buttons, ADC and so on scanned
 *
 * Period is set by experimental. It depends on system load, if used RTOS,
 * RTOS priority and many other stuff.\n
 * Scan "mute", "reset I2S", ADC volume control, ADC con voltage and so on.
 */
#define BRD_DRV_BUTTON_REFRESH_PERIOD           3


//============================| Advanced settings |============================
/**
 * \brief Define sample and hold time in ADC CLK cycles
 *
 * Due to capacitance, there should be some time to sample input value and\n
 * hold it. Higher value means more accurate results, but it takes more time.\n
 * Sample and hold time (in cycles) can be calculated as:\n
 *  tsh = (BRD_DRV_ADC_SAMPLE_HOLD_TIME + 3)\n
 * If you are unsure, set this value to 5. Example:\n
 * \code
 * #define BRD_DRV_ADC_SAMPLE_HOLD_TIME 5
 * \endcode
 *
 */
#define BRD_DRV_ADC_SAMPLE_HOLD_TIME                    5



/**
 * \brief Define minimal ADC start up time in ADC CLK cycles
 *
 * Start up time can be calculated as:\n
 *  tsup = (BRD_DRV_ADC_START_UP_TIME+1)*8\n
 * If you are unsure, set this value to 5. Example:\n
 * \code
 * #define BRD_DRV_ADC_START_UP_TIME 5
 * \endcode
 */
#define BRD_DRV_ADC_START_UP_TIME                       10

//================================| Includes |=================================
// IO pins
#include <avr32/io.h>

// Basic data types
#include <inttypes.h>

// PLL control
#include "cs2200.h"

// For codec control
#include "tlv320aic33.h"

// For LCD support
#include "LCD_5110.h"

// GPIO operations
///\todo Try to remove this dependency
#include "gpio.h"

#if BRD_DRV_SUPPORT_RTOS != 0
#include "FreeRTOS.h"
#include "task.h"
#endif

//===========================| Additional includes |===========================
#if BRD_DRV_SUPPORT_GENERIC_DRIVER == 1
// If used generic driver, get actual structures
#include "generic_driver.h"
// Also say compiler, that there exist settings on flash
extern const gd_config_struct BRD_DRV_config_table_flash[];
// And metadata
extern const gd_metadata BRD_DRV_metadata;

#else
/**
 * \brief Error codes common with generic driver
 *
 * Please note, that this enum MUST be same with as enum in generic driver!
 */
#ifndef _GENERIC_DRIVER_H_
#ifndef GD_RES_CODE_DEFINED

#define GD_RES_CODE_DEFINED
typedef enum{
        GD_SUCCESS =                   0,//!< GD_SUCCESS
        GD_FAIL =                      1,//!< GD_FAIL
        GD_INCORRECT_PARAMETER =       2,//!< GD_INCORRECT_PARAMETER
        GD_INCORRECT_CMD_ID =          3,//!< GD_INCORRECT_CMD_ID
        GD_CMD_ID_NOT_EQUAL_IN_FLASH = 4,//!< GD_CMD_ID_NOT_EQUAL_IN_FLASH
        GD_INCORRECT_DEVICE_ID =       5 //!< GD_INCORRECT_DEVICE_ID
} GD_RES_CODE;
#endif
#endif

#endif
//===============================| Structures |================================



//===============================| Definitions |===============================
/**
 * \name Error messages
 *
 * This is set of error messages, which will be written to debug UART or LCD
 * @{
 */
/// Can not set isolators I/O
#define BRD_DRV_MSG_ISOLATOR_FAIL       \
  {"BRD DRV init: setting I/O for isolators failed!\n"}

/// Can not initialize LCD display
#define BRD_DRV_MSG_LCD_INIT_FAIL       \
  {"BRD DRV init: LCD initialization failed!\n"}

/// Sonochan mk II
#define BRD_DRV_MSG_SONOCHAN_MK_II      \
  {"Sonochan mk II\n > Beta\n"}

/// Can not draw logo
#define BRD_DRV_MSG_DRAW_LOGO_FAIL      \
  {"BRD DRV: Write logo failed\n"}

/// Can not initialize ADC
#define BRD_DRV_MSG_ADC_INIT_FAIL       \
  {"BRD DRV: Can not initialize internal ADC\n"}

/// Can not initialize codec
#define BRD_DRV_MSG_CODEC_INIT_FAIL     \
  {"BRD DRV: Can not initialize codec\n"}

/// Can not set save flag
#define BRD_DRV_MSG_PLL_SET_SAVE_FLAG_FAIL      \
  {"BRD DRV: Can not set save flag at PLL\n"}
/// @}

//=================================| Macros |==================================

/**
 * \brief Define lock function when support for RTOS is enabled
 *
 * If BRD_DRV_SUPPORT_RTOS is enabled, then there is defined function, that\n
 * "lock" ADC device, so any other device can not use it.\n
 * If BRD_DRV_SUPPORT_RTOS is disabled, there is defined just void macro.
 */
#if BRD_DRV_SUPPORT_RTOS != 0
  #define BRD_DRV_LOCK_ADC_MODULE_IF_RTOS               \
    xSemaphoreTake( mutexADC, portMAX_DELAY );
#else
  #define BRD_DRV_LOCK_ADC_MODULE_IF_RTOS
#endif


#if BRD_DRV_SUPPORT_RTOS != 0
  #define BRD_DRV_UNLOCK_ADC_MODULE_IF_RTOS             \
    xSemaphoreGive( mutexADC );
#else
  #define BRD_DRV_UNLOCK_ADC_MODULE_IF_RTOS
#endif


/**
 * \brief Macro allow enabled needed channel
 *
 * Example:\n
 * \code
 * BRD_DRV_EN_ADC_CHANNEL_simple(3);    // Enable channel 3
 * \endcode
 * \note Argument must be number. If you want use symbolic name (through\n
 * define you should use BRD_DRV_EN_ADC_CHANNEL() macro.
 */
#define BRD_DRV_EN_ADC_CHANNEL_simple(CH)       p_adc->CHER.ch##CH = 1

/**
 * \brief Macro allow enabled needed channel
 *
 * Example:\n
 * \code
 * #define USED_CH      3
 * ...
 * BRD_DRV_EN_ADC_CHANNEL(USED_CH);     // Enable channel 3
 * \endcode
 */
#define BRD_DRV_EN_ADC_CHANNEL(CH)      BRD_DRV_EN_ADC_CHANNEL_simple(CH)

/**
 * \brief Test if selected channel done conversion
 *
 * Example:\n
 * \code
 * if(BRD_DRV_IS_ADC_DONE_simple(1))
 * {    // If ADC conversion done, continue here
 * ...
 * }
 * \endcode
 * \note Argument must be number. If you want use symbolic name (through\n
 * define you should use BRD_DRV_IS_ADC_DONE() macro
 */
#define BRD_DRV_IS_ADC_DONE_simple(CH)          p_adc->SR.eoc##CH

/**
 * \brief Test if selected cannel done conversion
 *
 * Example:\n
 * \code
 * #define USED_CH      1
 * ...
 * if(BRD_DRV_IS_ADC_DONE(USED_CH))
 * {    // If ADC conversion is done, continue here
 * ...
 * }
 * \endcode
 */
#define BRD_DRV_IS_ADC_DONE(CH)         BRD_DRV_IS_ADC_DONE_simple(CH)


/**
 * \brief Read data from selected ADC channel
 *
 * Example:\n
 * \code
 * my_variable = BRD_DRV_ADC_DATA_simple(6);
 * \endcode
 * \note Argument must be number. If you want use symbolic name (through\n
 * define you should use BRD_DRV_ADC_DATA() macro
 */
#define BRD_DRV_ADC_DATA_simple(CH)     p_adc->cdr##CH

/**
 * \brief Read data from selected ADC channel
 *
 * Example:\n
 * \code
 * #define USED_CH      0
 * ...
 * my_variable = BRD_DRV_ADC_DATA(USED_CH);
 * \endcode
 */
#define BRD_DRV_ADC_DATA(CH)            BRD_DRV_ADC_DATA_simple(CH)



#define BRD_DRV_IO_AS_INPUT_simple(PIN)                                 \
    gpio_port =                                                         \
          &AVR32_GPIO.port[PIN >> 5];                                   \
    gpio_port->oderc = 1 << (PIN & 0x1F);                               \
    gpio_port->gpers = 1 << (PIN & 0x1F)

#define BRD_DRV_IO_AS_INPUT(PIN)                BRD_DRV_IO_AS_INPUT_simple(PIN)


#define BRD_DRV_IO_LOW_simple(PIN)                                      \
    gpio_port =                                                         \
          &AVR32_GPIO.port[PIN >> 5];                                   \
    gpio_port->ovrc  = 1 << (PIN & 0x1F);                               \
    gpio_port->oders = 1 << (PIN & 0x1F);                               \
    gpio_port->gpers = 1 << (PIN & 0x1F)


#define BRD_DRV_IO_LOW(PIN)                     BRD_DRV_IO_LOW_simple(PIN)

#define BRD_DRV_IO_HIGH_simple(PIN)                                     \
    gpio_port =                                                         \
          &AVR32_GPIO.port[PIN >> 5];                                   \
    gpio_port->ovrs  = 1 << (PIN & 0x1F);                               \
    gpio_port->oders = 1 << (PIN & 0x1F);                               \
    gpio_port->gpers = 1 << (PIN & 0x1F)


#define BRD_DRV_IO_READ_simple(PIN)                                     \
    gpio_port =                                                         \
          &AVR32_GPIO.port[PIN >> 5];                                   \
    i_pin = (gpio_port->pvr >> (pin & 0x1F)) & 1

#define BRD_DRV_IO_READ(PIN)                    BRD_DRV_IO_READ_simple(PIN)


#define BRD_DRV_IO_HIGH(PIN)                    BRD_DRV_IO_HIGH_simple(PIN)
//==================| Checks and preprocessor calculations |===================
// Check sample and hold time
#if BRD_DRV_ADC_SAMPLE_HOLD_TIME > 15
#error "Sorry, but BRD_DRV_ADC_SAMPLE_HOLD_TIME can not be higher than 15."
#endif
#if BRD_DRV_ADC_SAMPLE_HOLD_TIME < 0
#error "Sorry, but BRD_DRV_ADC_SAMPLE_HOLD_TIME can not be negative."
#endif


// Check start-up time
#if BRD_DRV_ADC_START_UP_TIME > 127
#error "Sorry, but BRD_DRV_ADC_START_UP_TIME can not be higher than 127."
#endif
#if BRD_DRV_ADC_START_UP_TIME < 0
#error "Sorry, but BRD_DRV_ADC_START_UP_TIME can not be negative."
#endif




// Test if ADC clock is not too high
#if BRD_DRV_ADC_PRESCALLED_CLK > 8000000UL
#warning "BRD_DRV_ADC_PRESCALLED_CLK is higher than 8 MHz! ADC can give wrong\
  values! Please consider lower clock for ADC module."
#endif

// If user want let dirty job preprocessor
#if BRD_DRV_ADC_PRESCALLED_CLK == 0
// Define prescaller value. ADC will use 200 kHz input clock (if possible)
#if BRD_DRV_ADC_CLK_SOURCE >= 400000UL
#define BRD_DRV_ADC_PRESCAL             \
  ((BRD_DRV_ADC_CLK_SOURCE/(200000UL*2)) -1)
#else   // BRD_DRV_ADC_CLK_SOURCE >= 400000UL -> else slow clock.
#define BRD_DRV_ADC_PRESCAL             0
#endif

#else // If user want define ADC CLK
#define BRD_DRV_ADC_PRESCAL             \
  ((BRD_DRV_ADC_CLK_SOURCE/(BRD_DRV_ADC_PRESCALLED_CLK*2)) -1)
#endif


// Test if calculated value make sense
#if BRD_DRV_ADC_PRESCAL < 0
#error "Please decrease BRD_DRV_ADC_PRESCALLED_CLK. It can not be higher than\
  1/2 of BRD_DRV_ADC_CLK_SOURCE."
#endif
#if BRD_DRV_ADC_PRESCAL > 255
#error "Please increase BRD_DRV_ADC_PRESCALLED_CLK. It can not be lower than\
  1/512 of BRD_DRV_ADC_CLK_SOURCE."
#endif



// Test ADC channel options
#if BRD_DRV_ADC_VOLUME_CHANNEL < 0
#error "BRD_DRV_ADC_VOLUME_CHANNEL can not be negative!"
#endif
#if BRD_DRV_ADC_VOLUME_CHANNEL > 7
#error "BRD_DRV_ADC_VOLUME_CHANNEL can not be higher than 7!"
#endif

//==========================| High level functions |===========================
GD_RES_CODE brd_drv_pre_init(void);

GD_RES_CODE brd_drv_init(void);

void brd_drv_task(void);
//===========================| Mid level functions |===========================

//===========================| Low level functions |===========================



#endif
