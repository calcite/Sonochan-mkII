/**
 * \file
 *
 * \brief Board driver for "Sonochan mkII" HW version 03
 *
 * Allow set basic settings on board. Also support "generic driver"
 *
 * Created:  23.04.2014\n
 * Modified: 07.05.2014
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
 * \brief Define on which line begin write logo
 *
 * This should be set to 0 or at least same as BRD_DRV_LCD_WARN_MSG_LINE
 */
#define BRD_DRV_LCD_LOGO_LINE                   0

/**
 * \brief Define on which line begin write warning message
 *
 * This should be set to 0, because when warning message disappear logo is\n
 * written on these lines.
 */
#define BRD_DRV_LCD_WARN_MSG_LINE               0

/**
 * \brief Define on which line begin write info message
 *
 * It is recommended to leave BRD_DRV_LCD_WARN_MSG_LINE 2 lines and rest\n
 * for info message. This value define where info message start
 */
#define BRD_DRV_LCD_INFO_MSG_LINE               2
/**
 * \brief Define on which line will be displayed headphone volume
 */
#define BRD_DRV_LCD_HP_VOLUME_LINE              5



/**
 * \brief Define when volume is updated (how much must be value on ADC\n
 * different from previous value on ADC)
 *
 * Because of noise on ADC it is recommended to set value 3. Enough for\n
 * user, no I2C bandwidth wasting because of ADC noise. But it depends on\n
 * board and environs. For 8 bit ADC resolution is recommended 1 or 2.
 */
#define BRD_DRV_ADC_UPDATE_THRESHOLD            3

/**
 * \brief Define "period" in which will be input buttons, ADC and so on scanned
 *
 * Period is set by experimental. It depends on system load, if used RTOS,
 * RTOS priority and many other stuff.\n
 * Scan "mute", "reset I2S", ADC volume control, ADC con voltage and so on.
 */
#define BRD_DRV_BUTTON_REFRESH_PERIOD           4


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

// Because of sprintf()
#include <stdio.h>

// PLL control
#include "cs2200.h"

// For codec control
#include "tlv320aic33.h"

// For LCD support
#include "LCD_5110.h"

// Pin names
///\todo Move definitions here
#include "board.h"

// Printing debug messages through UART
#include "print_funcs.h"

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

/**
 * \brief Connector voltage states
 *
 * Generally voltage on connector side can have
 */
typedef enum{
  //!< brd_drv_con_low_vol Isolators turned off
  brd_drv_con_low_vol = 0,
  //!< brd_drv_con_save_vol Isolators works with save voltage
  brd_drv_con_save_vol = 1,
  //!< brd_drv_con_high_vol Isolators works with high voltage
  brd_drv_con_high_vol = 2,
  //!< brd_drv_con_undefined Undefined state. For state machine
  brd_drv_con_undefined = 3
}e_brd_drv_con_state;

/**
 * \brief Pin directions
 *
 * Symbolic names for pin directions
 */
typedef enum{
  brd_drv_dir_in = 0,//!< brd_drv_dir_in Input direction
  brd_drv_dir_out = 1//!< brd_drv_dir_out Output direction
}e_brd_drv_dir_t;

/**
 * \brief Mute structure
 */
typedef struct{
  e_brd_drv_dir_t e_mute_dir;
  uint8_t         i_mute_val;
}s_brd_drv_mute_t;

/**
 * \brief Reset I2S structure
 */
typedef struct{
  e_brd_drv_dir_t e_rst_i2s_dir;
  uint8_t         i_rst_i2s_val;
}s_brd_drv_rst_i2s_t;

//===============================| Definitions |===============================
/**
 * \name Error messages
 *
 * This is set of error messages, which will be written to debug UART or LCD
 * @{
 */
/// Can not initialize LCD display
#define BRD_DRV_MSG_LCD_INIT_FAIL       \
  {"LCD initialization failed!\n"}

/// Sonochan mk II
#define BRD_DRV_MSG_SONOCHAN_MK_II      \
  {"Sonochan mk II\n > Beta\n"}

/// Can not draw logo
#define BRD_DRV_MSG_DRAW_LOGO_FAIL      \
  {"Write logo failed\n"}

/// Can not initialize ADC
#define BRD_DRV_MSG_ADC_INIT_FAIL       \
  {"Can not initialize internal ADC\n"}

/// Can not initialize codec
#define BRD_DRV_MSG_CODEC_INIT_FAIL     \
  {"Can not initialize codec\n"}

/// Can not set save flag
#define BRD_DRV_MSG_PLL_SET_SAVE_FLAG_FAIL      \
  {"Can not set save flag at PLL\n"}

/// Can not set headphone volume in dB
#define BRD_DRV_TLV_FAILED_SET_HEADPHONE_VOL_DB \
  {"Can not set headphone volume in DB\n"}

/// Can not write to LCD
#define BRD_DRV_LCD_WRITE_FAIL                  \
  {"Can not write to LCD\n"}

/// Voltage on connector side is too low or powered off
#define BRD_DRV_CON_VOL_LOW                     \
  {"Con: low voltage\n"}

/// Voltage on connector side in save range
#define BRD_DRV_CON_VOL_SAVE                    \
  {"Con: save voltage\n"}

/// Voltage on connector side is too high
#define BRD_DRV_CON_VOL_HIGH                    \
  {"Con: high voltage!\n"}

/// Mute direction IN ; Mute off
#define BRD_DRV_MUTE_IN_MUTE_OFF                \
  {"MUTE (IN): OFF\n"}

/// Mute direction IN ; Mute on
#define BRD_DRV_MUTE_IN_MUTE_ON                 \
  {"MUTE (IN): ON\n"}

/// Mute direction OUT ; Mute off
#define BRD_DRV_MUTE_OUT_MUTE_OFF               \
  {"MUTE (OUT): OFF\n"}

/// Mute direction OUT ; Mute on
#define BRD_DRV_MUTE_OUT_MUTE_ON                \
  {"MUTE (OUT): ON\n"}

/// Mute direction IN ; Button pressed ; Mute on
#define BRD_DRV_MSG_MUTE_BTN_PRESSED            \
  {"MUTE BTN pressed: MUTE ON\n"}

/// Mute direction IN ; Button released ; Mute off
#define BRD_DRV_MSG_MUTE_BTN_RELEASED           \
  {"MUTE BTN released: MUTE OFF\n"}

/// Info that connector is not powered so can not react for buttons
#define BRD_DRV_CON_NOT_POWERED                 \
  {"Connector side is not powered\n"}
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

#define BRD_DRV_IO_HIGH(PIN)                    BRD_DRV_IO_HIGH_simple(PIN)


#define BRD_DRV_IO_READ_simple(VAR,PIN)                                     \
    gpio_port =                                                         \
          &AVR32_GPIO.port[PIN >> 5];                                   \
    VAR = (gpio_port->pvr >> (PIN & 0x1F)) & 1

#define BRD_DRV_IO_READ(VAR,PIN)                BRD_DRV_IO_READ_simple(VAR,PIN)



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
void brd_drv_send_msg(
    const char * p_msg,
    uint8_t i_write_to_DBG,
    uint8_t i_write_to_LCD,
    uint8_t i_LCD_line);

void brd_drv_send_warning_msg(
    const char * p_msg,
    uint8_t i_write_to_DBG,
    uint8_t i_write_to_LCD);

void brd_drv_send_error_msg(
    const char * p_msg,
    uint8_t i_write_to_DBG,
    uint8_t i_write_to_LCD);
//===========================| Low level functions |===========================
GD_RES_CODE brd_drv_set_isolators_to_HiZ(void);

GD_RES_CODE brd_drv_set_mute_reset_i2s_pins_as_input(void);

GD_RES_CODE brd_drv_set_mute_direction(e_brd_drv_dir_t e_mute_dir);

GD_RES_CODE brd_drv_set_rst_i2s_direction(e_brd_drv_dir_t e_rst_i2s_dir);

#endif
