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
 * \brief Board driver for "Sonochan mkII" HW version 03
 *
 * Allow set basic settings on board. Also support "generic driver".
 * Written only for AVR32 UC3A3.
 *
 * Created:  2014/04/23\n
 * Modified: 2015/11/22
 *
 * \version 0.6.1
 * \author  Martin Stejskal
 */

#ifndef _BRD_DRIVER_H_
#define _BRD_DRIVER_H_

//=================================| Options |=================================

/**
 * @brief Enable debug mode.
 *
 * In this mode non-standard settings can be used. This may decrease develop\n
 * time, but may confuse user. Beware!
 */
#define BRD_DRV_DEBUG                                   0
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
 * \name Digital audio interface - default parameters
 *
 * @{
 */

/**
 * @brief Default Digital Audio Interface mode
 *
 * Options: SSC_I2S/SSC_DSP/SSC_LEFT_JUSTIFIED/SSC_RIGHT_JUSTIFIED
 */
#define BRD_DRV_DEFAULT_DAI_MODE                SSC_I2S

/**
 * \brief Default MCLK oversampling relative to FSYNC singal
 */
#define BRD_DRV_DEFAULT_MCLK_OVERSAMPLING       256

/**
 * \brief Default BCLK oversampling relative to FSYNC singal
 */
#define BRD_DRV_DEFAULT_BCLK_OVERSAMPLING       64

/**
 * @brief Default data word length
 */
#define BRD_DRV_DEFAULT_DATA_WORD_LENGTH        24

/**
 * \brief This is only for reference.
 *
 * After connection audio driver change frequency anyway. This is only \n
 * "start up" value
 */
#define BRD_DRV_DEFAULT_FSYNC_FREQ              48000UL

/**
 * @brief By default, enable auto tune?
 *
 * 0 - Auto tune disabled (MCLK Fix) ; 1 - Auto tune enabled
 */
#define BRD_DRV_DEFAULT_AUTO_TUNE               1
/// @}


/**
 * @name Digital audio interface - default signal direction settings
 *
 * Options: brd_drv_dir_out, brd_drv_dir_in and brd_drv_dir_hiz
 *
 * @{
 */

#define BRD_DRV_DEFAULT_MUTE_DIR                brd_drv_dir_hiz
#define BRD_DRV_DEFAULT_RESET_DAI_DIR           brd_drv_dir_hiz
#define BRD_DRV_DEFAULT_MCLK_DIR                brd_drv_dir_hiz
#define BRD_DRV_DEFAULT_BCLK_DIR                brd_drv_dir_hiz
#define BRD_DRV_DEFAULT_FSYNC_DIR               brd_drv_dir_hiz
#define BRD_DRV_DEFAULT_DATA_TX_DIR             brd_drv_dir_hiz
#define BRD_DRV_DEFAULT_DATA_RX_DIR             brd_drv_dir_hiz
/// @}

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
 * \brief Address to memory where are PM registers
 *
 * Example: AVR32_PM_ADDRESS or (&AVR32_PM) - it is the same
 */
#define BRD_DRV_PM_ADDRESS                      AVR32_PM_ADDRESS

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
#define BRD_DRV_ADC_PRESCALLED_CLK      0UL



/**
 * \name ADC pin mapping for board
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
/// @}



/**
 * \name Other pin mapping for Sonochan mkII boards
 * @{
 */
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

///\brief RX enable on MCU side
#define BRD_DRV_RX_EN_B_PIN                     AVR32_PIN_PX00

///\brief Master clock enable on CON side
#define BRD_DRV_MCLK_EN_B_PIN                   AVR32_PIN_PX41

///\brief Bit clock enable on CON side
#define BRD_DRV_BCLK_EN_B_PIN                   AVR32_PIN_PX22

///\brief Frame sync enable on CON side
#define BRD_DRV_FS_EN_B_PIN                     AVR32_PIN_PX45

///\brief Mute button
#define BRD_DRV_MUTE_BTN_PIN                    AVR32_PIN_PX10

///\brief Reset I2S button
#define BRD_DRV_RESET_I2S_BTN_PIN               AVR32_PIN_PA28

///\brief Reset I2S signal
#define BRD_DRV_RESET_I2S_PIN                   AVR32_PIN_PX07

///\brief Mute signal
#define BRD_DRV_MUTE_PIN                        AVR32_PIN_PX38

///\brief Error indication
#define BRD_DRV_ERROR_SIG_PIN                   AVR32_PIN_PC00

/// @}

/*! \name GCLK Settings for the Sonochan mkII boards
 *
 * GCLK0 is used for generating MCLK.\n
 * GCLK1 is used for generating BCLK.
 *
 */
//! @{

#define MCLK_PIN                AVR32_PM_GCLK_0_1_PIN
#define MCLK_FUNCTION           AVR32_PM_GCLK_0_1_FUNCTION
#define MCLK_CLK                AVR32_PM_GCLK_GCLK0
#define BCLK_PIN                AVR32_PM_GCLK_1_1_PIN
#define BCLK_FUNCTION           AVR32_PM_GCLK_1_1_FUNCTION
#define BCLK_CLK                AVR32_PM_GCLK_GCLK1
//! @}




/**
 * \name Fine settings
 *
 * @{
 */
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
#define BRD_DRV_BUTTON_REFRESH_PERIOD           6


/**
 * \brief Define "period" in which will be button press evaluated as short
 *
 * Short RESET_I2S_BTN press clean error message, long reset I2S bus and\n
 * connector.
 */
#define BRD_DRV_SHORT_PRESS                     5

/**
 * \brief When RESET_I2S is output, this define period in high level
 */
#define BRD_DRV_RESET_I2S_PERIOD                1
///@}


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
 * If you are unsure, set this value to 10. Example:\n
 * \code
 * #define BRD_DRV_ADC_START_UP_TIME 10
 * \endcode
 */
#define BRD_DRV_ADC_START_UP_TIME                       10

/**
 * @brief Define minimum frequency which will be detected
 *
 * This is only valid for case that RTOS support is enabled.
 * For some unknown reason it does not comply with reality. Really do not know
 * why, but so it is.
 */
#define BRD_DRV_MIN_DET_FREQ            100UL
//================================| Includes |=================================
// IO pins
#include <avr32/io.h>

// Basic data types
#include <inttypes.h>

/* sprintf() is pure evil. With combination of RTOS is cause many unexpected
 * failures. So we use more lightweight version.
 */
#include "tinyprintf.h"
/* Tinyprintf does not support printing float. So there is small library that
 * simply recalculate float values to string (also with decimal point).
 * However it is quite limited. On the another hand it is very fast and not so
 * much memory consuming
 */
#include "tinyprintf_extra.h"

// Cause of GCLKx stuff
#include "pm.h"

// SSC driver (on chip AVR32)
#include "ssc.h"

// PLL control
#include "cs2200.h"

// For codec control
#include "tlv320aic33.h"

// For display support
#include "display.h"

// Operations with flash memory
#include "flashc.h"

// Printing debug messages through UART
#include "print_funcs.h"

// Because of uac1_device_audio_set_auto_tune() function
#include "uac1_device_audio_task.h"

// USB descriptors can be changed :P#include "usb_descriptors.h"
#include "usb_descriptors.h"
#include "uac1_usb_descriptors.h"

// GPIO operations
///\todo Try to remove this dependency
#include "gpio.h"

#if BRD_DRV_SUPPORT_RTOS != 0
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
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
        GD_INCORRECT_DEVICE_ID =       5,//!< GD_INCORRECT_DEVICE_ID
        GD_TIMEOUT =                   6,//!< GD_TIMEOUT
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
  brd_drv_dir_in = 0, //!< brd_drv_dir_in Input direction
  brd_drv_dir_out = 1,//!< brd_drv_dir_out Output direction
  brd_drv_dir_hiz = 2 //!< brd_drv_dir_hiz Hi-Z ("input" from both sides)
}e_brd_drv_dir_t;

/**
 * \brief Mute structure
 *
 * Mute for Digital Audio Interface
 */
typedef struct{
  e_brd_drv_dir_t e_mute_dir;
  uint8_t         i_mute_val;
}s_brd_drv_mute_dai_t;

/**
 * \brief Reset Digital Audio Interface (DAI) structure
 *
 * Mute for Digital Audio Interface
 */
typedef struct{
  e_brd_drv_dir_t e_rst_dai_dir;
  uint8_t         i_rst_dai_val;
}s_brd_drv_rst_dai_t;


/**
 * \brief Directions for pure Digital Audio Interface (DAI) signals
 *
 * Direction for: MCLK, BCLK, FSYNC, TX_DATA, RX_DATA
 */
typedef struct{
  e_brd_drv_dir_t e_mclk_dir;
  e_brd_drv_dir_t e_bclk_dir;
  e_brd_drv_dir_t e_fsync_dir;
  e_brd_drv_dir_t e_tx_data_dir;
  e_brd_drv_dir_t e_rx_data_dir;
}s_brd_drv_pure_dai_dir_t;


/**
 * @brief Structure for delayed setting FSYNC frequency
 *
 * Contains new (requested frequency) and status flag
 */
typedef struct{
  uint32_t i_new_FSYNC_freq;
  uint8_t  i_request;
}s_brd_drv_FSYNC_freq_req_t;

/**
 * \brief Structure for "EEPROM" settings
 *
 * In this structure are all variables, that should be saved to EEPROM like\n
 * memory when user want to keep settings.
 */
typedef struct{
  //===== I/O settings (also SSC settings)
  /// MUTE direction
  e_brd_drv_dir_t e_mute_dir;
  /// RESET I2S direction
  e_brd_drv_dir_t e_rst_dai_dir;
  /// MCLK direction
  e_brd_drv_dir_t e_mclk_dir;
  /// BCLK direction
  e_brd_drv_dir_t e_bclk_dir;
  /// FRAME SYNC direction
  e_brd_drv_dir_t e_fsync_dir;
  /// TX DATA direction
  e_brd_drv_dir_t e_tx_data_dir;
  /// RX DATA direction
  e_brd_drv_dir_t e_rx_data_dir;


  //===== SSC fine settings
  /// Digital interface mode
  e_ssc_digital_audio_interface_t e_dig_audio_mode;
  /// FSYNC RX edge
  e_ssc_edge_t e_FSYNC_RX_edge;
  /// FSYNC TX edge
  e_ssc_edge_t e_FSYNC_TX_edge;
  /// BCLK RX edge
  e_ssc_edge_t e_BCLK_RX_edge;
  /// BCLK TX edge
  e_ssc_edge_t e_BCLK_TX_edge;
  /// Length of FSYNC pulse (only in DSP mode)
  uint8_t i_FSYNC_pulse;
  /** Word bit offset
   * Number of bits (BCLK intervals) before start of actual audio data.
   */
  uint16_t i_word_bit_offset;
  /// Data length in bits
  uint8_t i_data_length;
  /// BCLK oversampling
  uint16_t i_BCLK_ovrsmpling;
  /// MCLK oversampling
  uint16_t i_MCLK_ovrsmpling;

  /// FSYNC frequency
  uint32_t i_FSYNC_freq;

  /// PPM offset of MCLK
  int32_t  i_MCLK_ppm_offset;

  //===== Other settings
  /// Auto tune external PLL option
  uint8_t i_auto_tune_pll;
}s_brd_drv_user_settings_t;


/**
 * \brief Structure that handle fine SSC settings
 *
 * Because SSC module is very flexible it is good idea to allow user\n
 * (developer) set also some fine settings. This may be useful when\n
 * developing new digital audio interface.
 */
typedef struct{
  /// Digital audio interface (I2S, DSP, ...)
  e_ssc_digital_audio_interface_t e_dig_aud_mode;
  /// FSYNC RX edge
  e_ssc_edge_t e_FSYNC_RX_edge;
  /// FSYNC TX edge
  e_ssc_edge_t e_FSYNC_TX_edge;
  /// BCLK RX edge
  e_ssc_edge_t e_BCLK_RX_edge;
  /// BCLK TX edge
  e_ssc_edge_t e_BCLK_TX_edge;
  /// Length of FSYNC pulse (only in DSP mode)
  uint8_t i_FSYNC_pulse;
  /** Word bit offset
   * Number of bits (BCLK intervals) before start of actual audio data.
   */
  uint16_t i_word_bit_offset;
  /// Data length in bits
  uint8_t i_data_length;
  /// BCLK oversampling
  uint16_t i_BCLK_ovrsmpling;
  /// MCLK oversampling
  uint16_t i_MCLK_ovrsmpling;

  /// FSYNC frequency
  uint32_t i_FSYNC_freq;

  /// PPM offset of MCLK
  int32_t  i_MCLK_ppm_offset;
}s_brd_drv_ssc_fine_setting_t;


//===============================| Definitions |===============================
/**
 * \name Error messages
 *
 * This is set of error messages, which will be written to debug UART or LCD
 * @{
 */
/// Can not initialize LCD display
#define BRD_DRV_MSG_ERR_LCD_INIT_FAIL       \
  "LCD initialization failed!\n"

/// To apply changes, please restart device
#define BRD_DRV_MSG_ERR_DEVICE_RESTART_REQUIRED                         \
  "Device MUST be restarted to apply settings!\n"

/// Can not initialize ADC
#define BRD_DRV_MSG_ERR_ADC_INIT_FAIL       \
  "Can not initialize internal ADC\n"

/// Can not initialize codec
#define BRD_DRV_MSG_ERR_CODEC_INIT_FAIL     \
  "Can not initialize codec\n"

/// Can not set save flag
#define BRD_DRV_MSG_ERR_PLL_SET_SAVE_FLAG_FAIL      \
  "Can not set save flag at PLL\n"

/// Can not set headphone volume in dB
#define BRD_DRV_MSG_ERR_TLV_FAILED_SET_HEADPHONE_VOL_DB \
  "Can not set headphone volume in DB\n"

/// Can not write to LCD
#define BRD_DRV_MSG_ERR_LCD_WRITE_FAIL                  \
  "Can not write to LCD\n"

/// Voltage on connector side is too high
#define BRD_DRV_MSG_ERR_CON_VOL_HIGH                    \
  "Con: high voltage!\n"

/// Info that connector is not powered so can not react for buttons
#define BRD_DRV_MSG_ERR_CON_NOT_POWERED         \
  "Connector side is not powered\n"

/// When setting MCLK oversampling value and PLL value is too high
#define BRD_DRV_MSG_ERR_PLL_HIGH_FREQ           \
  "MCLK ovrsam. Too high freq on PLL\n"

/// Can not set MCLK divider when settings MCLK oversampling
#define BRD_DRV_MSG_ERR_MCLK_DIV_FAIL           \
  "MCLK ovrsam. Can't set divider\n"

/// Can not set PLL frequency at set MCLK oversampling function
#define BRD_DRV_MSG_ERR_CAN_NOT_SET_PLL_FREQ_MCLK    \
  "MCLK ovrsam. Can not set PLL\n"

/// Can not set PLL frequency at set BCLK oversampling function
#define BRD_DRV_MSG_ERR_CAN_NOT_SET_PLL_FREQ_BCLK       \
  "BCLK ovrsam. Can not set PLL\n"

/// Can not set MCLK PPM offset
#define BRD_DRV_MSG_ERR_CAN_NOT_SET_MCLK_PPM_OFFSET     \
  "Can not set MCLK PPM offset!\n"

/// Can not set BCLK oversampling value
#define BRD_DRV_MSG_ERR_MCLK_OVRSAM_CAN_NOT_SET_BCLK_OVRSAM     \
  "MCLK ovrsam. Can not set BLCK ovrsam.\n"

/// Can not set selected mode, because it is unknown
#define BRD_DRV_MSG_ERR_UNKNOWN_DIG_ITF_MODE            \
  "Can not set digital audio mode, because is unknown!\n Code incomplete?\n"

/// MCLK too low. Can not be lower than BCLK.
#define BRD_DRV_MSG_ERR_MCLK_LOWER_THAN_BCLK            \
  "Can not set BCLK oversampling higher than MCLK oversampling!\n"

/// Can not set BCLK because it would cut data word.
#define BRD_DRV_MSG_ERR_BCLK_LOWER_THAN_DATA_WORD       \
  "Can not set BCLK this low, because of data word size.\n"

/// MCLK oversampling can not be 0
#define BRD_DRV_MSG_ERR_MCLK_OVERSAM_CANNOT_BE_0        \
  "MCLK oversampling can not be 0!\n"

/// BCLK is not exact multiplier of MCLK
#define BRD_DRV_MSG_ERR_BCLK_IS_NOT_MUL_OF_MCLK         \
  "BCLK is not exact multiplier of MCLK.\n"

/// BCLK oversampling can not be 0
#define BRD_DRV_MSG_ERR_BCLK_OVERSAM_CANNOT_BE_0        \
  "BCLK oversampling can not be 0!\n"

/// Request for setting data length, but BCLK is too low
#define BRD_DRV_MSG_ERR_DATA_LEN_LONG_BCLK_LOW          \
  "Can not set data length because of low BCLK\n"

/// Request to set word offset in RJF, but BCLK is too high
#define BRD_DRV_MSG_ERR_CANT_SET_WORD_OFF_BCLK_HIGH     \
  "Word offset can not be set because BCLK is too high!\n"

/// Can not set PLL frequency - too high value
#define BRD_DRV_MSG_ERR_CANT_SET_PLL_FREQ_TOO_HIGH      \
  "Can not set PLL frequency (too high frequency)\n"

/// MCLK PPM offset should not be set when auto tune is enabled
#define BRD_DRV_MSG_ERR_AUTO_TUNE_ENABLED_MCLK_PPM_OFF_NOT_ALLOW        \
  "Can not set MCLK PPM offset because auto tune PLL is enabled\n"

/// Settings FSYNC frequency (through request) failed
#define BRD_DRV_MSG_ERR_SET_FSYNC_FREQ_REQ_FAILED               \
  "Setting FSYNC frequency failed!\n"

/// FSYNC can not be zero
#define BRD_DRV_MSG_ERR_FSYNC_CAN_NOT_BE_ZERO                   \
  "FSYNC frequency is set to 0!"
/// @}



/**
 * \name Warning messages
 * @{
 */
#define BRD_DRV_MSG_WRN_FLASH_NOT_VALID_SETTINGS        \
  "Non valid settings in user flash!\n"

#define BRD_DRV_MSG_WRN_CAN_NOT_SET_PLL_TRYING_AGAIN    \
  "Can not set external PLL. Trying again...\n"

#define BRD_DRV_MSG_WRN_MCLK_OVRSM_HIGH                 \
  "MCLK oversampling is set to too high value\n"

#define BRD_DRV_MSG_WRN_OFF_PLUS_DATA_HIGHR_THAN_HALF_BCLK      \
  "Word offset plus data size can not fit to frame! LSB can be cut!\n"

#define BRD_DRV_MSG_WRN_CODEC_NOT_SUPPORT_HIGH_WORD_OFFSET      \
  "Codec does not support this word offset!\n"

#define BRD_DRV_MSG_WRN_CODEC_NOT_SUPP_WRLD_LEN_DSP             \
  "Codec not support this world length in DSP\n"

#define BRD_DRV_MSG_WRN_CODEC_NOT_SUPP_LOWER_THAN_BCLK32        \
  "Codec not support BCLK lower than 32\n"

///\brief When detecting frequency at BCLK, low level is not detected
#define BRD_DRV_MSG_WRN_BCLK_DTCT_FREQ_L_NDET           \
  "BCLK: Low level NOT detected!\n"

///\brief When detecting frequency at BCLK, high level is not detected
#define BRD_DRV_MSG_WRN_BCLK_DTCT_FREQ_H_NDET           \
  "BCLK: High level NOT detected!\n"

///\brief When detecting frequency at FSYNC, low level is not detected
#define BRD_DRV_MSG_WRN_FSYNC_DTCT_FREQ_L_NDET           \
  "FSYNC: Low level NOT detected!\n"

///\brief When detecting frequency at FSYNC, high level is not detected
#define BRD_DRV_MSG_WRN_FSYNC_DTCT_FREQ_H_NDET           \
  "FSYNC: High level NOT detected!\n"
/// @}



/**
 * \name Information messages
 * @{
 */
/// Message when initializing board driver
#define BRD_DRV_MSG_INFO_INIT_BRD                       \
  "Initializing device...\n"

/// Informs that board driver waiting for other tasks
#define BRD_DRV_MSG_INFO_WTING_4_TSKS                   \
  "Waiting for other tasks...\n"

/// When all dependent tasks are ready
#define BRD_DRV_MSG_INFO_TSKS_RDY                       \
  "Tasks are ready now\n"

/// Voltage on connector side in save range
#define BRD_DRV_MSG_INFO_CON_VOL_SAVE                    \
  "Con: save voltage\n"

/// Mute direction IN ; Mute off
#define BRD_DRV_MSG_INFO_MUTE_IN_MUTE_OFF                \
  "MUTE (IN): OFF\n"

/// Mute direction IN ; Mute on
#define BRD_DRV_MSG_INFO_MUTE_IN_MUTE_ON                 \
  "MUTE (IN): ON\n"

/// Mute direction OUT ; Mute off
#define BRD_DRV_MSG_INFO_MUTE_OUT_MUTE_OFF               \
  "MUTE (OUT): OFF\n"

/// Mute direction OUT ; Mute on
#define BRD_DRV_MSG_INFO_MUTE_OUT_MUTE_ON                \
  "MUTE (OUT): ON\n"

/// Mute direction IN ; Button pressed ; Mute on
#define BRD_DRV_MSG_INFO_MSG_MUTE_BTN_PRESSED            \
  "MUTE_BTN pressed\n"

/// Mute direction IN ; Button released ; Mute off
#define BRD_DRV_MSG_INFO_MSG_MUTE_BTN_RELEASED           \
  "MUTE_BTN released\n"

/// Signal MUTE - rising edge
#define BRD_DRV_MSG_INFO_MUTE_RISING_EDGE            \
  "MUTE: rising edge\n"

/// Signal MUTE - falling edge
#define BRD_DRV_MSG_INFO_MUTE_FALLING_EDGE           \
  "MUTE: falling edge\n"

/// Error message was cleaned from LCD
#define BRD_DRV_MSG_INFO_ERR_MSG_CLEARED             \
  "Error message cleared\n"

/// Info that restart I2S was occurred
#define BRD_DRV_MSG_INFO_RESET_I2S_DONE              \
  "Reset I2S done\n"

/// Info that RESET_I2S_PIN was set to HIGH
#define BRD_DRV_MSG_INFO_RESET_I2S_SET_TO_HIGH       \
  "Reset I2S (OUT) set to HIGH\n"

/// Info that RESET_I2S_PIN was set to LOW
#define BRD_DRV_MSG_INFO_RESET_I2S_SET_TO_LOW        \
  "Reset I2S (OUT) set to LOW\n"

/// RESET_I2S_PIN is input and there is require to turn off reset I2S
#define BRD_DRV_MSG_INFO_RESET_I2S_INPUT_OFF         \
  "Reset I2S (IN) off\n"

/// RESET_I2S_PIN is input and there is require to turn on reset I2S
#define BRD_DRV_MSG_INFO_RESET_I2S_INPUT_ON          \
  "Reset I2S (IN) on\n"

/// Info that rising edge was detected on RESET_I2S_PIN
#define BRD_DRV_MSG_INFO_RST_I2S_RISING_EDGE         \
  "RESET_I2S: rising edge\n"

/// Info that falling edge was detected on RESET_I2S_PIN
#define BRD_DRV_MSG_INFO_RST_I2S_FALLING_EDGE        \
  "RESET_I2S: falling edge\n"

/// RESET_I2S_BTN was pressed
#define BRD_DRV_MSG_INFO_RST_I2S_BTN_PRESSED         \
  "RESET_I2S_BTN pressed\n"

/// RESET_I2S_BTN was released
#define BRD_DRV_MSG_INFO_RST_I2S_BTN_RELEASED        \
  "RESET_I2S_BTN released\n"

/// Error message was removed
#define BRD_DRV_MSG_INFO_ERROR_CLEANED          \
  "ERROR message deleted\n\n\n"

#define BRD_DRV_MSG_INFO_LOAD_FACTRY_STTNGS         \
  "Loading factory settings...\n"

#define BRD_DRV_MSG_INFO_FACTRY_STTNGS_LOADED       \
  "Factory settings was loaded\n"

#define BRD_DRV_MSG_INFO_FLASH_VALID_SETTINGS           \
  "Valid user settings in user flash. Loading...\n"

#define BRD_DRV_MSG_INFO_CDC_ITF_I2S                    \
  "Interface at codec: I2S\n"

#define BRD_DRV_MSG_INFO_CDC_ITF_DSP                    \
  "Interface at codec: DPS\n"

#define BRD_DRV_MSG_INFO_CDC_ITF_R_JUS                  \
  "Interface at codec: right justified\n"

#define BRD_DRV_MSG_INFO_CDC_ITF_L_JUS                  \
  "Interface at codec: left justified\n"

#define BRD_DRV_MSG_INFO_FMT_I2S                        \
  "Using I2S format\n"

#define BRD_DRV_MSG_INFO_FMT_DSP                        \
  "Using DSP format\n"

#define BRD_DRV_MSG_INFO_FMT_R_JUS                      \
  "Using right justified format\n"

#define BRD_DRV_MSG_INFO_FMT_L_JUS                      \
  "Using left justified format\n"

///\brief Double PLL frequency, so we can set odd multiplier BCLK of MCLK
#define BRD_DRV_MSG_INFO_DOUBLING_PLL_FREQ_SET_BCLK_ODD_MUL     \
  "Doubling PLL frequency so BCLK can be odd multiplier of MCLK\n"

///\brief Settings saved to flash
#define BRD_DRV_MSG_INFO_SETTINGS_SAVED                 \
  "Settings saved\n"

///\brief Settings from flash loaded
#define BRD_DRV_MSG_INFO_SETTINGS_LOADED                \
  "Settings loaded\n"

///\brief Board driver initialized
#define BRD_DRV_MSG_INFO_BRD_DRV_INITIALIZED            \
  "Board driver initialized\n"

///\brief Detecting frequency at BCLK signal when set as input
#define BRD_DRV_MSG_INFO_BCLK_DTCT_FREQ                 \
  "BCLK: Detecting frequency...\n"

///\brief When detecting frequency at BCLK, low level detected
#define BRD_DRV_MSG_INFO_BCLK_DTCT_FREQ_L_DET           \
  "BCLK: Low level detected\n"

///\brief When detecting frequency at BCLK, high level detected
#define BRD_DRV_MSG_INFO_BCLK_DTCT_FREQ_H_DET           \
  "BCLK: High level detected\n"

///\brief Detecting frequency at FSYNC signal when set as input
#define BRD_DRV_MSG_INFO_FSYNC_DTCT_FREQ                 \
  "FSYNC: Detecting frequency...\n"

///\brief When detecting frequency at FSYNC, low level detected
#define BRD_DRV_MSG_INFO_FSYNC_DTCT_FREQ_L_DET           \
  "FSYNC: Low level detected\n"

///\brief When detecting frequency at FSYNC, high level detected
#define BRD_DRV_MSG_INFO_FSYNC_DTCT_FREQ_H_DET           \
  "FSYNC: High level detected\n"


/// @}

/**
 * \brief Check code which allow detect correct settings in user flash
 *
 * It should be in range 2 - 254 (0, 1 and 255 are excluded for safety reasons)
 * Also constant is more easy than CRC
 */
#define BRD_DRV_FLASH_CHECK_CODE                38
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

GD_RES_CODE brd_drv_reset_i2s(void);

GD_RES_CODE brd_drv_set_digital_audio_interface_mode(
    e_ssc_digital_audio_interface_t e_mode);
GD_RES_CODE brd_drv_get_digital_audio_interface_mode(
    e_ssc_digital_audio_interface_t *p_e_mode);

GD_RES_CODE brd_drv_set_FSYNC_freq(uint32_t i_FSYNC_freq);
GD_RES_CODE brd_drv_get_FSYNC_freq(uint32_t *p_i_FSYNC_freq);

GD_RES_CODE brd_drv_set_FSYNC_freq_rqst(uint32_t i_FSYNC_freq);

GD_RES_CODE brd_drv_set_MCLK_oversampling(uint16_t i_MCLK_oversampling);
GD_RES_CODE brd_drv_get_MCLK_oversampling(uint16_t *p_i_MCLK_oversampling);

GD_RES_CODE brd_drv_get_MCLK_freq(uint32_t *p_i_MCLK_freq);

GD_RES_CODE brd_drv_set_BCLK_oversampling(uint16_t i_BCLK_ovrsmpling);
GD_RES_CODE brd_drv_get_BCLK_oversampling(uint16_t *p_i_BCLK_oversampling);

GD_RES_CODE brd_drv_get_volume_dB(float *p_f_volume);

GD_RES_CODE brd_drv_set_number_to_product_name(uint8_t i_number);
GD_RES_CODE brd_drv_get_number_from_product_name(uint8_t *p_i_number);

GD_RES_CODE brd_drv_save_all_settings(void);

GD_RES_CODE brd_drv_restore_all_settings(void);

GD_RES_CODE brd_drv_load_default_settings(void);
//===========================| Mid level functions |===========================
GD_RES_CODE brd_drv_set_data_length(uint8_t i_data_length);
GD_RES_CODE brd_drv_get_data_length(uint8_t *p_i_data_length);

GD_RES_CODE brd_drv_set_FSYNC_RX_edge(e_ssc_edge_t e_edge);
GD_RES_CODE brd_drv_set_FSYNC_TX_edge(e_ssc_edge_t e_edge);

GD_RES_CODE brd_drv_set_BCLK_RX_edge(e_ssc_edge_t e_edge);
GD_RES_CODE brd_drv_set_BCLK_TX_edge(e_ssc_edge_t e_edge);

GD_RES_CODE brd_drv_set_word_offset(uint16_t i_word_offset);
GD_RES_CODE brd_drv_get_word_offset(uint16_t *p_i_word_offset);

GD_RES_CODE brd_drv_set_MCLK_ppm(int32_t i_MCLK_ppm_offset);
GD_RES_CODE brd_drv_get_MCLK_ppm(int32_t *p_i_MCLK_ppm_offset);


void brd_drv_send_msg(
    const char * p_msg,
    const uint8_t i_write_to_DBG,
    const uint8_t i_write_to_LCD,
    const uint8_t i_LCD_line);

void brd_drv_send_warning_msg(
    const char * p_msg,
    const uint8_t i_write_to_DBG,
    const uint8_t i_write_to_LCD);

void brd_drv_send_error_msg(
    const char * p_msg,
    const uint8_t i_write_to_DBG,
    const uint8_t i_write_to_LCD);
//===========================| Low level functions |===========================
GD_RES_CODE brd_drv_set_auto_tune(uint8_t i_enable);

GD_RES_CODE brd_drv_set_isolators_to_HiZ(void);

GD_RES_CODE brd_drv_get_auto_tune(uint8_t *p_i_enable);

GD_RES_CODE brd_drv_set_isolators(s_brd_drv_pure_dai_dir_t s_pure_dai_dir);

GD_RES_CODE brd_drv_set_mclk_dir(e_brd_drv_dir_t e_mclk_dir);

GD_RES_CODE brd_drv_set_bclk_dir(e_brd_drv_dir_t e_bclk_dir);

GD_RES_CODE brd_drv_set_fsync_dir(e_brd_drv_dir_t e_fsync_dir);

GD_RES_CODE brd_drv_set_tx_data_dir(e_brd_drv_dir_t e_tx_data_dir);

GD_RES_CODE brd_drv_set_rx_data_dir(e_brd_drv_dir_t e_rx_data_dir);

GD_RES_CODE brd_drv_set_mute_dir(e_brd_drv_dir_t e_mute_dir);

GD_RES_CODE brd_drv_set_mute(uint8_t i_mute_flag);

GD_RES_CODE brd_drv_set_rst_dai_dir(e_brd_drv_dir_t e_rst_dai_dir);

GD_RES_CODE brd_drv_set_rst_dai(uint8_t i_reset_dai_flag);

GD_RES_CODE brd_drv_set_BCLK_div(uint16_t i_div);

GD_RES_CODE brd_drv_get_BCLK_div(uint16_t *p_i_div);

GD_RES_CODE brd_drv_set_MCLK_div(uint16_t i_div);

GD_RES_CODE brd_drv_get_MCLK_div(uint16_t *p_i_div);

#endif
