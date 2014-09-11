/**
 * \file
 *
 * \brief Driver for SSC module on UC3A3256
 *
 * Assume that RX module is ALWAYS set as slave and TX module is configured\n
 * as master or even slave. Also assume, that some pins are wired:\n
 * (BCLK)  SSC-RX_CLOCK      <-----> SSC-TX_CLOCK\n
 * (FSYNC) SSC-RX_FRAME_SYNC <-----> SSC-TX_FRAME_SYNC\n
 * \n
 * Created:  20.08.2014\n
 * Modified: 09.09.2014
 *
 * \version 0.2
 * \author Martin Stejskal
 */
//[Martin] Because this is cross dependent
#include "brd_driver_hw_03.h"

#ifndef _SSC_UC3A3256_H_
#define _SSC_UC3A3256_H_

//===========================| Included libraries |============================
// Data types
#include <inttypes.h>

// AVR IO
#include <avr32/io.h>

// Use GPIO library, because this library is soooo HW dependent
///\todo Try to remove this dependency
#include "gpio.h"

//=============================| Basic settings |==============================

/**
 * \brief Allow define function, that will be given error messages
 *
 * As argument is always given only string, so functions can be pretty\n
 * simple. Example:\n
 * \code
 * #define SSC_ERR_FUNC(msg) printf(msg)
 * // or
 * #include "print_funcs.h"
 * #define SSC_ERR_FUNC(msg) my_LCD_function(msg)
 * \endcode
 *
 * \note Do not forget include correct .h file, where is function located ;)\n
 * If you have any troubles with included file, just put it at the end of\n
 * file (behind #endif) or to the begin file (before #ifndef .....).
 */
#define SSC_ERR_FUNC(msg)               brd_drv_send_error_msg(msg,1,1)

/**
 * \brief Allow define function, that will be given info messages
 *
 * As argument is always given only string, so functions can be pretty\n
 * simple. Example:\n
 * \code
 * #define SSC_INFO_FUNC(msg) printf(msg)
 * // or
 * #include "print_funcs.h"
 * #define SSC_INFO_FUNC(msg) my_LCD_function(msg)
 * \endcode
 *
 * \note Do not forget include correct .h file, where is function located ;)\n
 * If you have any troubles with included file, just put it at the end of\n
 * file (behind #endif) or to the begin file (before #ifndef .....).
 */
#define SSC_INFO_FUNC(msg)              brd_drv_send_msg(msg,1,0,-1)

/**
 * \brief TWI interface
 *
 * Aka (*((volatile avr32_twi_t*)AVR32_SSC_ADDRESS)) or (&AVR32_SSC)\n
 * Example: SSC_DEVICE (&AVR32_SSC)
 */
#define SSC_DEVICE                      (&AVR32_SSC)

/*! \name SSC pin mapping
 */
//! @{

#define SSC_RX_DATA                     AVR32_SSC_RX_DATA_0_2_PIN
#define SSC_RX_DATA_FUNCTION            AVR32_SSC_RX_DATA_0_2_FUNCTION
#define SSC_RX_FRAME_SYNC               AVR32_SSC_RX_FRAME_SYNC_0_2_PIN
#define SSC_RX_FRAME_SYNC_FUNCTION      AVR32_SSC_RX_FRAME_SYNC_0_2_FUNCTION
#define SSC_RX_CLOCK                    AVR32_SSC_RX_CLOCK_0_1_PIN
#define SSC_RX_CLOCK_FUNCTION           AVR32_SSC_RX_CLOCK_0_1_FUNCTION
#define SSC_TX_DATA                     AVR32_SSC_TX_DATA_0_1_PIN
#define SSC_TX_DATA_FUNCTION            AVR32_SSC_TX_DATA_0_1_FUNCTION
#define SSC_TX_FRAME_SYNC               AVR32_SSC_TX_FRAME_SYNC_0_1_PIN
#define SSC_TX_FRAME_SYNC_FUNCTION      AVR32_SSC_TX_FRAME_SYNC_0_1_FUNCTION
#define SSC_TX_CLOCK                    AVR32_SSC_TX_CLOCK_0_1_PIN
#define SSC_TX_CLOCK_FUNCTION           AVR32_SSC_TX_CLOCK_0_1_FUNCTION
/**
 * \brief Another external pin that allow watch FSYNC and synchronize data
 *
 * This pin MUST be used only as GPIO. No other function must not use this\n
 * pin. Also MUST be connected to FSYNC signal.
 */
#define SSC_EXTERNAL_FSYNC_DETECTION_PIN        AVR32_PIN_PX36

//! @}
//============================| Advanced settings |============================

/**
 * \brief Set default digital audio interface
 *
 * Options: SSC_I2S, SSC_DSP, SSC_LEFT_JUSTIFIED or SSC_RIGHT_JUSTIFIED.\n
 * Or just look to e_ssc_digital_audio_interface_t enum.
 */
#define SSC_DEFAULT_DIGITAL_AUDIO_INTERFACE     SSC_I2S


/**
 * @brief Set default data length in bits
 *
 * This is depend on actual settings, so there should be some default value\n
 * for higher layers. Recommended values are: 16, 24, 32.
 */
#define SSC_DEFAULT_DATA_LENGTH                 24


/**
 * @brief Set default frame length in bits
 * Not all bits in frame must be data. Some of them can be void, or just\n
 * dummy. So this define how long is frame, no matter how many data bits are\n
 * transmitted or received. If you doubt, just set value to 32. This should\n
 * work in most cases.
 */
#define SSC_DEFAULT_FRAME_LENGTH                32


/**
 * \brief Set default FSYNC edge for RX module
 *
 * Select FSYNC synchronization edge.\n
 * Options: SSC_FSYNC_FALLING, SSC_FSYNC_RISING, SSC_EDGE_DEFAULT (recommended)
 */
#define SSC_DEFAULT_FSYNC_RX_EDGE               SSC_EDGE_DEFAULT

/**
 * \brief Set default FSYNC edge for TX module
 *
 * Select FSYNC synchronization edge.\n
 * Options: SSC_FSYNC_FALLING, SSC_FSYNC_RISING, SSC_EDGE_DEFAULT (recommended)
 */
#define SSC_DEFAULT_FSYNC_TX_EDGE               SSC_EDGE_DEFAULT


/**
 * \brief Set default FSYNC TX MODE (when FSYNC as master)
 *
 * Options: SSC_RX (slave - FSYNC as input)\n
 * SSC_TX (master - FSYNC is generated)\n
 */
#define SSC_DEFAULT_FSYNC_ROLE                  SSC_ROLE_TX


/**
 * \brief Set default edge on which data will be sampled
 *
 * Select BCLK synchronization edge.\n
 * Options: SSC_FSYNC_FALLING, SSC_FSYNC_RISING, SSC_EDGE_DEFAULT (recommended)
 */
#define SSC_DEFAULT_BCLK_RX_EDGE                SSC_EDGE_DEFAULT

/**
 * \brief Set default edge on which data will be transmitted
 *
 * Select BCLK synchronization edge.\n
 * Options: SSC_FSYNC_FALLING, SSC_FSYNC_RISING, SSC_EDGE_DEFAULT (recommended)
 */
#define SSC_DEFAULT_BCLK_TX_EDGE                SSC_EDGE_DEFAULT

//===============================| Structures |================================

/**
 * \brief Possible return codes
 */
typedef enum{
  SSC_SUCCESS = 0,           //!< SSC_SUCCESS All OK
  SSC_FAIL = 1,              //!< SSC_FAIL Unspecified error
  SSC_INCORRECT_PARAMETER = 2//!< SSC_INCORRECT_PARAMETER Wrong input parameter
}SSC_RES_CODE;


/**
 * \brief Signal role. TX (master) or RX (slave)
 */
typedef enum{
  SSC_ROLE_RX = 0,//!< SSC_RX Slave
  SSC_ROLE_TX = 1 //!< SSC_TX Master
}e_ssc_Role_Rx_Tx_t;


/**
 * \brief Possible audio interface interface
 */
typedef enum{
  SSC_I2S = 0,           //!< SSC_I2S I2S
  SSC_DSP = 1,           //!< SSC_DSP DSP
  SSC_LEFT_JUSTIFIED = 2,//!< SSC_LEFT_JUSTIFIED Left justified
  SSC_RIGHT_JUSTIFIED =3,//!< SSC_RIGHT_JUSTIFIED Right justified
}e_ssc_digital_audio_interface_t;



typedef enum{
  SSC_EDGE_FALLING = 0,
  SSC_EDGE_RISING = 1,
  SSC_EDGE_DEFAULT = 2
}e_ssc_edge_t;

/**
 * \brief Structure, that store settings of SSC module
 */
typedef struct{
  /// Digital audio interface (I2S, DSP, ...)
  e_ssc_digital_audio_interface_t e_dig_aud_mode;

  /// Define edge on which RX data will be synchronized
  e_ssc_edge_t e_FSYNC_RX_edge;
  /// Define edge on which TX data will be synchronized
  e_ssc_edge_t e_FSYNC_TX_edge;

  /// Define edge on which RX BCLK will be synchronized
  e_ssc_edge_t e_BCLK_RX_edge;
  /// Define edge on which TX BCLK be synchronized / generated
  e_ssc_edge_t e_BCLK_TX_edge;

  /// FSYNC role (master / slave)
  e_ssc_Role_Rx_Tx_t e_FSYNC_role;

  /// Data length in bits
  uint8_t i_data_length;
  /// Frame length in bits
  uint8_t i_frame_length;
}s_ssc_settings_t;
//===============================| Definitions |===============================
/**
 * \name Error messages
 *
 * @{
 */
#define SSC_MSG_ERR_RESET_FAILED                \
  "SSC: Reset failed\n"

#define SSC_MSG_ERR_SET_DIG_INTFCE_MODE         \
  "SSC: Setting digital interface mode failed\n"

#define SSC_MSG_ERR_SSC_ENABLE_FAILED           \
  "SSC: Module enable failed\n"

#define SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED     \
  "SSC: Feature not complete yet\n"
///@}

/**
 * \name Info messages
 * @{
 */

#define SSC_MSG_INFO_SSC_INIT                   \
  "SSC: Initialization\n"
#define SSC_MSG_INFO_MODE_I2S                   \
  "SSC: Setting I2S mode\n"
///@}
//=================================| Macros |==================================

//===========================| Function prototypes |===========================
//==========================| High level functions |===========================
SSC_RES_CODE ssc_init(void);
SSC_RES_CODE ssc_reset(void);

SSC_RES_CODE ssc_set_digital_interface_mode(
    e_ssc_digital_audio_interface_t e_mode);
SSC_RES_CODE ssc_get_digital_interface_mode(
    e_ssc_digital_audio_interface_t *p_e_mode);

//===========================| Mid level functions |===========================
SSC_RES_CODE ssc_wait_for_FSYNC_RX(void);



SSC_RES_CODE ssc_set_digital_interface_mode_I2S(void);

//===========================| Low level functions |===========================
SSC_RES_CODE ssc_set_data_length(uint8_t i_data_length);
SSC_RES_CODE ssc_get_data_length(uint8_t *p_i_data_length);

SSC_RES_CODE ssc_set_frame_length(uint8_t i_frame_length);
SSC_RES_CODE ssc_get_frame_length(uint8_t *p_i_frame_length);


SSC_RES_CODE ssc_set_FSYNC_RX_edge(e_ssc_edge_t e_edge);
SSC_RES_CODE ssc_get_FSYNC_RX_edge(e_ssc_edge_t *p_e_edge);

SSC_RES_CODE ssc_set_FSYNC_TX_edge(e_ssc_edge_t e_edge);
SSC_RES_CODE ssc_get_FSYNC_TX_edge(e_ssc_edge_t *p_e_edge);

SSC_RES_CODE ssc_set_FSYNC_role(e_ssc_Role_Rx_Tx_t e_role);
SSC_RES_CODE ssc_get_FSYNC_role(e_ssc_Role_Rx_Tx_t *p_e_role);

SSC_RES_CODE ssc_set_BCLK_RX_edge(e_ssc_edge_t e_edge);
SSC_RES_CODE ssc_get_BCLK_RX_edge(e_ssc_edge_t *p_e_edge);

SSC_RES_CODE ssc_set_BCLK_TX_edge(e_ssc_edge_t e_edge);
SSC_RES_CODE ssc_get_BCLK_TX_edge(e_ssc_edge_t *p_e_edge);


SSC_RES_CODE ssc_disable_RX_TX(void);
SSC_RES_CODE ssc_enable_RX_TX(void);


#endif


