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
 * Modified: 29.08.2014
 *
 * \version 0.1
 * \author Martin Stejskal
 */
#ifndef _SSC_UC3A3256_H_
#define _SSC_UC3A3256_H_

//===========================| Included libraries |============================
// Data types
#include <inttypes.h>

// AVR IO
#include <avr32/io.h>

// Use SDR-Widget GPIO library, because this library is soooo HW dependent
///\todo Try to remove this dependency
#include "gpio.h"

//=============================| Basic settings |==============================

/**
 * \brief TWI interface
 *
 * Aka (*((volatile avr32_twi_t*)AVR32_SSC_ADDRESS)) or (&AVR32_SSC)\n
 * Example: SSC_DEVICE (&AVR32_SSC)
 */
#define SSC_DEVICE                      (&AVR32_SSC)

/**
 * \brief Another external pin that allow watch FSYNC and synchronize data
 *
 * This pin MUST be used only as GPIO. No other function must not use this pin.
 */
#define SSC_EXTERNAL_FSYNC_DETECTION_PIN        AVR32_PIN_PX36

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
 * transmitted or received.
 */
#define SSC_DEFAULT_FRAME_LENGTH                32


/**
 * \brief Set default FSYNC edge
 *
 * Select FSYNC synchronization edge.\n
 * Options: SSC_FSYNC_FALLING, SSC_FSYNC_RISING, SSC_EDGE_DEFAULT (recommended)
 */
#define SSC_DEFAULT_FSYNC_EDGE                  SSC_EDGE_DEFAULT

/**
 * \brief Set default FSYNC TX MODE (when FSYNC as master)
 *
 * Options: SSC_RX (slave - FSYNC as input)\n
 * SSC_TX (master - FSYNC is generated)\n
 */
#define SSC_DEFAULT_FSYNC_ROLE       SSC_ROLE_TX


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
//=================================| Macros |==================================

//===========================| Function prototypes |===========================
//==========================| High level functions |===========================
SSC_RES_CODE ssc_init(void);
SSC_RES_CODE ssc_reset(void);


//===========================| Mid level functions |===========================
SSC_RES_CODE ssc_wait_for_FSYNC_RX(void);

SSC_RES_CODE ssc_set_digital_interface_mode(
    e_ssc_digital_audio_interface_t e_mode);
SSC_RES_CODE ssc_get_digital_interface(
    e_ssc_digital_audio_interface_t *p_e_mode);

SSC_RES_CODE ssc_set_digital_interface_mode_I2S(void);

//===========================| Low level functions |===========================
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
