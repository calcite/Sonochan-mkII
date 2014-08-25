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
 * Modified: 25.08.2014
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
 * \brief Set default FSYNC edge
 *
 * Options: SSC_FSYNC_FALLING, SSC_FSYNC_RISING
 */
#define SSC_DEFAULT_FSYNC_EDGE                  SSC_FALLING

/**
 * \brief Set default FSYNC TX MODE (when FSYNC as master)
 *
 * Options: AVR32_SSC_TFMR_FSOS_INPUT_ONLY\n
 * AVR32_SSC_TFMR_FSOS_NEG_PULSE\n
 * AVR32_SSC_TFMR_FSOS_POS_PULSE\n
 * AVR32_SSC_TFMR_FSOS_LOW_DURING_DATA\n
 * AVR32_SSC_TFMR_FSOS_HIGH_DURING_DATA\n
 * AVR32_SSC_TFMR_FSOS_TOGGLE_DATA_START
 */
#define SSC_DEFAULT_FSYNC_TX_MODE       AVR32_SSC_TFMR_FSOS_NEG_PULSE


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
  SSC_RX = 0,//!< SSC_RX Slave
  SSC_TX = 1 //!< SSC_TX Master
}e_ssc_Role_Rx_Tx_t;


/**
 * \brief Possible audio interface interface
 */
typedef enum{
  SSC_I2S = 0,           //!< SSC_I2S I2S
  SSC_DSP = 1,           //!< SSC_DSP DSP
  SSC_LEFT_JUSTIFIED = 2,//!< SSC_LEFT_JUSTIFIED Left justified
  SSC_RIGHT_JUSTIFIED = 3//!< SSC_RIGHT_JUSTIFIED Right justified
}e_ssc_digital_audio_interface_t;



typedef enum{
  SSC_FALLING = 0,
  SSC_RISING = 1
}e_ssc_edge_t;

typedef struct{
  e_ssc_digital_audio_interface_t e_dig_aud_mode;
  e_ssc_edge_t e_FSYNC_RX_edge;
  e_ssc_edge_t e_FSYNC_TX_edge;
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



//===========================| Low level functions |===========================
SSC_RES_CODE ssc_FSYNC_RX_edge(e_ssc_edge_t e_edge);
SSC_RES_CODE ssc_FSYNC_role(e_ssc_Role_Rx_Tx_t e_role);






#endif
