/**
 * \file
 *
 * \brief Driver for SSC module on UC3A3256
 *
 * This driver "can not" have HAL, because A LOF of things are too hardware\n
 * dependent.\n
 *
 * Created:  20.08.2014\n
 * Modified: 20.08.2014
 *
 * \version 0.1
 * \author Martin Stejskal
 */

#ifndef _SSC_UC3A3256_H_
#define _SSC_UC3A3256_H_

//===========================| Included libraries |============================
#include <avr32/io.h>


/**
 * \brief TWI interface
 *
 * Aka (*((volatile avr32_twi_t*)AVR32_SSC_ADDRESS)) or (&AVR32_SSC)\n
 * Example: SSC_DEVICE (&AVR32_SSC)
 */
#define SSC_DEVICE                      (&AVR32_SSC)

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
 * \brief
 */
typedef enum{
  SSC_RX = 0,//!< SSC_RX
  SSC_TX = 1 //!< SSC_TX
}e_Role_Rx_Tx;
//===============================| Definitions |===============================

//=================================| Macros |==================================

//===========================| Function prototypes |===========================
//==========================| High level functions |===========================
SSC_RES_CODE ssc_reset(void);


//===========================| Mid level functions |===========================




//===========================| Low level functions |===========================







#endif
