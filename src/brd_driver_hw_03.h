/**
 * \file
 *
 * \brief Board driver for "Sonochan mkII" HW version 03
 *
 * Allow set basic settings on board. Also support "generic driver"
 *
 * Created:  23.04.2014\n
 * Modified: 23.04.2014\n
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
#define BRD_DRV_DEFAULT_HEADPHONES_VOLUME_IN_DB         -35

/**
 * \brief Option for generic driver support (1 -> true , 0 -> false)
 */
#define BRD_DRV_SUPPORT_GENERIC_DRIVER          1

/**
 * \name Pin mapping for BRD_DRV driver
 * @{
 */
///\todo Define pins
/// @}

/**
 * \brief Define when volume is updated (how much must be value on ADC\n
 * different from previous value on ADC)
 */
#define BRD_DRV_ADC_UPDATE_TRESHOLD             2

/**
 * \brief Define "time" in witch will be input buttons scanned
 *
 * Scan "mute" and "reset I2S"
 */
#define BRD_DRV_BUTTON_REFRESH_TIME             200


//================================| Includes |=================================
// Basic data types
#include <inttypes.h>

// For codec control
#include "tlv320aic33.h"

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
//=================================| Macros |==================================



//==========================| High level functions |===========================
GD_RES_CODE brd_drv_init(void);
//===========================| Mid level functions |===========================

//===========================| Low level functions |===========================



#endif
