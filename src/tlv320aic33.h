/**
 * \file
 *
 * \brief Driver for codec TLV320AIC33
 *
 * Created:  02.04.2014\n
 * Modified: 02.04.2014
 *
 * \version 0.1
 * \author Martin Stejskal
 */

#ifndef _TLV320AIC33_H_
#define _TLV320AIC33_H_

//===========================| Included libraries |============================
/* HAL for PLL !!! Please include correct driver for used architecture!
 * Also in included driver should be defined enumeration TLV320AIC33_status_t
 */
#include "tlv320aic33_HAL_AVR32_UC3A3_HW_interface.h"

//=================================| Options |=================================

/**
 * \name Basic TLV320AIC33 settings
 *
 * When options are 1 (true) and 0 (false) it is highly recommended use\n
 * 1 or 0 instead of true and false
 *
 * @{
 */

/**
 * \brief Support for generic driver support
 *
 * Options: 1 (support enabled) or 0 (support disabled)
 *
 */
#define TLV320AIC33_SUPPORT_GENERIC_DRIVER   0
/// @}


/**
 * \name Advanced TLV320AIC33 settings
 *
 * @{
 */
/**
 * \brief Allow enable FreeRTOS features
 *
 * Options: 0 (disabled) or 1 (enabled). When enabled, then FreeRTOS features\n
 * are enabled.
 */
#define TLV320AIC33_SUPPORT_RTOS             1

/**
 * \brief Allow create mutex mutexI2C when TLV320AIC33_init is called.
 *
 * Mutexes is needed when in RTOS two functions share same hardware. This\n
 * option allow create mutex mutexI2C, so any other functions just can use\n
 * xSemaphoreTake and xSemaphoreGive functions.\n
 * Options: 0 (disabled) or 1 (enabled).\n
 *
 * \note This option is valid only if RTOS support is enabled.
 */
#define TLV320AIC33_RTOS_CREATE_MUTEX        0

/// @}

//===============================| Structures |================================
//===========================| Additional includes |===========================
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER == 1
// If used generic driver, get actual structures
#include "generic_driver.h"
// Also say compiler, that there exist settings on flash
extern const gd_config_struct TLV320AIC33_config_table[];
// And metadata
extern const gd_metadata TLV320AIC33_metadata;

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

#if TLV320AIC33_SUPPORT_RTOS != 0
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern xSemaphoreHandle mutexI2C;

#endif

//===============================| Definitions |===============================
//=================================| Macros |==================================

/**
 * \brief Define lock function when support for RTOS is enabled
 *
 * If TLV320AIC33_SUPPORT_RTOS is enabled, then there is defined function, that\n
 * "lock" TWI device, so any other device can not use it.\n
 * If TLV320AIC33_SUPPORT_RTOS is disabled, there is defined just void macro.
 */
#if TLV320AIC33_SUPPORT_RTOS != 0
  #define TLV320AIC33_LOCK_TWI_MODULE_IF_RTOS    \
    xSemaphoreTake( mutexI2C, portMAX_DELAY );
#else
  #define TLV320AIC33_LOCK_TWI_MODULE_IF_RTOS
#endif


#if TLV320AIC33_SUPPORT_RTOS != 0
  #define TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS        \
    xSemaphoreGive( mutexI2C );
#else
  #define TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS
#endif



//==========================| High level functions |===========================
GD_RES_CODE tlv320aic33_init(void);
//===========================| Mid level functions |===========================

//===========================| Low level functions |===========================
GD_RES_CODE tlv320aic33_write_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes);

GD_RES_CODE tlv320aic33_read_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes);


#endif
