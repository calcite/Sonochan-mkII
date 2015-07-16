/**
 * \file
 *
 * \brief Driver for fractional PLL CS2200
 *
 * Created:  2014.03.12\n
 * Modified: 2015.07.15
 *
 * \version 0.7.4
 * \author Martin Stejskal
 */


#ifndef _CS2200_H_
#define _CS2200_H_

//===========================| Included libraries |============================
/* HAL for PLL !!! Please include correct driver for used architecture!
 * Also in included driver should be defined enumeration cs2200_status_t
 */
#include "cs2200_HAL_AVR32_UC3A3_HW_interface.h"

//=================================| Options |=================================
/**
 * \name Basic CS2200 settings
 *
 * When options are 1 (true) and 0 (false) it is highly recommended use\n
 * 1 or 0 instead of true and false
 *
 * @{
 */

/**
 * \brief Reference clock frequency
 *
 * Define reference clock connected to XTI/REF_CLK. There can be connected\n
 * crystal or external clock. Anyway, if there is variable or unknown clock\n
 * source, then this value should be set to 0. When is set to 0, function\n
 * cs2200_set_PLL_freq do not recalculate frequency and just send argument\n
 * value directly to PLL. So this is alternative way to set frequency.\n
 * Supported frequencies: 8~14 MHz, 16~28 Mhz and 32~56 MHz (50 MHz if used\n
 * crystal)\n
 * Example:\n
 * \code
 * #define CS2200_REF_CLK_FREQ  12000000UL
 * \endcode
 */
#define CS2200_REF_CLK_FREQ             12000000UL

/**
 * \brief Support for generic driver support
 *
 * Options: 1 (support enabled) or 0 (support disabled)
 *
 */
#define CS2200_SUPPORT_GENERIC_DRIVER   1
/// @}


/**
 * \name Advanced CS2200 settings
 *
 * @{
 */
/**
 * \brief Allow enable FreeRTOS features
 *
 * Options: 0 (disabled) or 1 (enabled). When enabled, then FreeRTOS features\n
 * are enabled.
 */
#define CS2200_SUPPORT_RTOS             1

/**
 * \brief Allow create mutex mutexI2C when cs2200_init is called.
 *
 * Mutexes is needed when in RTOS two functions share same hardware. This\n
 * option allow create mutex mutexI2C, so any other functions just can use\n
 * xSemaphoreTake and xSemaphoreGive functions.\n
 * Options: 0 (disabled) or 1 (enabled).\n
 *
 * \note This option is valid only if RTOS support is enabled.
 */
#define CS2200_RTOS_CREATE_MUTEX        1

/// @}


//===============================| Structures |================================

/**
 * \brief Symbolic values for different input dividers
 *
 * Because we have only this options it is good idea use enum when selecting\n
 * input divider.
 */
typedef enum{
  CS2200_INPUT_DIVIDER_1x = 2,//!< CS2200_INPUT_DIVIDER_1x
  CS2200_INPUT_DIVIDER_2x = 1,//!< CS2200_INPUT_DIVIDER_2x
  CS2200_INPUT_DIVIDER_4x = 0 //!< CS2200_INPUT_DIVIDER_4x
} cs2200_in_div_t;

/**
 * \brief Symbolic values for R-Mod selection
 *
 * Because we have only these options it is advantageous use enum when\n
 * selecting one of the option.
 */
typedef enum{
  CS2200_R_MOD__MUL_1x = 0,
  CS2200_R_MOD__MUL_2x = 1,
  CS2200_R_MOD__MUL_4x = 2,
  CS2200_R_MOD__MUL_8x = 3,
  CS2200_R_MOD__DIV_2x = 4,
  CS2200_R_MOD__DIV_4x = 5,
  CS2200_R_MOD__DIV_8x = 6,
  CS2200_R_MOD__DIV_16x= 7
} cs2200_r_mod_t;

/**
 * \brief Structure that allow split 32 bit value into 8 bit and vice versa
 */
typedef union{
  uint32_t i_32bit;
  uint8_t  i_8bit[4];
} cs2200_32b_to_8b_t;


/**
 * \name Structures for CS2200 registers
 *
 * This structures allow write bit by bit to register image, Also save memory.
 * @{
 */

// Note, that this is endian (and compiler) dependent. So at least try...
#if defined(__ORDER_BIG_ENDIAN__) || defined(__AVR32__)
typedef struct device_id
{
  uint8_t Device        :5;     // MSB
  uint8_t Revision      :3;
}device_id_t;

typedef struct device_Ctrl_t
{
  uint8_t Unlock        :1;
  uint8_t               :5;     // 5 Reserved bits - blank space
  uint8_t AuxOutDis     :1;
  uint8_t ClkOutDis     :1;
} device_ctrl_t;

typedef struct device_Cfg_1_t
{
  uint8_t RModSel       :3;
  uint8_t               :2;     // Space
  uint8_t AuxOutSrc     :2;
  uint8_t EnDevCfg1     :1;

} device_Cfg_1_t;

typedef struct global_Cfg_t
{
  uint8_t               :4;     // Space
  uint8_t Freeze        :1;
  uint8_t               :2;     // Space
  uint8_t EnDevCfg2     :1;
} global_Cfg_t;

typedef struct func_Cfg_1_t
{
  uint8_t               :1;     // Space
  uint8_t AuxLockCfg    :1;
  uint8_t               :1;     // Space
  uint8_t RefClkDiv     :2;
  uint8_t               :3;     // Space
} func_Cfg_1_t;

typedef struct func_Cfg_2_t
{
  uint8_t               :3;     // Space
  uint8_t ClkOutUnl     :1;
  uint8_t               :4;     // Space
} func_Cfg_2_t;
#else   // Little Endian
typedef struct device_id
{
  uint8_t Revision      :3;     // LSB
  uint8_t Device        :5;
}device_id_t;

typedef struct device_Ctrl_t
{
  uint8_t ClkOutDis     :1;
  uint8_t AuxOutDis     :1;
  uint8_t               :5;     // 5 Reserved bits - blank space
  uint8_t Unlock        :1;
} device_ctrl_t;

typedef struct device_Cfg_1_t
{
  uint8_t EnDevCfg1     :1;
  uint8_t AuxOutSrc     :2;
  uint8_t               :2;     // Space
  uint8_t RModSel       :3;

} device_Cfg_1_t;

typedef struct global_Cfg_t
{
  uint8_t EnDevCfg2     :1;
  uint8_t               :2;     // Space
  uint8_t Freeze        :1;
  uint8_t               :4;     // Space
} global_Cfg_t;

typedef struct func_Cfg_1_t
{
  uint8_t               :3;     // Space
  uint8_t RefClkDiv     :2;
  uint8_t               :1;     // Space
  uint8_t AuxLockCfg    :1;
  uint8_t               :1;     // Space

} func_Cfg_1_t;

typedef struct func_Cfg_2_t
{
  uint8_t               :4;     // Space
  uint8_t ClkOutUnl     :1;
  uint8_t               :3;     // Space
} func_Cfg_2_t;
#endif



///@}


/**
 * \brief Register image of CS2200
 *
 * Because in one register is many independent bits, we must know every\n
 * single bit state when writing value to CS2200's register. So for this is\n
 * there this image which help keep collected data.
 */
typedef struct{
  union
  {
    uint8_t device_id_reg;
    device_id_t Device_id;
  };

  union
  {
    uint8_t device_crtl_reg;
    device_ctrl_t Device_Ctrl;
  };

  union
  {
    uint8_t device_cfg_1_reg;
    device_Cfg_1_t Device_Cfg_1;
  };

  union
  {
    uint8_t global_cfg_reg;
    global_Cfg_t Global_Cfg;
  };

  // A little bit special. used as union, that allow easy operate with bytes
  cs2200_32b_to_8b_t Ratio;

  union
  {
    uint8_t func_cfg_1_reg;
    func_Cfg_1_t Func_Cfg_1;
  };

  union
  {
    uint8_t func_cfg_2_reg;
    func_Cfg_2_t Func_Cfg_2;
  };
} cs2200_reg_img_t;


/**
 * \brief Virtual settings variables
 *
 * Because some variables are not directly represented in registers, so there\n
 * is it virtual structure which represent some values more human readable.
 */
typedef struct{
  // Frequency as integer value
  cs2200_32b_to_8b_t i_real_freq;

  // Output divider/multiplier as float
  float f_out_div_mul;

  /* When frequency is changing, PLL is preset to set PLL_OUT to LOW level
   * and wait until is locked, because if PLL is not locked, output is
   * undefined. However if we want change ratio by 1, there is big chance that
   * PLL output will work as expected. So there is option to override this
   * default settings only for functions that change ratio by 1.
   * Options:
   *  1 - save mode (when PLL is not locked -> low level)
   *  0 - continuous clock (but PLL output might be undefined)
   */
  uint8_t i_save_change_ratio_by_1;

  // Simple access to freeze bit (mainly for generic driver)
  uint8_t i_freeze_flag;
} cs2200_virtual_reg_img_t;

//===========================| Additional includes |===========================
#if CS2200_SUPPORT_GENERIC_DRIVER != 0
// If used generic driver, get actual structures
#include "generic_driver.h"
// Also say compiler, that there exist settings on flash
extern const gd_config_struct CS2200_config_table[];
// And metadata
extern const gd_metadata CS2200_metadata;

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



#if CS2200_SUPPORT_RTOS != 0
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern xSemaphoreHandle mutexI2C;
#endif

//===============================| Definitions |===============================
/**
 * \name CS2200 register addresses
 *
 * There are symbolic names for registers which CS2200 have.
 *
 * @{
 */
/// \brief Device I.D. and Revision
#define CS2200_REG_DEVICE_ID            0x1
/// \brief Device Control
#define CS2200_REG_DEVICE_CTRL          0x2
/// \brief Device configuration 1
#define CS2200_REG_DEVICE_CFG_1         0x3
/// \brief Global configuration
#define CS2200_REG_GLOBAL_CFG           0x5
/**
 * \brief Ratio
 * Ratio is 32 bit wide, so this is just address of first (MSB) register
 */
#define CS2200_REG_RATIO                0x6
/// \brief Function configuration 1
#define CS2200_REG_FUNC_CFG_1           0x16
/// \brief Function configuration 2
#define CS2200_REG_FUNC_CFG_2           0x17
/// @}


/**
 * @brief Minimum frequency that PLL can generate
 *
 * Please DO NOT CHANGE!!! This value is depend on HW and it is defined in\n
 * datasheet. Also higher layers can use this value.
 */
#define CS2200_MIN_FREQ                 6000000UL
/**
 * @brief Maximum frequency that PLL can generate
 *
 * Please DO NOT CHANGE!!! This value is depend on HW and it is defined in\n
 * datasheet. Also higher layers can use this value.
 */

#define CS2200_MAX_FREQ                 75000000UL


///\brief Auto increment bit position in address byte
#define CS2200_MAP_AUTO_INC_BIT         7

//=================================| Macros |==================================

/**
 * \brief Define lock function when support for RTOS is enabled
 *
 * If CS2200_SUPPORT_RTOS is enabled, then there is defined function, that\n
 * "lock" TWI device, so any other device can not use it.\n
 * If CS2200_SUPPORT_RTOS is disabled, there is defined just void macro.
 */
#if CS2200_SUPPORT_RTOS != 0
  #define CS2200_LOCK_TWI_MODULE_IF_RTOS    \
    xSemaphoreTake( mutexI2C, portMAX_DELAY );
#else
  #define CS2200_LOCK_TWI_MODULE_IF_RTOS
#endif


#if CS2200_SUPPORT_RTOS != 0
  #define CS2200_UNLOCK_TWI_MODULE_IF_RTOS        \
    xSemaphoreGive( mutexI2C );
#else
  #define CS2200_UNLOCK_TWI_MODULE_IF_RTOS
#endif

//==========================| High level functions |===========================
GD_RES_CODE cs2200_init(void);

GD_RES_CODE cs2200_set_PLL_freq(uint32_t i_freq);

GD_RES_CODE cs2200_get_PLL_freq(uint32_t *p_i_freq);

GD_RES_CODE cs2200_inc_PLL_freq(void);

GD_RES_CODE cs2200_dec_PLL_freq(void);

GD_RES_CODE cs2200_set_out_divider_multiplier_float(float f_div_mul);

GD_RES_CODE cs2200_get_out_divider_multiplier_float(float *p_div_mul);

GD_RES_CODE cs2200_set_out_divider_multiplier(cs2200_r_mod_t e_out_div_mul);

GD_RES_CODE cs2200_get_out_divider_multiplier(cs2200_r_mod_t *p_e_out_div_mul);

GD_RES_CODE cs2200_set_safe_change_ratio_by_1(uint8_t i_safe_change);

GD_RES_CODE cs2200_get_safe_change_ratio_by_1(uint8_t *p_i_safe_change);
//===========================| Mid level functions |===========================
GD_RES_CODE cs2200_set_ratio(uint32_t i_ratio);

GD_RES_CODE cs2200_get_ratio(uint32_t *p_i_ratio);

GD_RES_CODE cs2200_set_freeze_bit(uint8_t i_freeze_flag);

GD_RES_CODE cs2200_get_freeze_bit(uint8_t *p_i_freeze_flag);

GD_RES_CODE cs2200_enable_device_cfg(void);

GD_RES_CODE cs2200_disable_device_cfg(void);

GD_RES_CODE cs2200_set_in_divider(cs2200_in_div_t e_in_div);

GD_RES_CODE cs2200_get_in_divider(cs2200_in_div_t *p_e_in_div);
//===========================| Low level functions |===========================
GD_RES_CODE cs2200_calc_frequency(uint32_t i_ratio, uint32_t *p_i_frequency);

GD_RES_CODE cs2200_calc_ratio(uint32_t i_frequency, uint32_t *p_i_ratio);

GD_RES_CODE cs2200_write_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes);

GD_RES_CODE cs2200_read_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes);
#endif
