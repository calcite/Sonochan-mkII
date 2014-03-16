/**
 * \file
 *
 * \brief Driver for fractional PLL CS2200
 *
 * Created:  12.03.2014\n
 * Modified: 16.03.2014
 *
 * \version 0.1
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
 * Options: 1 (support enabled) or 0 (support disabled) (recommended)
 *
 * \note Support is not even in testing mode! Do not turn on unless you\n
 * testing software!
 */
#define CS2200_SUPPORT_GENERIC_DRIVER   0
/// @}


/**
 * \name Advanced CS2200 settings
 *
 * @{
 */

/// @}


//===============================| Structures |================================

/**
 * \brief Symbolic values for different input dividers
 *
 * Because we have only this options it is good idea use enum when selecting\n
 * input divider.
 */
typedef enum{
  CS2200_INPUT_DIVIDER_1x = 2,
  CS2200_INPUT_DIVIDER_2x = 1,
  CS2200_INPUT_DIVIDER_4x = 0
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
typedef struct device_id
{
  uint8_t Device        :5;
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


//===========================| Additional includes |===========================
#if CS2200_SUPPORT_GENERIC_DRIVER == 1
// If used generic driver, get actual structures
#include "ALPS_generic_driver.h"
// Also say compiler, that there exist settings on flash
extern const gd_config_struct CS2200_config_table_flash[] PROGMEM;
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
 * \name Bit positions and masks in CS2200 registers
 *
 * @{
 */
/// \brief Device identification bit position
#define CS2200_DEVICE_BIT               3
/// \brief Device identification mask
#define CS2200_DEVICE_MASK              0x1F

///\brief Device revision bit position
#define CS2200_REVID_BIT                0
///\brief Device revision mask
#define CS2200_REVID_MASK               0x7

///\brief Unlock indicator bit position
#define CS2200_UNLOCK_BIT               7
///\brief Unlock indicator mask
#define CS2200_UNLOCK_MASK              0x1

///\brief Auxiliary output disable bit position
#define CS2200_AUXOUTDIS_BIT            1
///\brief Auxiliary output disable mask
#define CS2200_AUXOUTDIS_MASK           0x1

///\brief PLL clock output disable bit position
#define CS2200_CLKOUTDIS_BIT            0
///\brief PLL clock output disable mask
#define CS2200_CLKOUTDIS_MASK           0x1

///\brief R-Mod selection bit position
#define CS2200_RMODSEL_BIT              5
///\brief R-Mod selection mask
#define CS2200_RMODSEL_MASK             0x7

///\brief Auxiliary output source selection bit position
#define CS2200_AUXOUTSRC_BIT            1
///\brief Auxiliary output source selection mask
#define CS2200_AUXOUTSRC_MASK           0x3

///\brief Enable device configuration registers 1 bit position
#define CS2200_ENDEVCFG1_BIT            0
///\brief Enable device configuration registers 1 mask
#define CS2200_ENDEVCFG1_MASK           0x1

///\brief Device configuration freeze bit position
#define CS2200_FREEZE_BIT               3
///\brief Device configuration freeze mask
#define CS2200_FREEZE_MASK              0x1

///\brief Enable device configuration registers 2 bit position
#define CS2200_ENDEVCFG2_BIT            0
///\brief Enable device configuration registers 2 mask
#define CS2200_ENDEVCFG2_MASK           0x1

///\brief AUX PLL lock output configuration bit position
#define CS2200_AUXLOCKCFG_BIT           6
///\brief AUX PLL lock output configuration mask
#define CS2200_AUDLOCKCFG_MASK          0x1

///\brief Reference clock input divider bit position
#define CS2200_REFCLKDIV_BIT            3
///\brief Reference clock input divider
#define CS2200_REFCLKDIV_MASK           0x3

///\brief Enable PLL clock output on unlock bit position
#define CS2200_CLKOUTUNL_BIT            4
///\brief Enable PLL clock output on unlock mask
#define CS2200_CLKOUTUNL_MASK           0x1
/// @}

///\brief Auto increment bit position in address byte
#define CS2200_MAP_AUTO_INC_BIT         7

//=================================| Macros |==================================


//==========================| High level functions |===========================
GD_RES_CODE cs2200_init(void);

GD_RES_CODE cs2200_set_PLL_freq(uint32_t i_freq);

GD_RES_CODE cs2200_set_out_divider_multiplier(cs2200_r_mod_t e_out_div_mul);
//===========================| Low level functions |===========================
GD_RES_CODE cs2200_set_freeze_bit(uint8_t i_freeze_flag);

GD_RES_CODE cs2200_enable_device_cfg(void);

GD_RES_CODE cs2200_disable_device_cfg(void);

GD_RES_CODE cs2200_set_in_divider(cs2200_in_div_t e_in_div);
#endif
