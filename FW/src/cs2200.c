/**
 * \file
 *
 * \brief Driver for fractional PLL CS2200
 *
 * Created:  12.03.2014\n
 * Modified: 20.08.2014
 *
 * \version 0.7.1
 * \author Martin Stejskal
 */

#include "cs2200.h"
//============================| Global variables |=============================
/**
 * \brief Register image of CS2200.
 *
 * It is static, so it is "global" only for this file.
 *
 */
static cs2200_reg_img_t s_register_img;

/**
 * \brief Virtual register image
 *
 * This values PLL directly do not have in memory. However, there are in more\n
 * human readable format. Also some values can use generic driver (need to\n
 * point to result variables and so on).\n
 * t is static, so it is "global" only for this file.
 */
static cs2200_virtual_reg_img_t s_virtual_reg_img;



//=========================| Generic driver support |==========================
#if CS2200_SUPPORT_GENERIC_DRIVER != 0
/**
 * \brief Configure table for device
 */

// Test architecture (AVR8 versus other)
#ifdef __AVR_ARCH__
const gd_config_struct CS2200_config_table[] PROGMEM =
#else
const gd_config_struct CS2200_config_table[] =
#endif
  {
    {
      0,                      // Command ID
      "Initialize CS2200 hardware",  // Name
      "Initialize I/O and TWI module (if used)",      // Descriptor
      void_type,              // Input data type
      {.data_uint32 = 0},     // Minimum input value
      {.data_uint32 = 0},     // Maximum input value
      void_type,              // Output data type
      {.data_uint32 = 0},     // Minimum output value
      {.data_uint32 = 0},     // Maximum output value
      (GD_DATA_VALUE*)&gd_void_value, // Output value
      cs2200_init                     /* Function, that should be
                                       * called
                                       */
    },
    {
      1,
      "Set PLL frequency",
      "Frequency format: unsigned 32 bit in Hz",
      uint32_type,
      {.data_uint32 =  6000000},        // Min 6 MHz
      {.data_uint32 = 75000000},        // Max 75 MHz
      uint32_type,
      {.data_uint32 =  6000000},
      {.data_uint32 = 75000000},
      (GD_DATA_VALUE*)&s_virtual_reg_img.i_real_freq.i_32bit,
      cs2200_set_PLL_freq
    },
    {
      2,
      "Increase PLL frequency by 1",
      "Increase ratio number by 1",
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      (GD_DATA_VALUE*)&gd_void_value,
      cs2200_inc_PLL_freq
    },
    {
      3,
      "Decrease PLL frequency by 1",
      "Decrease ratio number by 1",
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      (GD_DATA_VALUE*)&gd_void_value,
      cs2200_dec_PLL_freq
    },
    {
      4,
      "Set ratio",
      "Unsigned 32 bit raw value",
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0xFFFFFFFF},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0xFFFFFFFF},
      (GD_DATA_VALUE*)&s_register_img.Ratio.i_32bit,
      cs2200_set_ratio
    },
    {
      5,
      "Set output divider/multiplier ratio",
      "Input values: 1 2 4 8 0.5 0.25 0.125 0.0625",
      float_type,
      {.data_float = 0.0625},
      {.data_float = 8},
      float_type,
      {.data_float = 0.0625},
      {.data_float = 8},
      (GD_DATA_VALUE*)&s_virtual_reg_img.f_out_div_mul,
      cs2200_set_out_divider_multiplier_float
    },
    {
      6,
      "Set safe change flag (frequency change)",
      "Options: 1 - safe (recommended) ; 0 - dangerous change (may cause glitches)",
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 1},
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 1},
      (GD_DATA_VALUE*)&s_virtual_reg_img.i_save_change_ratio_by_1,
      cs2200_set_save_change_ratio_by_1
    },
    {
      7,
      "Set (1) or clear (0) freeze bit",
      "Freeze bit allow configure PLL while changes are not applied",
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 1},
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 1},
      (GD_DATA_VALUE*)&s_virtual_reg_img.i_freeze_flag,
      cs2200_set_freeze_bit
    }
  };
/// \brief Maximum command ID (is defined by last command)
#define CS2200_MAX_CMD_ID          7


const gd_metadata CS2200_metadata =
{
        CS2200_MAX_CMD_ID,              // Max CMD ID
        "Fractional PLL CS2200-CP v0.7.1",     // Description
        (gd_config_struct*)&CS2200_config_table[0],
        0x21    // Serial number (0~255)
};
#endif


//=============================| FreeRTOS stuff |==============================
// If RTOS support is enabled, create this
#if CS2200_SUPPORT_RTOS != 0
portBASE_TYPE xStatus;
xSemaphoreHandle mutexI2C;
#endif

//==========================| High level functions |===========================

/**
 * \brief Initialize CS2200
 *
 * Must be called \b before any \b other \b function from this library!
 *
 * @return GD_SUCCESS if all right
 */
GD_RES_CODE cs2200_init(void)
{
  // If RTOS support enable and create flag is set then create mutex
#if (CS2200_SUPPORT_RTOS != 0) && (CS2200_RTOS_CREATE_MUTEX != 0)
  mutexI2C = xSemaphoreCreateMutex();
#endif


  // Lock TWI module if RTOS used
  CS2200_LOCK_TWI_MODULE_IF_RTOS
  // Initialize low-level driver and test status
  if(cs2200_HAL_init() != CS2200_OK)
  {
    /* If not OK (do not care about error), just return FAIL (because of limit
     * return values GD_RES_CODE). Also unlock TWI module
     */
    CS2200_UNLOCK_TWI_MODULE_IF_RTOS
    return GD_FAIL;
  }
  // Unlock TWI module
  CS2200_UNLOCK_TWI_MODULE_IF_RTOS

  // Clean image register. Exclude register Device ID
  s_register_img.device_crtl_reg = 0;
  s_register_img.device_cfg_1_reg = 0;
  s_register_img.global_cfg_reg = 0;
  s_register_img.Ratio.i_32bit = 0;     // Set ratio to 0
  s_register_img.func_cfg_1_reg = 0;
  s_register_img.func_cfg_2_reg = 0;

  // Frequency not set yet
  s_virtual_reg_img.i_real_freq.i_32bit = 0;
  // Multiplier not set yet (1x)
  s_virtual_reg_img.f_out_div_mul = 1;
  // Save mode. PLL output will be in LOW when changing ratio by 1
  s_virtual_reg_img.i_save_change_ratio_by_1 = 1;
  // Freeze bit - by default is set to 0
  s_virtual_reg_img.i_freeze_flag = 0;

  // Create status variable
  GD_RES_CODE e_status;

  // Create TX buffer
  uint8_t i_tx_buffer[2];

  // Set MAP address
  i_tx_buffer[0] = CS2200_REG_DEVICE_ID;

  /* Now load Device ID register from PLL. Send memory address from which we
   * will read.
   * Write just one byte.
   */
  e_status = cs2200_write_data(&i_tx_buffer[0], 1);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // OK, now read value from PLL directly to structure
  e_status = cs2200_read_data(&(s_register_img.device_id_reg), 1);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // If we know input clock, then set divider
#if CS2200_REF_CLK_FREQ != 0
  // Check for too sloooow clock
#if CS2200_REF_CLK_FREQ < 8000000UL
  /* Lower than 8 MHz -> it should not be. Anyway, set prescaller to 1 and
   * write warning.
   */
#warning "CS2200_REF_CLK_FREQ is lower than 8 MHz! PLL should operate in range\
  from 8 MHz up to 56 MHz (50 MHz if used crystal). Please consider source\
  with higher frequency. Prescaller set to 1x."
  return cs2200_set_in_divider(CS2200_INPUT_DIVIDER_1x);
#elif CS2200_REF_CLK_FREQ <= 14000000UL
  // Set divider to 1x
  return cs2200_set_in_divider(CS2200_INPUT_DIVIDER_1x);
#elif CS2200_REF_CLK_FREQ < 16000000UL
  // 14 - 16 MHz is not directly supported
#warning "CS2200_REF_CLK_FREQ is between 14 and 16 MHz. These frequencies are\
  not directly supported by PLL! Prescaller set to 1x."
  return cs2200_set_in_divider(CS2200_INPUT_DIVIDER_1x);
#elif CS2200_REF_CLK_FREQ <= 28000000UL
  return cs2200_set_in_divider(CS2200_INPUT_DIVIDER_2x);
#elif CS2200_REF_CLK_FREQ < 32000000UL
#warning "CS2200_REF_CLK_FREQ is between 14 and 32 MHz. These frequencies are\
  not directly supported by PLL! Prescaller se to 2x"
  return cs2200_set_in_divider(CS2200_INPUT_DIVIDER_2x);
#elif CS2200_REF_CLK_FREQ <= 56000000UL
  return cs2200_set_in_divider(CS2200_INPUT_DIVIDER_4x);
#else
#warning "CS2200_REF_CLK_FREQ is higher than 56 MHz. PLL should operate in\
  range from 8 MHz up to 56 MHz (50 MHz if used crystal). Please consider\
  source with lower frequency. Prescaller set to 4x."
  return cs2200_set_in_divider(CS2200_INPUT_DIVIDER_4x);
#endif
#endif

}



/**
 * \brief Set PLL to get user defined frequency
 *
 * If CS2200_REF_CLK_FREQ is 0, then just send data (i_freq argument) to PLL\n
 * and activate values. Also set output divider/multiplier back to 1 to make\n
 * sure that output frequency is correct.
 *
 * @param i_freq Frequency which user want. In case, that CS2200_REF_CLK_FREQ\n
 * is 0, then no calculation is done and data are send directly to PLL.
 *
 * @return GD_SUCCESS if all right
 */
GD_RES_CODE cs2200_set_PLL_freq(uint32_t i_freq)
{
  // Temporary 32 bit value
  uint32_t i_ratio;

  // Create status variable
  GD_RES_CODE e_status;


  /* Preprocessor test if user want just dummy set value or if want calculate
   * frequency
   */
#if CS2200_REF_CLK_FREQ != 0
  /* Calculate ratio form frequency. Result will be saved to u_tmp as 32 bit
   * value
   */
  e_status = cs2200_calc_ratio(i_freq, &i_ratio);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }
#else   // When CLK is not defined
  // i_freq save to union structure -> easy to split into bytes
  u_tmp32.i_32bit = i_freq;
#endif


  // Set freeze bit if not set (to avoid glitches when configuration is enabled
  if(s_register_img.Global_Cfg.Freeze == 0)
  {
    // Try to set freeze bit
    e_status = cs2200_set_freeze_bit(1);
    if(e_status != GD_SUCCESS)
    {
      // If configuration failed
      return e_status;
    }
  }

  // Set ratio
  e_status = cs2200_set_ratio(i_ratio);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Disable divider
  e_status = cs2200_set_out_divider_multiplier(CS2200_R_MOD__MUL_1x);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Disable freeze
  e_status = cs2200_set_freeze_bit(0);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }


  // Check if is needed to activate configuration
  if((s_register_img.Device_Cfg_1.EnDevCfg1 == 0) ||
     (s_register_img.Global_Cfg.EnDevCfg2 == 0))
  {     // Not set - need to set
    // If all OK, then just activate configuration
    return cs2200_enable_device_cfg();
  }
  // Else is already set. Do not need to set again
  return GD_SUCCESS;
}


/**
 * \brief Load set PLL frequency
 * @param p_i_freq Pointer to memory where frequency will be written
 * @return GD_SUCCESS if all right
 */
inline GD_RES_CODE cs2200_get_PLL_freq(uint32_t *p_i_freq)
{
  *p_i_freq = s_virtual_reg_img.i_real_freq.i_32bit;

  return GD_SUCCESS;
}



/**
 * \brief Increase PLL frequency by 1 LSB in ratio
 *
 * Increase PLL output frequency approximately by 10 ppm
 * @return GD_SUCCESS if all right
 */
GD_RES_CODE cs2200_inc_PLL_freq(void)
{
  // For saving e_status. Define default value (just for case)
  GD_RES_CODE e_status = GD_FAIL;

  /* Try to set new ratio value. Function cs2200_set_ratio() do all checks and
   * if there is no problem, then update ratio and frequency value.
   */

  // Test if we want save, or "dangerous" frequency change
  if(s_virtual_reg_img.i_save_change_ratio_by_1 == 0)
  {
    // If user want change PLL out immediately, set output as always enabled
    s_register_img.Func_Cfg_2.ClkOutUnl = 1;

    // TX buffer - register and value
    uint8_t i_tx_buffer[] = {
        CS2200_REG_FUNC_CFG_2,
        s_register_img.func_cfg_2_reg};

    e_status = cs2200_write_data(&i_tx_buffer[0], 2);
    if(e_status != GD_SUCCESS)
    {
      return e_status;
    }

    // Now change ratio
    e_status = cs2200_set_ratio(s_register_img.Ratio.i_32bit +1);
    if(e_status != GD_SUCCESS)
    {
      return e_status;
    }

    // Clear "dangerous" bit back to zero
    s_register_img.Func_Cfg_2.ClkOutUnl = 0;
    i_tx_buffer[1] = s_register_img.func_cfg_2_reg;     // Change just value

    e_status = cs2200_write_data(&i_tx_buffer[0], 2);
    if(e_status != GD_SUCCESS)
    {
      /* Because in this phase we do not know if bit was cleared or not,
       * let's set it in the image register so user should note it
       */
      s_register_img.Func_Cfg_2.ClkOutUnl = 1;
      return e_status;
    }

    // Jump to "return e_status;"
  }
  else
  {
    // Else user want increase ratio by save way
    return cs2200_set_ratio(s_register_img.Ratio.i_32bit +1);
  }
  return e_status;
}



/**
 * \brief Decrease PLL frequency by 1 LSB in ratio
 *
 * Decrease PLL output frequency approximately by 10 ppm
 * @return GD_SUCCESS if all right
 */
GD_RES_CODE cs2200_dec_PLL_freq(void)
{
  // For saving e_status. Define default value (just for case)
  GD_RES_CODE e_status = GD_FAIL;

  /* Try to set new ratio value. Function cs2200_set_ratio() do all checks and
   * if there is no problem, then update ratio and frequency value.
   */

  // Test if we want save, or "dangerous" frequency change
  if(s_virtual_reg_img.i_save_change_ratio_by_1 == 0)
  {
    // If user want change PLL out immediately, set output as always enabled
    s_register_img.Func_Cfg_2.ClkOutUnl = 1;

    // TX buffer - register and value
    uint8_t i_tx_buffer[] = {
        CS2200_REG_FUNC_CFG_2,
        s_register_img.func_cfg_2_reg};

    e_status = cs2200_write_data(&i_tx_buffer[0], 2);
    if(e_status != GD_SUCCESS)
    {
      return e_status;
    }

    // Now change ratio
    e_status = cs2200_set_ratio(s_register_img.Ratio.i_32bit -1);
    if(e_status != GD_SUCCESS)
    {
      return e_status;
    }

    // Clear "dangerous" bit back to zero
    s_register_img.Func_Cfg_2.ClkOutUnl = 0;
    i_tx_buffer[1] = s_register_img.func_cfg_2_reg;     // Change just value

    e_status = cs2200_write_data(&i_tx_buffer[0], 2);
    if(e_status != GD_SUCCESS)
    {
      /* Because in this phase we do not know if bit was cleared or not,
       * let's set it in the image register so user should note it
       */
      s_register_img.Func_Cfg_2.ClkOutUnl = 1;
      return e_status;
    }

    // Jump to "return e_status;"
  }
  else
  {
    // Else user want increase ratio by save way
    return cs2200_set_ratio(s_register_img.Ratio.i_32bit -1);
  }
  return e_status;
}

/**
 * \brief Set output divider
 *
 * This function allow easily a quickly change output frequency. Sometimes\n
 * can be this useful when need just change frequency to 1/2x, 1/4x, 2x an so\n
 * on.
 * @param f_div_mul Divider/multiplier ratio.\n
 *  Valid values: 1 2 4 8 0.5 0.25 0.125 0.0625
 * @return GD_SUCCESS if all right
 */
GD_RES_CODE cs2200_set_out_divider_multiplier_float(float f_div_mul)
{
  if(f_div_mul == 1)
    return cs2200_set_out_divider_multiplier(CS2200_R_MOD__MUL_1x);
  else if(f_div_mul == 2)
    return cs2200_set_out_divider_multiplier(CS2200_R_MOD__MUL_2x);
  else if(f_div_mul == 4)
    return cs2200_set_out_divider_multiplier(CS2200_R_MOD__MUL_4x);
  else if(f_div_mul == 8)
    return cs2200_set_out_divider_multiplier(CS2200_R_MOD__MUL_8x);
  else if(f_div_mul == 0.5)
    return cs2200_set_out_divider_multiplier(CS2200_R_MOD__DIV_2x);
  else if(f_div_mul == 0.25)
    return cs2200_set_out_divider_multiplier(CS2200_R_MOD__DIV_4x);
  else if(f_div_mul == 0.125)
    return cs2200_set_out_divider_multiplier(CS2200_R_MOD__DIV_8x);
  else if(f_div_mul == 0.0625)
    return cs2200_set_out_divider_multiplier(CS2200_R_MOD__DIV_16x);
  // If value not found in list -> incorrect parameter
  else
    return GD_INCORRECT_PARAMETER;
}


/**
 * \brief Load output divider/multiplier value
 * @param p_div_mul Pinter to memory where value will be written
 * @return GD_SUCCESS if all right
 */
inline GD_RES_CODE cs2200_get_out_divider_multiplier_float(float *p_div_mul)
{
  *p_div_mul = s_virtual_reg_img.f_out_div_mul;
  return GD_SUCCESS;
}


/**
 * \brief Set output divider
 *
 * This function allow easily a quickly change output frequency. Sometimes\n
 * can be this useful when need just change frequency to 1/2x, 1/4x, 2x an so\n
 * on.
 *
 * @param e_out_div_mul Options: CS2200_R_MOD__MUL_1x, CS2200_R_MOD__MUL_2x,\n
 * CS2200_R_MOD__MUL_4x, CS2200_R_MOD__MUL_8x, CS2200_R_MOD__DIV_2x,\n
 * CS2200_R_MOD__DIV_4x, CS2200_R_MOD__DIV_8x, CS2200_R_MOD__DIV_16x
 *
 * @return  GD_SUCCESS if all right
 */
GD_RES_CODE cs2200_set_out_divider_multiplier(cs2200_r_mod_t e_out_div_mul)
{
  // Check input values
  if( (e_out_div_mul != CS2200_R_MOD__MUL_1x) &&
      (e_out_div_mul != CS2200_R_MOD__MUL_2x) &&
      (e_out_div_mul != CS2200_R_MOD__MUL_4x) &&
      (e_out_div_mul != CS2200_R_MOD__MUL_8x) &&
      (e_out_div_mul != CS2200_R_MOD__DIV_2x) &&
      (e_out_div_mul != CS2200_R_MOD__DIV_4x) &&
      (e_out_div_mul != CS2200_R_MOD__DIV_8x) &&
      (e_out_div_mul != CS2200_R_MOD__DIV_16x))
  {
    // If not at least one of them -> return error
    return GD_INCORRECT_PARAMETER;
  }
  // Create TX buffer
  uint8_t i_tx_buffer[2];

  // Create status variable
  GD_RES_CODE e_status;

  // Backup register value
  uint8_t i_backup = s_register_img.device_cfg_1_reg;

  // Fill TX buffer
  i_tx_buffer[0] = CS2200_REG_DEVICE_CFG_1;

  // Set new value
  s_register_img.Device_Cfg_1.RModSel = e_out_div_mul;

  i_tx_buffer[1] = s_register_img.device_cfg_1_reg;

  // Buffer ready, send data
  e_status = cs2200_write_data(&i_tx_buffer[0], 2);
  if(e_status != GD_SUCCESS)
  {
    // If problem occurs, then use backup value and return FAIL
    s_register_img.device_cfg_1_reg = i_backup;
    return e_status;
  }

  // Else all OK - Write correct divider/multiplier value to virtual register
  switch(e_out_div_mul)
  {
  case CS2200_R_MOD__MUL_1x:
    s_virtual_reg_img.f_out_div_mul = 1;
    break;
  case CS2200_R_MOD__MUL_2x:
    s_virtual_reg_img.f_out_div_mul = 2;
    break;
  case CS2200_R_MOD__MUL_4x:
    s_virtual_reg_img.f_out_div_mul = 4;
    break;
  case CS2200_R_MOD__MUL_8x:
    s_virtual_reg_img.f_out_div_mul = 8;
    break;
  case CS2200_R_MOD__DIV_2x:
    s_virtual_reg_img.f_out_div_mul = 0.5;
    break;
  case CS2200_R_MOD__DIV_4x:
    s_virtual_reg_img.f_out_div_mul = 0.25;
    break;
  case CS2200_R_MOD__DIV_8x:
    s_virtual_reg_img.f_out_div_mul = 0.125;
    break;
  case CS2200_R_MOD__DIV_16x:
    s_virtual_reg_img.f_out_div_mul = 0.0625;
    break;
  // Never should happen
  default:
    while(1);
  }
  return GD_SUCCESS;
}



/**
 * \brief Load value for output divider/multiplier
 * @param p_e_out_div_mul Pointer to memory where value will be written
 * @return GD_SUCCESS if all right
 */
inline GD_RES_CODE cs2200_get_out_divider_multiplier(
    cs2200_r_mod_t *p_e_out_div_mul)
{
  *p_e_out_div_mul = s_register_img.Device_Cfg_1.RModSel;
  return GD_SUCCESS;
}


/**
 * \brief Set flag that control PLL output when ratio is changed by 1
 *
 * When PLL changing output frequency there may be undefined output. So in\n
 * default PLL set output to LOW and when is locked then enable output clock.\n
 * However this may take some time. When changing ratio just by 1, there is\n
 * big chance that output "will not get crazy" and time when output is in low\n
 * is reduced from approximately 500 us to 150 ns which sometimes can solve\n
 * a lot.\n
 * Anyway generally is recommended set this flag to 1 (save change).
 *
 * @param i_save_change Options: 1 - save frequency change by 1 (recommended)\n
 *  0 - "dangerous" frequency change by 1, but with very short clock shortage
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE cs2200_set_save_change_ratio_by_1(uint8_t i_save_change)
{
  if(i_save_change == 0)
  {
    s_virtual_reg_img.i_save_change_ratio_by_1 = 0;
  }
  else
  {
    s_virtual_reg_img.i_save_change_ratio_by_1 = 1;
  }

  return GD_SUCCESS;
}


/**
 * \brief Get flag that control PLL output when ratio is changed by 1
 *
 * When PLL changing output frequency there may be undefined output. So in\n
 * default PLL set output to LOW and when is locked then enable output clock.\n
 * However this may take some time. When changing ratio just by 1, there is\n
 * big chance that output "will not get crazy" and time when output is in low\n
 * is reduced from approximately 500 us to 150 ns which sometimes can solve\n
 * a lot.\n
 * Anyway generally recommended value is 1.
 *
 * @param p_save_change Address where result will be saved
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE cs2200_get_save_change_ratio_by_1(uint8_t *p_save_change)
{
  *p_save_change = s_virtual_reg_img.i_save_change_ratio_by_1;
  return GD_SUCCESS;
}
//===========================| Mid level functions |===========================

/**
 * \brief Set ratio in PLL
 *
 * This function set ratio and if possible then calculate actual frequency.
 *
 * @param i_ratio Ratio value
 * @return GD_SUCCESS if all right
 */
GD_RES_CODE cs2200_set_ratio(uint32_t i_ratio)
{
  // Buffer for transmitting. Allow prepare bytes
  uint8_t i_tx_buffer[5];

  // Create status variable
  GD_RES_CODE e_status;

  // Temporary 32 bit value (allow split 32 bits into 8 bits)
  cs2200_32b_to_8b_t u_tmp32;
  u_tmp32.i_32bit = i_ratio;

  // Write register address (1B, enable auto increment) and value (4B)
  i_tx_buffer[0] = CS2200_REG_RATIO | (1<<CS2200_MAP_AUTO_INC_BIT);
  i_tx_buffer[1] = u_tmp32.i_8bit[0];
  i_tx_buffer[2] = u_tmp32.i_8bit[1];
  i_tx_buffer[3] = u_tmp32.i_8bit[2];
  i_tx_buffer[4] = u_tmp32.i_8bit[3];

  // Write data to PLL
  e_status = cs2200_write_data(&i_tx_buffer[0], 5);
  if(e_status != GD_SUCCESS)
  {
    // Operation failed
    return e_status;
  }

  // Ratio written, update in image
  s_register_img.Ratio.i_32bit = i_ratio;

  // Check if know clock. If yes, then calculate frequency
#if CS2200_REF_CLK_FREQ != 0
  // Else OK, so recalculate frequency (and update frequency value)
  return cs2200_calc_frequency(
      i_ratio,
      &s_virtual_reg_img.i_real_freq.i_32bit);
#else
  return GD_SUCCESS
#endif
}

/**
 * \brief Load ratio value
 * @param p_i_ratio Pointer define address where value will be written
 * @return GD_SUCCESS if all right
 */
inline GD_RES_CODE cs2200_get_ratio(uint32_t *p_i_ratio)
{
  *p_i_ratio = s_register_img.Ratio.i_32bit;

  return GD_SUCCESS;
}





/**
 * \brief Set or clear freeze bit
 * @param i_freeze_flag 0 clear freeze bit, otherwise set freeze bit
 * @return GD_SUCCESS if all right
 */
GD_RES_CODE cs2200_set_freeze_bit(uint8_t i_freeze_flag)
{
  // TX buffer - MAP value, register value
  uint8_t i_tx_buffer[2];

  // Backup register value
  uint8_t i_backup = s_register_img.global_cfg_reg;

  // Create status variable
  GD_RES_CODE e_status;

  i_tx_buffer[0] = CS2200_REG_GLOBAL_CFG;

  // Test if we want freeze enable or disable
  if(i_freeze_flag == 0)
  {     // disable freeze
    // Disable freeze value in image
    s_register_img.Global_Cfg.Freeze = 0;
  }
  else
  {     // Enable freeze
    // Enable freeze in image
    s_register_img.Global_Cfg.Freeze = 1;
  }

  // Load register value to buffer
  i_tx_buffer[1] = s_register_img.global_cfg_reg;

  // Send data (register value). Send register address and value
  e_status = cs2200_write_data(&i_tx_buffer[0], 2);
  if(e_status != GD_SUCCESS)
  {
    // If failed -> set value back
    s_register_img.global_cfg_reg = i_backup;

    return e_status;
  }

  // If everything is OK write flag
  s_virtual_reg_img.i_freeze_flag = i_freeze_flag;
  return GD_SUCCESS;
}



/**
 * \brief Load freeze bit
 * @param p_i_freeze_flag Pointer to memory where freeze bit will be written
 * @return GD_SUCCESS if all right
 */
inline GD_RES_CODE cs2200_get_freeze_bit(uint8_t *p_i_freeze_flag)
{
  *p_i_freeze_flag = s_register_img.Global_Cfg.Freeze;
  return GD_SUCCESS;
}



/**
 * \brief Enable configuration in PLL registers
 *
 * For normal operation (enable control port mode) is needed activate\n
 * configuration. This is done by this function.
 *
 * @return GD_SUCCESS if all right
 */
GD_RES_CODE cs2200_enable_device_cfg(void)
{
  // TX buffer - MAP value, register value
  uint8_t i_tx_buffer[2];

  // Create status variable
  GD_RES_CODE e_status;

  // Backup values
  uint8_t i_backup =
      (s_register_img.Device_Cfg_1.EnDevCfg1) | // Bit position: 0
      (s_register_img.Global_Cfg.EnDevCfg2<<1); // Bit position: 1

  // Set new values
  s_register_img.Device_Cfg_1.EnDevCfg1 = 1;
  s_register_img.Global_Cfg.EnDevCfg2 = 1;


  // First set register Device cfg 1
  i_tx_buffer[0] = CS2200_REG_DEVICE_CFG_1;
  i_tx_buffer[1] = s_register_img.device_cfg_1_reg;

  e_status = cs2200_write_data(&i_tx_buffer[0], 2);
  if(e_status != GD_SUCCESS)
  {
    // If something goes wrong... restore origin values and return FAIL code
    s_register_img.Device_Cfg_1.EnDevCfg1 = i_backup & 0x01;
    s_register_img.Global_Cfg.EnDevCfg2 = i_backup >>1;
    return e_status;
  }

  // Then set register Global cfg
  i_tx_buffer[0] = CS2200_REG_GLOBAL_CFG;
  i_tx_buffer[1] = s_register_img.global_cfg_reg;

  e_status = cs2200_write_data(&i_tx_buffer[0], 2);
  if(e_status != GD_SUCCESS)
  {
    // If something goes wrong... restore origin values and return FAIL
    s_register_img.Device_Cfg_1.EnDevCfg1 = i_backup & 0x01;
    s_register_img.Global_Cfg.EnDevCfg2 = i_backup >>1;
    return e_status;
  }

  // If there is no problem, return SUCCESS
  return GD_SUCCESS;
}



/**
 * \brief Disable configuration in PLL registers
 *
 * For normal operation (enable control port mode) is needed activate\n
 * configuration. This function turn off this
 *
 * @return GD_SUCCESS if all right
 */
GD_RES_CODE cs2200_disable_device_cfg(void)
{
  // TX buffer - MAP value, register value
  uint8_t i_tx_buffer[2];

  // Create status variable
  GD_RES_CODE e_status;

  // Backup values
  uint8_t i_backup =
      (s_register_img.Device_Cfg_1.EnDevCfg1) | // Bit position: 0
      (s_register_img.Global_Cfg.EnDevCfg2<<1); // Bit position: 1

  // Set new values
  s_register_img.Device_Cfg_1.EnDevCfg1 = 0;
  s_register_img.Global_Cfg.EnDevCfg2 = 0;


  // First set register Device cfg 1
  i_tx_buffer[0] = CS2200_REG_DEVICE_CFG_1;
  i_tx_buffer[1] = s_register_img.device_cfg_1_reg;

  e_status = cs2200_write_data(&i_tx_buffer[0], 2);
  if(e_status != GD_SUCCESS)
  {
    // If something goes wrong... restore origin values and return FAIL
    s_register_img.Device_Cfg_1.EnDevCfg1 = i_backup & 0x01;
    s_register_img.Global_Cfg.EnDevCfg2 = i_backup >>1;
    return e_status;
  }

  // Then set register Global cfg
  i_tx_buffer[0] = CS2200_REG_GLOBAL_CFG;
  i_tx_buffer[1] = s_register_img.global_cfg_reg;

  e_status = cs2200_write_data(&i_tx_buffer[0], 2);
  if(e_status != GD_SUCCESS)
  {
    // If something goes wrong... restore origin values and return FAIL
    s_register_img.Device_Cfg_1.EnDevCfg1 = i_backup & 0x01;
    s_register_img.Global_Cfg.EnDevCfg2 = i_backup >>1;
    return e_status;
  }

  // If there is no problem, return SUCCESS
  return GD_SUCCESS;
}



/**
 * \brief Allow easily set input divider
 *
 * This function is called in cs2200_init when CS2200_REF_CLK_FREQ is non\n
 * zero value
 * @param e_in_div Options: CS2200_INPUT_DIVIDER_1x, CS2200_INPUT_DIVIDER_2x,\n
 * CS2200_INPUT_DIVIDER_4x
 * @return GD_SUCCESS if all right
 */
GD_RES_CODE cs2200_set_in_divider(cs2200_in_div_t e_in_div)
{
  // Check input value if correct or not
  if((e_in_div != CS2200_INPUT_DIVIDER_1x) &&
     (e_in_div != CS2200_INPUT_DIVIDER_2x) &&
     (e_in_div != CS2200_INPUT_DIVIDER_4x))
  {
    /* If not equal to at least one of configuration, then input parameter
     * is incorrect -> return incorrect parameter
     */
    return GD_INCORRECT_PARAMETER;
  }

  // Create TX buffer
  uint8_t i_tx_buffer[2];

  // Create status variable
  GD_RES_CODE e_status;

  // Backup register value
  uint8_t i_backup = s_register_img.func_cfg_1_reg;

  // Fill TX buffer
  i_tx_buffer[0] = CS2200_REG_FUNC_CFG_1;

  // OK, input value is correct, so let's set new value
  s_register_img.Func_Cfg_1.RefClkDiv = e_in_div;

  i_tx_buffer[1] = s_register_img.func_cfg_1_reg;

  // Buffer ready, so send data
  e_status = cs2200_write_data(&i_tx_buffer[0], 2);
  if(e_status != GD_SUCCESS)
  {
    // If problem occurs, then use backup value and return FAIL
    s_register_img.func_cfg_1_reg = i_backup;
    return e_status;
  }

  // Else all OK
  return GD_SUCCESS;
}



/**
 * \brief Load in divider value
 * @param p_e_in_div Pointer to memory where divider value will be written
 * @return GD_SUCCESS if all right
 */
inline GD_RES_CODE cs2200_get_in_divider(cs2200_in_div_t *p_e_in_div)
{
  *p_e_in_div = s_register_img.Func_Cfg_1.RefClkDiv;
  return GD_SUCCESS;
}
//===========================| Low level functions |===========================
/**
 * \brief Calculate actual frequency from ratio
 * @param i_ratio Ratio value
 * @param p_i_frequency Pointer to memory where frequency will be written
 * @return GD_SUCCESS if clock source is known
 */
inline GD_RES_CODE cs2200_calc_frequency(
    uint32_t i_ratio,
    uint32_t *p_i_frequency)
{
#if CS2200_REF_CLK_FREQ == 0
  // If 0 -> do not know clock source -> return fail
  return GD_FAIL;
#else
  // We know actual clock source frequency -> calculate
  /* Need to multiply two 32 bit values -> 64 bit temporary variable to aviod
   * overflow
   */
  uint64_t i_tmp64;
  i_tmp64 = ((uint64_t)i_ratio*CS2200_REF_CLK_FREQ)/ 1048576UL;

  // Copy to 32 bit value
  *p_i_frequency = (uint32_t)i_tmp64;

  return GD_SUCCESS;
#endif
}

/**
 * \brief Calculate actual ratio from frequency
 * @param i_frequency Frequency
 * @param p_i_ratio Pointer to memory where ratio will be written
 * @return GD_SUCCESS if clock source is known
 */
inline GD_RES_CODE cs2200_calc_ratio(uint32_t i_frequency, uint32_t *p_i_ratio)
{
#if CS2200_REF_CLK_FREQ == 0
  // If 0 -> do not know clock source -> return fail
  return GD_FAIL;
#else
  // We know actual clock source frequency -> calculate
  /* Need to calculate fractional value and other stuff.
   * Calculated frequency save back to i_freq
   * Because we multiply 2^20 and input frequency number, we need 64bit value
   * to avoid overflow.
   */
  uint64_t i_tmp64;
  i_tmp64 = (((uint64_t)i_frequency) *1048576UL) /CS2200_REF_CLK_FREQ;

  // Copy to 32 bit value
  *p_i_ratio = (uint32_t)i_tmp64;

  return GD_SUCCESS;
#endif
}



/**
 * \brief Write data on TWI (I2C) bus
 *
 * MCU is in master mode. Send CS2200 address and then data (argument). This\n
 * function also check if TWI module is available (when RTOS support is\n
 * enabled).
 *
 * @param p_data Pointer to data array which will be send to PLL thru TWI (I2C)
 *
 * @param i_number_of_bytes Number of data bytes, which will be send
 *
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE cs2200_write_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes)
{
  // If RTOS support enable, then "lock" TWI module
  CS2200_LOCK_TWI_MODULE_IF_RTOS

  // Write data to PLL thru HAL. Also check result status
  if(cs2200_HAL_write_data(p_data, i_number_of_bytes) != CS2200_OK)
  {
    // If not OK -> unlock device (if needed) and return FAIL
    CS2200_UNLOCK_TWI_MODULE_IF_RTOS
    return GD_FAIL;
  }

  // "Unlock" device if needed
  CS2200_UNLOCK_TWI_MODULE_IF_RTOS
  return GD_SUCCESS;
}

/**
 * \brief Read data on TWI (I2C) bus
 *
 * MCU is in master mode. Send CS2200 address and then receive data. This\n
 * function also check if TWI module is available (when RTOS support is\n
 * enabled).
 *
 * @param p_data Data are saved thru this pointer
 *
 * @param i_number_of_bytes Number of data bytes, which will be received
 *
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE cs2200_read_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes)
{
  // If RTOS support enable, then "lock" TWI module
  CS2200_LOCK_TWI_MODULE_IF_RTOS

  // Read data from PLL thru HAL. Also check result status
  if(cs2200_HAL_read_data(p_data, i_number_of_bytes) != CS2200_OK)
  {
    // If not OK -> unlock device (if needed) and return FAIL
    CS2200_UNLOCK_TWI_MODULE_IF_RTOS
    return GD_FAIL;
  }

  // "Unlock" device if needed
  CS2200_UNLOCK_TWI_MODULE_IF_RTOS
  return GD_SUCCESS;
}

