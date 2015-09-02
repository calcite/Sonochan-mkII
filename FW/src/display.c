/**
 * @file
 *
 * @brief Library for generic display
 *
 * Created:  2015/08/27\n
 * Modified: 2015/09/01
 *
 * \version 0.1
 * \author  Martin Stejskal
 */

#include "display.h"



//============================| Static variables |=============================

/**
 * @brief Non zero if info is shown on display
 *
 * Zero if info should be cleared and show parameters again
 */
static uint8_t i_show_info;

/**
 * @brief Non zero if warning is shown on display.
 *
 * Zero if warning should be cleared and show parameters again
 */
static uint8_t i_show_warn;

/**
 * @brief Non zero if error is shown on display.
 *
 * Zero if error should be cleared and show parameters again
 */
static uint8_t i_show_err;

//==========================| High level functions |===========================
/**
 * @brief Initialize display driver and show basic informations
 *
 * Necessary to run as first. Else unexpected behavior can occur.
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_init(void)
{
  GD_RES_CODE e_status;

  // Initialize low level driver
  e_status = LCD_5110_init();
  if(e_status != 0) return GD_FAIL;

  // Configure LCD
  // We do not want "book style" LCD
  LCD_5110_auto_clear_display(0);
  // We want auto clean line
  LCD_5110_auto_clear_line(1);
  /* Do not need automatic newline, because all messages have '\n' because of
   * debug output
   */
  LCD_5110_auto_newline(0);

  // Show all data. Later will be overwritten anyway
  e_status = disp_refresh_display();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  return e_status;
}


/**
 * @brief Task for display
 *
 * This function automatically check all watched parameters (by using "get"\n
 * functions) and if needed update them. So display will not "blink" and we\n
 * can save a lot of CPU time.\n
 * Also if there is request to show only message, no parameter is updated\n
 * until this request is cleared.
 */
void disp_task(void)
{
  GD_RES_CODE e_status;

  // Static variable - keep informations about previous values
  static disp_params_t s_params_bkp;
  disp_params_t s_params_actual;

  // Do not update display if information/warning/error is shown
  if((i_show_info != 0) ||
     (i_show_warn != 0) ||
     (i_show_err  != 0))
  {
    // Just keep it on LCD. Do not change anything
    return;
  }


  /* Load actual values. Do not need to check error code, it will be checked
   * also before writing to display
   */
  e_status = brd_drv_get_digital_audio_interface_mode(&s_params_actual.e_dai);
  e_status = brd_drv_get_auto_tune(&s_params_actual.i_auto_tune);
  e_status = brd_drv_get_FSYNC_freq(&s_params_actual.i_FSYNC_freq);
  e_status = brd_drv_get_BCLK_oversampling(&s_params_actual.i_BCLK_ovrsam);
  e_status = brd_drv_get_MCLK_freq(&s_params_actual.i_MCLK_freq);
  e_status = brd_drv_get_volume_dB(&s_params_actual.f_volume);

  // Check if there is problem
  if(e_status != GD_SUCCESS)
  {
    print_dbg(DISP_ERR_MSG_LDING_PARAM_FAIL);
    // It does not have meaning to continue here
    return;
  }

  /* Compare actual values and backup values. If some item is different,
   * update actual parameter
   */
  // DIGITAL AUDIO INTERFACE
  if(s_params_actual.e_dai != s_params_bkp.e_dai)
  {
    s_params_bkp.e_dai = s_params_actual.e_dai;

    e_status = disp_show_dai_mode();
    if(e_status != GD_SUCCESS)
    {
      print_dbg(DISP_ERR_MSG_CNT_SHOW_DAI);
    }
  }

  // AUTO TUNE
  if(s_params_actual.i_auto_tune != s_params_bkp.i_auto_tune)
  {
    s_params_bkp.i_auto_tune = s_params_actual.i_auto_tune;

    e_status = disp_show_auto_tune();
    if(e_status != GD_SUCCESS)
    {
      print_dbg(DISP_ERR_MSG_CNT_SHOW_AUTO_TUNE);
    }
  }

  // FSYNC FREQ
  if(s_params_actual.i_FSYNC_freq != s_params_bkp.i_FSYNC_freq)
  {
    s_params_bkp.i_FSYNC_freq = s_params_actual.i_FSYNC_freq;

    e_status = disp_show_FSYNC();
    if(e_status != GD_SUCCESS)
    {
      print_dbg(DISP_ERR_MSG_CNT_SHOW_FSYNC_FREQ);
    }
  }

  // BCLK OVERSAMPLING
  if(s_params_actual.i_BCLK_ovrsam != s_params_bkp.i_BCLK_ovrsam)
  {
    s_params_bkp.i_BCLK_ovrsam = s_params_actual.i_BCLK_ovrsam;

    e_status = disp_show_BCLK();
    if(e_status != GD_SUCCESS)
    {
      print_dbg(DISP_ERR_MSG_CNT_SHOW_BCLK_OVRSMPLNG);
    }
  }

  // MCLK FREQ
  if(s_params_actual.i_MCLK_freq != s_params_bkp.i_MCLK_freq)
  {
    s_params_bkp.i_MCLK_freq = s_params_actual.i_MCLK_freq;

    e_status = disp_show_MCLK();
    if(e_status != GD_SUCCESS)
    {
      print_dbg(DISP_ERR_MSG_CNT_SHOW_MCLK_FREQ);
    }
  }

  // VOLUME
  if(s_params_actual.f_volume != s_params_bkp.f_volume)
  {
    s_params_bkp.f_volume = s_params_actual.f_volume;

    e_status = disp_show_volume();
    if(e_status != GD_SUCCESS)
    {
      print_dbg(DISP_ERR_MSG_CNT_SHOW_VOLUME);
    }
  }
}
//===========================| Mid level functions |===========================
/**
 * @brief Function that show message on display
 * @param p_c_msg Message itself
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_show_info(const char *p_c_msg)
{
  GD_RES_CODE e_status;

  // So we want to show info. So all warning and error messages disappear
  i_show_info = 1;
  i_show_warn = 0;
  i_show_err  = 0;

  e_status = LCD_5110_clear();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // Write message
  e_status = LCD_5110_write_xy(p_c_msg,
                               DISP_LCD_X_INFO_MSG*6,
                               DISP_LCD_LINE_INFO_MSG);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // And we always want to show device number
  e_status = disp_show_dev_num();
  if(e_status != GD_SUCCESS) return e_status;

  return e_status;
}

/**
 * @brief Clear info message from display and show parameters
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE disp_clear_info(void)
{
  GD_RES_CODE e_status;

  i_show_info = 0;
  e_status = disp_refresh_display();

  return e_status;
}

/**
 * @brief Show warning message on display
 * @param p_c_msg Pointer to string array
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_show_warn(const char *p_c_msg)
{
  GD_RES_CODE e_status;

  // Want to show warning message.
  // All info and error messages are no longer displayed
  i_show_warn = 1;
  i_show_info = 0;
  i_show_err  = 0;

  e_status = LCD_5110_clear();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // Write message
  e_status = LCD_5110_write_xy(p_c_msg,
                               DISP_LCD_X_WRN_MSG*6,
                               DISP_LCD_LINE_WRN_MSG);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // And we always want to show device number
  e_status = disp_show_dev_num();
  if(e_status != GD_SUCCESS) return e_status;

  return e_status;
}

/**
 * @brief Clear warning message from display and show parameters
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE disp_clear_warn(void)
{
  GD_RES_CODE e_status;

  i_show_warn = 0;
  e_status = disp_refresh_display();

  return e_status;
}


/**
 * @brief Show error message on display
 * @param p_c_msg Pointer to string array
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_show_err(const char *p_c_msg)
{
  GD_RES_CODE e_status;

  // Error message show, delete info or warning message
  i_show_err  = 1;
  i_show_info = 0;
  i_show_warn = 0;

  e_status = LCD_5110_clear();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // Write message
  e_status = LCD_5110_write_xy(p_c_msg,
                               DISP_LCD_X_ERR_MSG*6,
                               DISP_LCD_LINE_ERR_MSG);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // And we always want to show device number
  e_status = disp_show_dev_num();
  if(e_status != GD_SUCCESS) return e_status;

  return e_status;
}

/**
 * @brief Clear error message and show all contents again
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE disp_clear_err(void)
{
  GD_RES_CODE e_status;

  i_show_err = 0;
  e_status = disp_refresh_display();

  return e_status;
}


/**
 * @brief Show all basic informations to display no matter what
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_refresh_display(void)
{
  GD_RES_CODE e_status;

  e_status = LCD_5110_clear();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  e_status = disp_show_dev_num();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  e_status = disp_show_FW_version();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  e_status = disp_show_dai_mode();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  e_status = disp_show_auto_tune();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  e_status = disp_show_FSYNC();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  e_status = disp_show_BCLK();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  e_status = disp_show_MCLK();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  e_status = disp_show_volume();
  if(e_status != GD_SUCCESS) return GD_FAIL;

  return e_status;
}
//===========================| Low level functions |===========================
/**
 * @brief Write device number to display
 *
 * Thanks to this number every single device can be simply recognized at USB.\n
 * Also it helps in user application.
 *
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_show_dev_num(void)
{
  // Keep string in RAM
  char c[5];

  // Status
  GD_RES_CODE e_status;

  // Product number (at USB stack)
  uint8_t i_product_number;


  e_status = brd_drv_get_number_from_product_name(&i_product_number);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  tfp_sprintf(&c[0], "#%u",i_product_number);

  // Clean own part on LCD
  e_status = LCD_5110_write_xy("   ",
                               DISP_LCD_X_DEV_NUM*6,
                               DISP_LCD_LINE_DEV_NUM);
  if(e_status != 0) return GD_FAIL;

  // Write actually to LCD
  e_status = LCD_5110_write_xy(&c[0],
                               DISP_LCD_X_DEV_NUM*6,
                               DISP_LCD_LINE_DEV_NUM);
  if(e_status != 0) return GD_FAIL;

  return e_status;
}

/**
 * @brief Write to display information about FW version
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_show_FW_version(void)
{
  /* Following preprocessor constants are given to preprocessor during compile
   * time. These values are automatically increment. "Next values" can be
   * found at "Release" folder as TXT files. We do not recommend to change
   * this constants manually.
   */
  static const unsigned long bld_num_A = __BUILD_NUMBER_A;
  static const unsigned long bld_num_B = __BUILD_NUMBER_B;
  static const unsigned long bld_num_C = __BUILD_NUMBER_C;

  // Status
  GD_RES_CODE e_status;

  char c[20];

  // *6 -> font size is 6px -> count move in
  // Clean own part on LCD
  e_status = LCD_5110_write_xy("          ",
                               DISP_LCD_X_FW_VERSION*6,
                               DISP_LCD_LINE_FW_VERSION);
  if(e_status != 0) return GD_FAIL;

  tfp_sprintf(&c[0],"FW %lu.%lu.%lu", bld_num_A, bld_num_B, bld_num_C);

  e_status = LCD_5110_write_xy(&c[0],
                               DISP_LCD_X_FW_VERSION*6,
                               DISP_LCD_LINE_FW_VERSION);
  if(e_status != 0) return GD_FAIL;

  return e_status;
}


/**
 * @brief Show digital audio interface mode
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_show_dai_mode(void)
{
  GD_RES_CODE e_status;

  // I2S/DSP/LJS/RJS
  e_ssc_digital_audio_interface_t e_dai;

  // Keep string in memory
  char c[20];

  e_status = brd_drv_get_digital_audio_interface_mode(&e_dai);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  switch(e_dai)
  {
  case SSC_I2S:
    strcpy(c,"I2S");
    break;
  case SSC_DSP:
    strcpy(c,"DSP");
    break;
  case SSC_LEFT_JUSTIFIED:
    strcpy(c,"LJF");
    break;
  case SSC_RIGHT_JUSTIFIED:
    strcpy(c,"RJF");
    break;
  default:
    // In case there is not handled option -> error
    return GD_FAIL;
  }

  // Clean old value
  e_status = LCD_5110_write_xy("   ",
                               DISP_LCD_X_DAI_MODE*6,
                               DISP_LCD_LINE_DAI_MODE);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // Write new one
  e_status = LCD_5110_write_xy(&c[0],
                               DISP_LCD_X_DAI_MODE*6,
                               DISP_LCD_LINE_DAI_MODE);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  return e_status;
}


/**
 * @brief Show auto tune option
 *
 * Auto - auto tune on ; Fix - MCLK is fixed
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_show_auto_tune(void)
{
  GD_RES_CODE e_status;

  // Keep string in RAM
  char c[20] = "PLL: ";

  // Auto tune enabled?
  uint8_t i_auto_tune_en;

  e_status = brd_drv_get_auto_tune(&i_auto_tune_en);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  if(i_auto_tune_en == 0)
  {
    strcat(c, "Fix");
  }
  else
  {
    strcat(c, "Auto");
  }

  // Clean old value
  e_status = LCD_5110_write_xy("         ",
                               DISP_LCD_X_AUTO_TUNE*6,
                               DISP_LCD_LINE_AUTO_TUNE);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // Write new value
  e_status = LCD_5110_write_xy(&c[0],
                               DISP_LCD_X_AUTO_TUNE*6,
                               DISP_LCD_LINE_AUTO_TUNE);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  return GD_SUCCESS;
}


/**
 * @brief Display FSYNC frequency in Hz
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_show_FSYNC(void)
{
  // Status
  GD_RES_CODE e_status;

  // FSYNC frequency value
  uint32_t i_FSYNC_freq;

  // String array
  char c[20];

  e_status = brd_drv_get_FSYNC_freq(&i_FSYNC_freq);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  tfp_sprintf(&c[0], "FS %lu Hz", i_FSYNC_freq);

  // Clean old value
  e_status = LCD_5110_write_xy("            ",
                               DISP_LCD_X_FSYNC_FREQ*6,
                               DISP_LCD_LINE_FSYNC_FREQ);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // Write new one
  e_status = LCD_5110_write_xy(&c[0],
                               DISP_LCD_X_FSYNC_FREQ*6,
                               DISP_LCD_LINE_FSYNC_FREQ);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  return e_status;
}


/**
 * @brief Show BCLK oversampling value
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_show_BCLK(void)
{
  // Status
  GD_RES_CODE e_status;

  // BCLK oversampling value
  uint16_t i_BCLK_ovrsam;

  // String array
  char c[20];

  e_status = brd_drv_get_BCLK_oversampling(&i_BCLK_ovrsam);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  tfp_sprintf(&c[0],"BCLK %uxFS", i_BCLK_ovrsam);

  // Clean old value
  e_status = LCD_5110_write_xy("            ",
                               DISP_LCD_X_BCLK_OVRSAMPLING*6,
                               DISP_LCD_LINE_BCLK_OVRSAMPLING);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // Write new one
  e_status = LCD_5110_write_xy(&c[0],
                               DISP_LCD_X_BCLK_OVRSAMPLING*6,
                               DISP_LCD_LINE_BCLK_OVRSAMPLING);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  return e_status;
}


/**
 * @brief Display MCLK frequency in Hz
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_show_MCLK(void)
{
  GD_RES_CODE e_status;

  // MCLK frequency
  uint32_t i_MCLK_freq;

  // String array
  char c[20];

  e_status = brd_drv_get_MCLK_freq(&i_MCLK_freq);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  tfp_sprintf(&c[0], "MCLK %lu", i_MCLK_freq);

  // Clear old value
  e_status = LCD_5110_write_xy("              ",
                               DISP_LCD_X_MCLK_FREQ*6,
                               DISP_LCD_LINE_MCLK_FREQ);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // Write new one
  e_status = LCD_5110_write_xy(&c[0],
                               DISP_LCD_X_MCLK_FREQ*6,
                               DISP_LCD_LINE_MCLK_FREQ);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  return e_status;
}


/**
 * @brief Display volume at codec
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE disp_show_volume(void)
{
  GD_RES_CODE e_status;

  // Volume value
  float f_volume;

  // String array
  char c[20] = "Vol: -XX.X dB";

  e_status = brd_drv_get_volume_dB(&f_volume);
  if(e_status != GD_SUCCESS) return GD_SUCCESS;

  floatToStr(f_volume, 1, &c[5]);

  // Add dB
  strcat(c, " dB");

  // Clear old value
  e_status = LCD_5110_write_xy("             ",
                               DISP_LCD_X_HP_VOLUME*6,
                               DISP_LCD_LINE_HP_VOLUME);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  // Write new one
  e_status = LCD_5110_write_xy(&c[0],
                               DISP_LCD_X_HP_VOLUME*6,
                               DISP_LCD_LINE_HP_VOLUME);
  if(e_status != GD_SUCCESS) return GD_FAIL;

  return e_status;
}

