/*
Sonochan Mk.II Firmware.
Copyright (C) 2013-2015 Martin Stejskal,
ALPS Electric Czech, s.r.o.,
ALPS Electric Co., Ltd.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

Contact information:
Martin Stejskal <martin.stejskal@jp.alps.com>,
Josef Nevrly <jnevrly@alps.cz>
ALPS Electric Czech, s.r.o., Sebranice 240, 679 31 Sebranice u Boskovic
Czech Republic
*/
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


#ifndef _DISPLAY_H_
#define _DISPLAY_H_
//=================================| Options |=================================
//================================| Includes |=================================
#include <inttypes.h>

// We need to call functions from board driver
#include "brd_driver_hw_03.h"

// Lightweight sprintf() functions
#include "tinyprintf.h"
#include "tinyprintf_extra.h"

// Printing debug messages through UART
#include "print_funcs.h"

// Driver for display
#include "LCD_5110.h"

//=================================| Defines |=================================
/**
 * \name Display configuration
 *
 * @{
 */
/**
 * \brief Define on which line will be written device number
 */
#define DISP_LCD_LINE_DEV_NUM           0
/**
 * @brief Define X coordinates for device number
 */
#define DISP_LCD_X_DEV_NUM              0



/**
 * @brief Define on which line will be written firmware version
 */
#define DISP_LCD_LINE_FW_VERSION        0
/**
 * @brief Define X coordinates for device number
 */
#define DISP_LCD_X_FW_VERSION           4



/**
 * @brief Define on which line will be written Digital Audio Interface mode
 */
#define DISP_LCD_LINE_DAI_MODE          1
/**
 * @brief Define X coordinates for Digital Audio Interface mode
 */
#define DISP_LCD_X_DAI_MODE             0



/**
 * @brief Line on which will be written information about auto tune option
 */
#define DISP_LCD_LINE_AUTO_TUNE         1
/**
 * @brief X coordinates for auto tune option
 */
#define DISP_LCD_X_AUTO_TUNE            4


/**
 * \brief Define on which line will be FSYNC frequency written
 */
#define DISP_LCD_LINE_FSYNC_FREQ                2
/**
 * @brief X coordinated for display FSYNC frequency
 */
#define DISP_LCD_X_FSYNC_FREQ                   0


/**
 * \brief Define on which line will be written BCLK oversampling value
 */
#define DISP_LCD_LINE_BCLK_OVRSAMPLING          3
/**
 * @brief X coordinates for BCLK oversampling information
 */
#define DISP_LCD_X_BCLK_OVRSAMPLING             0


/**
 * \brief Define on which line will be written MCLK frequency
 */
#define DISP_LCD_LINE_MCLK_FREQ                 4
/**
 * @brief X coordinates for MCLK frequency
 */
#define DISP_LCD_X_MCLK_FREQ                    0




/**
 * \brief Define on which line will be displayed headphone volume
 */
#define DISP_LCD_LINE_HP_VOLUME                 5
/**
 * @brief X coordinates for Volume information
 */
#define DISP_LCD_X_HP_VOLUME                    0


/**
 * \brief Define on which line begin write info message
 */
#define DISP_LCD_LINE_INFO_MSG                  0
/**
 * @brief X coordinates for information message
 */
#define DISP_LCD_X_INFO_MSG                     4


/**
 * @brief Define on which line begin write warning message
 */
#define DISP_LCD_LINE_WRN_MSG                   0
/**
 * @brief X coordinates for warning message
 */
#define DISP_LCD_X_WRN_MSG                      4

/**
 * @brief Define on which line begin write error message
 */
#define DISP_LCD_LINE_ERR_MSG                   0
/**
 * @brief X coordinates for error message
 */
#define DISP_LCD_X_ERR_MSG                      4


///@}


/**
 * \name Error messages for debug output
 *
 * @{
 */
#define DISP_ERR_MSG_LDING_PARAM_FAIL           \
  "DISPLAY: Loading parameters from board driver failed!\n"

#define DISP_ERR_MSG_CNT_SHOW_DAI               \
  "DISPLAY: Can not display digital audio interface mode!\n"

#define DISP_ERR_MSG_CNT_SHOW_AUTO_TUNE         \
  "DISPLAY: Can not display auto tune option!\n"

#define DISP_ERR_MSG_CNT_SHOW_FSYNC_FREQ        \
  "DISPLAY: Can not display FSYNC frequency!\n"

#define DISP_ERR_MSG_CNT_SHOW_BCLK_OVRSMPLNG    \
  "DISPLAY: Can not display BCLK oversampling!\n"

#define DISP_ERR_MSG_CNT_SHOW_MCLK_FREQ         \
  "DISPLAY: Can not display MCLK frequency!\n"

#define DISP_ERR_MSG_CNT_SHOW_VOLUME            \
  "DISPLAY: Can not display volume!\n"
///@}


//===============================| Structures |================================
/**
 * @brief Structure for keeping parameters
 *
 * When some parameter is changed, display should be updated. So for this\n
 * purpose we have this nice structure that keeps all used parameters at one\n
 * place.
 */
typedef struct{
  // Keep information about selected mode (I2S/DSP/...)
  e_ssc_digital_audio_interface_t e_dai;

  // Auto tune enabled?
  uint8_t i_auto_tune;

  // Keep information about sampling frequency
  uint32_t i_FSYNC_freq;

  // Store information about BLCK oversampling value
  uint16_t i_BCLK_ovrsam;

  // Keep information about MCLK frequency
  uint32_t i_MCLK_freq;

  // Keep information about volume value
  float f_volume;
}disp_params_t;

//==========================| High level functions |===========================
GD_RES_CODE disp_init(void);

void disp_task(void);

//===========================| Mid level functions |===========================
GD_RES_CODE disp_show_info(const char *p_c_msg);

GD_RES_CODE disp_clear_info(void);

GD_RES_CODE disp_show_warn(const char *p_c_array);

GD_RES_CODE disp_clear_warn(void);

GD_RES_CODE disp_show_err(const char *p_c_array);

GD_RES_CODE disp_clear_err(void);

GD_RES_CODE disp_refresh_display(void);
//===========================| Low level functions |===========================
GD_RES_CODE disp_show_dev_num(void);

GD_RES_CODE disp_show_FW_version(void);

GD_RES_CODE disp_show_dai_mode(void);

GD_RES_CODE disp_show_auto_tune(void);

GD_RES_CODE disp_show_FSYNC(void);

GD_RES_CODE disp_show_BCLK(void);

GD_RES_CODE disp_show_MCLK(void);

GD_RES_CODE disp_show_volume(void);



#endif
