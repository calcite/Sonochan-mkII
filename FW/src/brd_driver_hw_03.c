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
 * \file
 *
 * \brief Board driver for "Sonochan mkII" HW version 03
 *
 * Allow set basic settings on board. Also support "generic driver".
 * Written only for AVR32 UC3A3.
 *
 * Created:  2014/04/23\n
 * Modified: 2017/09/15
 *
 * \version 0.7
 * \author  Martin Stejskal
 */

#include "brd_driver_hw_03.h"
//============================| Global variables |=============================
/**
 * \brief Mute structure
 *
 * Contains mute direction and mute value
 */
static s_brd_drv_mute_dai_t s_brd_drv_mute;

/**
 * \brief Reset DAI structure
 *
 * Contains reset DAI direction and value
 */
static s_brd_drv_rst_dai_t s_brd_drv_rst_i2s;

/**
 * \brief Store directions for MCLK, BCLK, FSYNC, TX_DATA, RX_DATA
 */
static s_brd_drv_pure_dai_dir_t s_brd_drv_pure_dai_dir;

static s_brd_drv_FSYNC_freq_req_t s_brd_drv_FSYNC_freq_req;

/**
 * \brief Store fine settings of SSC module
 */
static s_brd_drv_ssc_fine_setting_t s_brd_drv_ssc_fine_settings;

/**
 * \brief Store auto tune PLL value
 */
static uint8_t i_auto_tune_pll;

/**
 * \brief Copied value from USB descriptors. Number added behind device name
 *
 * This value should be copied, else optimizer set it as constant and this is\n
 * something that we do not want.
 */
static uint8_t i_product_name_number;

//===========================| "EEPROM" variables |============================
// Set up NVRAM (EEPROM) storage
#if defined (__GNUC__)
__attribute__((__section__(".userpage")))
#endif
/**
 * \brief Structure for saving and restoring settings
 */
static s_brd_drv_user_settings_t s_brd_drv_user_settings;

/**
 * \brief Variable that allow detect correct or incorrect settings in user\n
 * flash
 */
#if defined (__GNUC__)
__attribute__((__section__(".userpage")))
#endif
static uint8_t i_brd_drv_settings_check;

//=========================| Generic driver support |==========================
#if BRD_DRV_SUPPORT_GENERIC_DRIVER == 1
/**
 * \brief Configure table for device
 */

// Do not test architecture. This is mainly for AVR32.
const gd_config_struct BRD_DRV_config_table[] =
  {
    {
#define BRD_DRV_CMD_INIT        0
      BRD_DRV_CMD_INIT,                         // Command ID
      "Initialize board hardware",              // Name
      "Initialize I/O and prepare codec",       // Descriptor
      void_type,              // Input data type
      {.data_uint32 = 0},     // Minimum input value
      {.data_uint32 = 0},     // Maximum input value
      void_type,              // Output data type
      {.data_uint32 = 0},     // Minimum output value
      {.data_uint32 = 0},     // Maximum output value
      (GD_DATA_VALUE*)&gd_void_value, // Output value
      brd_drv_init                    /* Function, that should be
                                       * called
                                       */
    },
    {
#define BRD_DRV_CMD_RESET_CON   BRD_DRV_CMD_INIT+1
      BRD_DRV_CMD_RESET_CON,
      "Reset connector I/O's",
      "Set all signals to Hi-Z. Codec on board will work.",
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      (GD_DATA_VALUE*)&gd_void_value,
      brd_drv_reset_i2s
    },
    {
#define BRD_DRV_CMD_MCLK_DIR    BRD_DRV_CMD_RESET_CON+1
      BRD_DRV_CMD_MCLK_DIR,
      "MCLK direction",
      "0 - input ; 1 - output ; 2 - Hi-Z",
      uint32_type,      // Cause it is enum and 32 bit system
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_pure_dai_dir.e_mclk_dir,
      brd_drv_set_mclk_dir
    },
    {
#define BRD_DRV_CMD_BCLK_DIR    BRD_DRV_CMD_MCLK_DIR+1
      BRD_DRV_CMD_BCLK_DIR,
      "BCLK role and direction",
      "0 - slave, BCLK is input ; 1 - master, BCLK output ; 2 - Hi-z",
      uint32_type,      // Cause it is enum and 32 bit system
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_pure_dai_dir.e_bclk_dir,
      brd_drv_set_bclk_dir
    },
    {
#define BRD_DRV_CMD_FSYNC_DIR   BRD_DRV_CMD_BCLK_DIR+1
      BRD_DRV_CMD_FSYNC_DIR,
      "FSYNC role and direction",
      "0-slave, FSYNC (word clock) is input; 1-master, FSYNC is output; 2-hi-z",
      uint32_type,      // Cause it is enum and 32 bit system
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_pure_dai_dir.e_fsync_dir,
      brd_drv_set_fsync_dir
    },
    {
#define BRD_DRV_CMD_MUTE_DIR    BRD_DRV_CMD_FSYNC_DIR+1
      BRD_DRV_CMD_MUTE_DIR,
      "MUTE direction",
      "0 - input ; 1 - output ; 2 - Hi-Z",
      uint32_type,      // Cause it is enum and 32 bit system
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_mute.e_mute_dir,
      brd_drv_set_mute_dir
    },
    {
#define BRD_DRV_CMD_MUTE_FLAG   BRD_DRV_CMD_MUTE_DIR+1
      BRD_DRV_CMD_MUTE_FLAG,
      "Set mute flag",
      "0 - mute off ; 1 - mute on",
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 1},
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 1},
      (GD_DATA_VALUE*)&s_brd_drv_mute.i_mute_val,
      brd_drv_set_mute
    },
    {
#define BRD_DRV_CMD_RESET_DIR   BRD_DRV_CMD_MUTE_FLAG+1
      BRD_DRV_CMD_RESET_DIR,
      "Set RESET_I2S direction",
      "0 - input ; 1 - output ; 2 - Hi-Z",
      uint32_type,      // Cause it is enum and 32 bit system
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_rst_i2s.e_rst_dai_dir,
      brd_drv_set_rst_dai_dir
    },
    {
#define BRD_DRV_CMD_RESET_FLAG  BRD_DRV_CMD_RESET_DIR+1
      BRD_DRV_CMD_RESET_FLAG,
      "Set reset I2S flag",
      "0 - off ; 1 - on",
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 1},
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 1},
      (GD_DATA_VALUE*)&s_brd_drv_rst_i2s.i_rst_dai_val,
      brd_drv_set_rst_dai
    },
    {
#define BRD_DRV_CMD_TX_DATA_DIR BRD_DRV_CMD_RESET_FLAG+1
      BRD_DRV_CMD_TX_DATA_DIR,
      "Set TX_DATA direction",
      "1 - output ; 2 - HiZ",
      uint32_type,      // Cause it is enum and 32 bit system
      {.data_uint32 = 1},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 1},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_pure_dai_dir.e_tx_data_dir,
      brd_drv_set_tx_data_dir
    },
    {
#define BRD_DRV_CMD_RX_DATA_DIR BRD_DRV_CMD_TX_DATA_DIR+1
      BRD_DRV_CMD_RX_DATA_DIR,
      "Set RX_DATA direction",
      "0 - input ; 1 - not allowed ; 2 - HiZ",
      uint32_type,      // Cause it is enum and 32 bit system
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_pure_dai_dir.e_rx_data_dir,
      brd_drv_set_rx_data_dir
    },
    {
#define BRD_DRV_CMD_WORD_SIZE           BRD_DRV_CMD_RX_DATA_DIR+1
      BRD_DRV_CMD_WORD_SIZE,
      "Set word size",
      "Data size in bits. Usually 16, 20, 24 and 32",
      uint8_type,
      {.data_uint8 = 1},
      {.data_uint8 = 32},
      uint8_type,
      {.data_uint8 = 1},
      {.data_uint8 = 32},
      (GD_DATA_VALUE*)&s_brd_drv_ssc_fine_settings.i_data_length,
      brd_drv_set_data_length
    },
    {
#define BRD_DRV_CMD_MCLK_OVERSAM        BRD_DRV_CMD_WORD_SIZE+1
      BRD_DRV_CMD_MCLK_OVERSAM,
      "MCLK frequency",
      "Options: 16, 32, 64, 128, 256, 512 FSYNC",
      uint16_type,
      {.data_uint16 = 1},
      {.data_uint16 = 1024}, // Actually limit is PLL frequency
      uint16_type,
      {.data_uint16 = 1},
      {.data_uint16 = 1024},
      (GD_DATA_VALUE*)&s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling,
      brd_drv_set_MCLK_oversampling
    },
    {
#define BRD_DRV_CMD_MCLK_PPM_OFFSET     BRD_DRV_CMD_MCLK_OVERSAM+1
      BRD_DRV_CMD_MCLK_PPM_OFFSET,
      "Offset of MCLK in PPM",
      "Allow slightly change MCLK frequency."
        " Auto tune PLL have to be disabled!",
      int32_type,
      {.data_int32 = -1000000},
      {.data_int32 =  1000000},
      int32_type,
      {.data_int32 = -1000000},
      {.data_int32 =  1000000},
      (GD_DATA_VALUE*)&s_brd_drv_ssc_fine_settings.i_MCLK_ppm_offset,
      brd_drv_set_MCLK_ppm
    },
    {
#define BRD_DRV_CMD_BCLK_OVERSAM        BRD_DRV_CMD_MCLK_PPM_OFFSET+1
      BRD_DRV_CMD_BCLK_OVERSAM,
      "BCLK frequency",
      "Options: 16, 32, 64, 128, 256, 512 FSYNC",
      uint16_type,
      {.data_uint16 = 1},
      {.data_uint16 = 512},
      uint16_type,
      {.data_uint16 = 1},
      {.data_uint16 = 512},
      (GD_DATA_VALUE*)&s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling,
      brd_drv_set_BCLK_oversampling
    },
    {
#define BRD_DRV_CMD_DIG_AUD_INTERFACE   BRD_DRV_CMD_BCLK_OVERSAM+1
      BRD_DRV_CMD_DIG_AUD_INTERFACE,
      "Digital audio interface mode",
      "0-I2S ; 1-DSP ; 2-Left justified ; 3-Right justified",
      uint32_type,      // Because it is enum on 32 bit AVR must be 32 bit
      {.data_uint32 = 0},
      {.data_uint32 = 3},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 3},
      (GD_DATA_VALUE*)&s_brd_drv_ssc_fine_settings.e_dig_aud_mode,
      brd_drv_set_digital_audio_interface_mode
    },
    {
#define BRD_DRV_CMD_WORD_OFFSET     BRD_DRV_CMD_DIG_AUD_INTERFACE+1
        BRD_DRV_CMD_WORD_OFFSET,
        "Word offset (delay between FSYNC and TX/RX_DATA)",
        "0 ~ 255. Codec safe limit is 16. Value 256 means default value",
        uint16_type,
        {.data_uint16 = 0},
        {.data_uint16 = 256},
        uint16_type,
        {.data_uint16 = 0},
        {.data_uint16 = 256},
        (GD_DATA_VALUE*)&s_brd_drv_ssc_fine_settings.i_word_bit_offset,
        brd_drv_set_word_offset
    },
    {
#define BRD_DRV_CMD_RX_FSYNC_EDGE    BRD_DRV_CMD_WORD_OFFSET+1
      BRD_DRV_CMD_RX_FSYNC_EDGE,
      "RX FSYNC edge",
      "RX FSYNC sync edge ; 0 - falling ; 1 - rising ; 2 - default",
      uint32_type,      // Because it is enum on 32 bit AVR must be 32 bit
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_ssc_fine_settings.e_FSYNC_RX_edge,
      brd_drv_set_FSYNC_RX_edge
    },
    {
#define BRD_DRV_CMD_TX_FSYNC_EDGE    BRD_DRV_CMD_RX_FSYNC_EDGE+1
      BRD_DRV_CMD_TX_FSYNC_EDGE,
      "TX FSYNC edge",
      "TX FSYNC sync edge ; 0 - falling ; 1 - rising ; 2 - default",
      uint32_type,      // Because it is enum on 32 bit AVR must be 32 bit
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_ssc_fine_settings.e_FSYNC_TX_edge,
      brd_drv_set_FSYNC_TX_edge
    },
    {
#define BRD_DRV_CMD_RX_BCLK_EDGE        BRD_DRV_CMD_TX_FSYNC_EDGE+1
      BRD_DRV_CMD_RX_BCLK_EDGE,
      "RX BCLK sampling edge",
      "RX BCLK sync edge ; 0 - falling ; 1 - rising ; 2 - default",
      uint32_type,      // Because it is enum on 32 bit AVR must be 32 bit
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_ssc_fine_settings.e_BCLK_RX_edge,
      brd_drv_set_BCLK_RX_edge
    },
    {
#define BRD_DRV_CMD_TX_BCLK_EDGE        BRD_DRV_CMD_RX_BCLK_EDGE+1
      BRD_DRV_CMD_TX_BCLK_EDGE,
      "TX BCLK transmitting edge",
      "TX BCLK sync edge ; 0 - falling ; 1 - rising ; 2 - default",
      uint32_type,      // Because it is enum on 32 bit AVR must be 32 bit
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_ssc_fine_settings.e_BCLK_TX_edge,
      brd_drv_set_BCLK_TX_edge
    },
    {
#define BRD_DRV_CMD_AUTO_TUNE           BRD_DRV_CMD_TX_BCLK_EDGE+1
      BRD_DRV_CMD_AUTO_TUNE,
      "Auto tune PLL when audio feedback not work",
      "Enable (1) or disable (0)",
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 1},
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 1},
      (GD_DATA_VALUE*)&i_auto_tune_pll,
      brd_drv_set_auto_tune
    },
    {
#define BRD_DRV_CMD_SET_NAME_NUMBER     BRD_DRV_CMD_AUTO_TUNE+1 //BRD_DRV_CMD_TEST_FUNC+1
      BRD_DRV_CMD_SET_NAME_NUMBER,
      "Add number behind device name",
      "After that, device MUST be restarted. 0 means no number (erase number).",
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 15},
      uint8_type,
      {.data_uint8 = 0},
      {.data_uint8 = 15},
      (GD_DATA_VALUE*)&i_product_name_number,
      brd_drv_set_number_to_product_name
    },
    {
#define BRD_DRV_CMD_RESTORE_SETTING     BRD_DRV_CMD_SET_NAME_NUMBER+1
      BRD_DRV_CMD_RESTORE_SETTING,
      "Restore saved settings",
      "Load and apply saved settings",
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      (GD_DATA_VALUE*)&gd_void_value,
      brd_drv_restore_all_settings
    },
    {
#define BRD_DRV_CMD_DEFAULT_SETTINGS    BRD_DRV_CMD_RESTORE_SETTING+1
      BRD_DRV_CMD_DEFAULT_SETTINGS,
      "Load default factory settings (Erase actual settings!)",
      "This function also erase saved settings! So think twice!",
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      (GD_DATA_VALUE*)&gd_void_value,
      brd_drv_load_default_settings
    },
    {
#define BRD_DRV_CMD_SAVE_SETTINGS       BRD_DRV_CMD_DEFAULT_SETTINGS+1
      BRD_DRV_CMD_SAVE_SETTINGS,
      "Save all settings",
      "Just save variables to flash memory",
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      (GD_DATA_VALUE*)&gd_void_value,
      brd_drv_save_all_settings
    },
  };
/// \brief Maximum command ID (is defined by last command)
#define BRD_DRV_MAX_CMD_ID          BRD_DRV_CMD_SAVE_SETTINGS


const gd_metadata BRD_DRV_metadata =
{
        BRD_DRV_MAX_CMD_ID,              // Max CMD ID
        "Board driver for Sonochan mkII v0.6.1",     // Description
        (gd_config_struct*)&BRD_DRV_config_table[0],
        0x0F    // Serial number (0~255)
};
#endif

//=================| Definitions that user should not change |=================
/// Define ADC level when on connector side is save voltage (minimum)
#define BRD_DRV_ADC_SAVE_VOL_MIN                320

/// Define ADC level when on connector side is high voltage
#define BRD_DRV_ADC_HIGH_VOL_MIN                42

//====================| Function prototypes not for user |=====================
GD_RES_CODE brd_drv_TLV_default(void);

GD_RES_CODE brd_drv_adc_init(void);

void brd_drv_process_mute_button(void);

void brd_drv_process_mute_signal(void);

void brd_drv_process_rst_i2s_button(void);

void brd_drv_process_rst_i2s_signal(void);

void brd_drv_clean_up_info_msg(void);

void brd_drv_clean_up_error_msg(void);

//=============================| FreeRTOS stuff |==============================
// If RTOS support is enabled, create this
#if BRD_DRV_SUPPORT_RTOS != 0
portBASE_TYPE xStatus;
xSemaphoreHandle mutexADC;
#endif

#if BRD_DRV_SUPPORT_RTOS != 0
/**
* \brief Board driver FreeRTOS task
*
* Instead of complicating brd_drv_task() we just make function, that is ready\n
* for FreeRTOS stuff, so code will be still easy to use.
*/
void brd_drv_task_FreeRTOS(void *pvParameters)
{
  static uint8_t i_initialized = 0;
  //portTickType xLastWakeTime;

  //xLastWakeTime = xTaskGetTickCount();
  while (TRUE)
  {
    //vTaskDelayUntil(&xLastWakeTime, configTSK_HW_bridge_uniprot_PERIOD);
    vTaskDelay(configTSK_brd_drv_PERIOD);

    /* To be 100% sure, that during board initialization is used pre-defined
     * stack, initialization is done at the task. Onlz once, but in task.
     * So when checking RAM, we can see if heap overflows or not. Because when
     * a lot of memory will be used and initialization will not be in task,
     * it can rewrite some FreeRTOS variables in stack and whole system can
     * freeze.
     */
    if(i_initialized == 0)
    {
      i_initialized = 1;
      if(brd_drv_init() != GD_SUCCESS)
      {
        print_dbg("!!! Board initialization failed.\n If you do not panic, you should now....\n");
        // We can not continue. So let's stay in loop
        while(1)
        {
          print_dbg("!!! Board initialization failed !!!\n\n");
        }
      }
      brd_drv_send_msg(BRD_DRV_MSG_INFO_BRD_DRV_INITIALIZED,1,0,-1);
    }
    // Call simple task
    brd_drv_task();
  }
}
#endif

//==========================| High level functions |===========================
/**
 * \brief Initialization that MUST be done as soon as possible
 *
 * Because some parts should be set as soon as possible (I/O pins, set clock\n
 * for MCU) this function should be called in main on first line.
 *
 * @return GD_SUCCESS (0) if all right
 */

GD_RES_CODE brd_drv_pre_init(void)
{
  // Variable for storing status
  GD_RES_CODE e_status;

  // Set I/O - most fail safe is Hi-Z
  brd_drv_set_isolators_to_HiZ();

  // Clock must be set BEFORE setting UC3 clock
  // Initialize cs2200 library
  e_status = cs2200_init();
  while(e_status != GD_SUCCESS)
  {
    /* Without clock we are doomed. So let's try again, and again and again.
     * Until it success.
     */
    e_status = cs2200_init();
  }

  /* Set some testing frequency. Anyway it will be changed, but we must give
   * UC3 at least some clock.
   */
  e_status = cs2200_set_PLL_freq(8000000);
  while(e_status != GD_SUCCESS)
  {
    /* Again, there MUST be set SOME frequency. So program must try again and
     * again, until some frequency will be set. Without valid clock this is
     * just a garbage. OK, so at least try report this fail
     */
    brd_drv_send_warning_msg(BRD_DRV_MSG_WRN_CAN_NOT_SET_PLL_TRYING_AGAIN,1,1);
    e_status = cs2200_set_PLL_freq(8000000);
  }

  //======================| Route MCLK and set prescaller |====================
  // OSC1 is clocked by 12.288Mhz Osc (Clock from external PLL)
  pm_enable_osc1_ext_clock(&AVR32_PM);
  // from Xtal Oscillator
  pm_enable_clk1(&AVR32_PM, AVR32_PM_OSCCTRL1_STARTUP_2048_RCOSC);

  /*[Martin] Use GCLK0 as master clock because PLL can not produce
   * frequency lower than 6 MHz. So we can use divider. In default we use
   * 48 kHz sampling rate, so divider will be disabled. In previous we decided
   * according to uac1 variable, but nowadays it does not make any sense,
   * because frequency is changed dynamically according to sampling frequency
   */
  gpio_enable_module_pin(MCLK_PIN, MCLK_FUNCTION);
  pm_gc_setup(&AVR32_PM, MCLK_CLK,   // GCLK0
        0,        // OSC source
        1,        // OSC1
        0,        // Divider disabled
        0);       // Divider ratio -> Fout = Fin/( 2*(DIV+1) )
  pm_gc_enable(&AVR32_PM, MCLK_CLK);

  //======================| Route BCLK and set prescaller |====================
  // Route BCLK to GCLK1 module
  gpio_enable_module_pin(BCLK_PIN, BCLK_FUNCTION);
  ///\todo Simplify - Just write to registers
  pm_gc_setup(&AVR32_PM, BCLK_CLK, // gc
          0,                  // osc_or_pll: use Osc (if 0) or PLL (if 1)
          1,                  // pll_osc: select Osc0/PLL0 or Osc1/PLL1
          1,                  // diven - enabled
          1);                 // divided by 4.  Therefore GCLK1 = 3.072Mhz
  // Anyway enable clock
  pm_gc_enable(&AVR32_PM, BCLK_CLK);


  /* If FreeRTOS is used, then create task. Note that
   * "configTSK_brd_drv_*" should be defined in FreeRTOSConfig.h
   * file to keep code clear. However it should be possible set settings
   * here.
   */
#if BRD_DRV_SUPPORT_RTOS != 0
  xTaskCreate(brd_drv_task_FreeRTOS,             // Task function
      configTSK_brd_drv_NAME,       // String name
      configTSK_brd_drv_STACK_SIZE, // Stack size
      NULL,
      configTSK_brd_drv_PRIORITY,   // 0 is lowest
      NULL);
#endif  // FREERTOS_USED

  // If all OK -> return SUCCESS (0)
  return GD_SUCCESS;
}


/**
 * \brief Basic initialization of board components
 *
 * This function initialize rest of hardware on board. It assume, that debug\n
 * print output, clock and I/O pins have been initialized by\n
 * brd_drv_pre_init() function.
 *
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_init(void)
{
  // If RTOS support enable and create flag is set then create mutex
#if (BRD_DRV_SUPPORT_RTOS != 0) && (BRD_DRV_RTOS_CREATE_MUTEX != 0)
  mutexADC = xSemaphoreCreateMutex();
#endif
  // Variable for storing status
  GD_RES_CODE e_status;

  // Load product name number (must be done in runtime)
  i_product_name_number = usb_desc_get_number_from_product_name();

  //=================================| IO pins |===============================
  // Set isolators I/O (recommended default)
  brd_drv_set_isolators_to_HiZ();

  //================================| LCD stuff |==============================
  // Initialize LCD
  e_status = disp_init();
  if(e_status != GD_SUCCESS)
  {
    // Send message over debug interface
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_LCD_INIT_FAIL, 1, 0);
    return e_status;
  }
  brd_drv_send_msg(BRD_DRV_MSG_INFO_INIT_BRD, 1,1,0);

  //==============================| External PLL |=============================
  // Precise setting of PLL
  /* Turn of save setting when changing frequency by 1. We want continuous
   * clock at "any price". PLL CLK OUT should work as expected.
   */
  e_status = cs2200_set_safe_change_ratio_by_1(0);
  if(e_status != GD_SUCCESS)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_PLL_SET_SAVE_FLAG_FAIL, 1 ,1);
    return e_status;
  }


  //===================================| ADC |=================================
  // Take control over ADC
  BRD_DRV_LOCK_ADC_MODULE_IF_RTOS
  // ADC for volume control
  e_status = brd_drv_adc_init();
  // Give back control on ADC
  BRD_DRV_UNLOCK_ADC_MODULE_IF_RTOS

  if(e_status != GD_SUCCESS)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_ADC_INIT_FAIL, 1, 1);
    return e_status;
  }


  //===============================| SSC module |==============================
  // Preset SSC
  e_status = ssc_init();
    if(e_status != GD_SUCCESS)
      return e_status;

  //==================================| Codec |================================
  // Prepare TLV (CS2200 have to be already initialized)
  e_status = brd_drv_TLV_default();
  if(e_status != GD_SUCCESS)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_CODEC_INIT_FAIL, 1, 1);
    return e_status;
  }


  //======================| Try load settings from flash |=====================
  /* Try to restore settings. If fail (invalid setting in user flash),
   * then keep all pins in Hi-Z. But some settings should be defined. So that
   * is why is there if() condition
   */

  e_status = brd_drv_restore_all_settings();
  if(e_status != GD_SUCCESS)
  {
    e_status = brd_drv_load_default_settings();
    if(e_status != GD_SUCCESS) return e_status;
  }

  // Show regular data on LCD
  brd_drv_clean_up_info_msg();

  return GD_SUCCESS;
}








/**
 * \brief Board driver task. Watch ADC and buttons
 *
 * If detected some change, then start processing. Else return
 */
void brd_drv_task(void)
{
  // Simple time counter
  static uint32_t i_time_cnt = 0;

  // Store ADC volume value
  static uint16_t i_saved_volume_value = 0;

  // Set undefined state as default
  static e_brd_drv_con_state e_con_vol = brd_drv_con_undefined;


  // For saving status
  GD_RES_CODE e_status;

  // Variable for store actual volume value
  uint16_t i_actual_volume;

  // Variable for store voltage on connector side
  uint16_t i_voltage_connector_side;

  // Pointer to ADC structure
  volatile avr32_adc_t *p_adc;
  p_adc = (avr32_adc_t*)BDR_DRV_ADC_ADDRESS;


  //===========================| FSYNC request check |=========================
  // Check if there is request for setting FSYNC frequency
  if(s_brd_drv_FSYNC_freq_req.i_request != 0)
  {
    e_status = brd_drv_set_FSYNC_freq(
        s_brd_drv_FSYNC_freq_req.i_new_FSYNC_freq);
    if(e_status != GD_SUCCESS)
    {
      brd_drv_send_error_msg(BRD_DRV_MSG_ERR_SET_FSYNC_FREQ_REQ_FAILED,1,1);
    }
    s_brd_drv_FSYNC_freq_req.i_request = 0;
  }

  /* Check if all ADC conversions are done and if there is "time" for some
   * data processing
   */
  if((BRD_DRV_IS_ADC_DONE(BRD_DRV_ADC_VOLUME_CONTROL_CHANNEL)) &&
     (BRD_DRV_IS_ADC_DONE(BRD_DRV_ADC_CON_VOLTAGE_CHANNEL)) &&
     i_time_cnt > BRD_DRV_BUTTON_REFRESH_PERIOD)
  {
    // ADC ready and timeout occurs -> let's process some data
    i_time_cnt = 0;

    //=============================| Get ADC values |==========================
    // Lock (if needed) ADC and save ADC values
    BRD_DRV_LOCK_ADC_MODULE_IF_RTOS
    i_actual_volume = BRD_DRV_ADC_DATA(BRD_DRV_ADC_VOLUME_CONTROL_CHANNEL);
    i_voltage_connector_side  =
        BRD_DRV_ADC_DATA(BRD_DRV_ADC_CON_VOLTAGE_CHANNEL);
    // Start new conversion
    p_adc->CR.start = 1;
    // Unlock
    BRD_DRV_UNLOCK_ADC_MODULE_IF_RTOS



    //==============================| Change volume |==========================
    // Test if volume value has been changed
    if(i_actual_volume > i_saved_volume_value)
    {
      // Actual volume is higher than saved
      // Check if difference is higher then threshold
      if((i_actual_volume - i_saved_volume_value) >
                                                  BRD_DRV_ADC_UPDATE_THRESHOLD)
      {
        // Save actual ADC value for "next round"
        i_saved_volume_value = i_actual_volume;

        /* Calculate volume value. Because higher value on ADC mean louder output
         * and codec can set volume as attenuation we need to invert value.
         * So maximal attenuation is approximately -79 dB, but ADC give 1023 :/
         * OK, we divide inverted ADC value by 12.9494 (1023/79) so we should
         * valid number in dB. Not so easy, but it work.
         */
        float f_volume = -1*((1023-i_actual_volume)/(12.9494));
        e_status = tlv320aic33_set_headphones_volume_dB(f_volume);
        // Check result
        if(e_status != GD_SUCCESS)
        {
          // If some problem -> tell user
          // Print do debug output and LCD
          brd_drv_send_error_msg(BRD_DRV_MSG_ERR_TLV_FAILED_SET_HEADPHONE_VOL_DB,1,1);
        }
      }
    }
    else // if(i_actual_volume > i_saved_volume_value)
    {
      // Actual volume is lower or equal
      if((i_saved_volume_value - i_actual_volume) >
                                                  BRD_DRV_ADC_UPDATE_THRESHOLD)
      {
        i_saved_volume_value = i_actual_volume;
        float f_volume = -1*((1023-i_actual_volume)/(12.9494));
        e_status = tlv320aic33_set_headphones_volume_dB(f_volume);
        if(e_status != GD_SUCCESS)
        {
          brd_drv_send_error_msg(BRD_DRV_MSG_ERR_TLV_FAILED_SET_HEADPHONE_VOL_DB,1,1);
        }
      }
    }// Test if volume value has been changed






    //=========================| Check connector voltage |=====================
    // Process mute and reset_i2s signals

    // Check CON_VOLTAGE value
    if(i_voltage_connector_side >= BRD_DRV_ADC_SAVE_VOL_MIN)
    {
      // Power off or very low voltage.
      if((e_con_vol != brd_drv_con_low_vol) &&
         (e_con_vol != brd_drv_con_high_vol))
      {
        // If actual status is not same -> change
        e_con_vol = brd_drv_con_low_vol;

        // Send message
        brd_drv_send_error_msg(BRD_DRV_MSG_ERR_CON_NOT_POWERED,1,1);
        // Nothing more to do
      }// Low voltage
    }// Check CON_VOLTAGE value
    else if(i_voltage_connector_side >BRD_DRV_ADC_HIGH_VOL_MIN)
    {
      // Connector side is powered and voltage is in save area
      // Check if previous state was low voltage. If yes, then delete error
      if(e_con_vol == brd_drv_con_low_vol)
      {
        brd_drv_clean_up_error_msg();
      }

      // Check if state is not high voltage (previous state) and save voltage
      if((e_con_vol != brd_drv_con_high_vol) &&
         (e_con_vol != brd_drv_con_save_vol))
      {
        // Save actual state
        e_con_vol = brd_drv_con_save_vol;

        // Send message
        brd_drv_send_msg(BRD_DRV_MSG_INFO_CON_VOL_SAVE, 1, 0, -1);
      }
    }// Check CON_VOLTAGE value
    else
    {
      // Connector side is powered by high voltage!
      // Check if state is different
      if(e_con_vol != brd_drv_con_high_vol)
      {
        // Save actual state
        e_con_vol = brd_drv_con_high_vol;

        // Send warning message
        brd_drv_send_warning_msg(BRD_DRV_MSG_ERR_CON_VOL_HIGH, 1, 1);
      }
    }

    /* If previous state was high voltage and voltage is much
     * lower (approx. +5.0 V)
     */
    if((e_con_vol == brd_drv_con_high_vol) &&
            ((i_voltage_connector_side >(BRD_DRV_ADC_HIGH_VOL_MIN+1) )))
    {
      // Set status to undefined. Next "round" check voltage again...
      e_con_vol = brd_drv_con_undefined;
    }// Check CON_VOLTAGE value


    //=============================| Process signals |=========================
    // Anyway process mute button. Maybe user set up volume too high
    brd_drv_process_mute_button();

    // Reset button process
    brd_drv_process_rst_i2s_button();

  }/* Check if all ADC conversions are done and if there is "time" for some
    * data processing
    */


  // Run display task only if relative time is "correct"
  if(i_time_cnt == 1)
  {
    disp_task();
  }

  // Increase counter (always)
  i_time_cnt++;
}







/**
 * \brief Preset I2S connector to default state. Also reset codec.
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_reset_i2s(void)
{
  GD_RES_CODE e_status;

  // Reset settings of codec to be sure we're have correct settings
  brd_drv_TLV_default();

  // Codec is ready, so let's set factory settings.
  e_status = brd_drv_load_default_settings();
  if(e_status != GD_SUCCESS) return e_status;

  // Send message to debug interface
  brd_drv_send_msg(BRD_DRV_MSG_INFO_RESET_I2S_DONE, 1, 0, -1);

  return GD_SUCCESS;
}



/**
 * @brief Simple function, that allow set digital audio interface mode
 *
 * Can switch between I2S, DSP, Right justify and so on.
 * @param e_mode Options: SSC_I2S, SSC_DSP, SSC_LEFT_JUSTIFIED,\n
 *   SSC_RIGHT_JUSTIFIED
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_set_digital_audio_interface_mode(
    e_ssc_digital_audio_interface_t e_mode)
{
  // Status code
  GD_RES_CODE e_status;

  /* Because some functions need to know at which mode they operate, we need
   * to change digital audio mode ASAP. But in case of some failure we should
   * use old value (switch was not successful)
   */
  e_ssc_digital_audio_interface_t e_dig_aud_mode_backup =
                                    s_brd_drv_ssc_fine_settings.e_dig_aud_mode;
  s_brd_drv_ssc_fine_settings.e_dig_aud_mode = e_mode;

  e_status = ssc_set_digital_interface_mode(e_mode);
  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.e_dig_aud_mode = e_dig_aud_mode_backup;
    return e_status;
  }

  // Also set codec interface (different values)
  switch(e_mode)
  {
  case SSC_I2S:
    /* This is not error. Because codec count word offset different way, we
     * just use left justified mode to avoid some troubles with word offset.
     * This will make code more simple and codec will work in much more cases.
     * But we have to set word offset after that.
     */
    e_status = tlv320aic33_set_data_interface(
                                   serial_data_bus_uses_left_justified_mode);
    break;
  case SSC_DSP:
    e_status = tlv320aic33_set_data_interface(serial_data_bus_uses_DSP_mode);
    break;
  case SSC_LEFT_JUSTIFIED:
    e_status = tlv320aic33_set_data_interface(
                                   serial_data_bus_uses_left_justified_mode);
    break;
  case SSC_RIGHT_JUSTIFIED:
    /* Because RIGHT justified is tricky to keep in sync with SSC on AVR, we
     * just use at codec LEFT justified and we will shift word by correct
     * offset. This actually allow us listen data in much more cases without
     * any pain. If we would use right justified on codec, word offset is count
     * little bit different way and that make troubles. But we have to also set
     * correct word offset after that
     */
    e_status = tlv320aic33_set_data_interface(
                                   serial_data_bus_uses_left_justified_mode);
    break;
  default:
    // Unknown state -> error
    return GD_FAIL;
  }

  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.e_dig_aud_mode = e_dig_aud_mode_backup;
    return e_status;
  }

  /* Now set all fine configuration settings again. This is because SSC driver's
   * behavior is "set mode and all to default to make it working". But because
   * we're using generic driver support, we need to keep all settings to avoid
   * de-synchronization between application and real setting in Sonochan mkII.
   *
   * This is automatically done at setting word length if RJF is used.
   */
  if(s_brd_drv_ssc_fine_settings.e_dig_aud_mode != SSC_RIGHT_JUSTIFIED)
  {
    e_status = brd_drv_set_word_offset(
                 s_brd_drv_ssc_fine_settings.i_word_bit_offset);
    if(e_status != GD_SUCCESS)
    {
      s_brd_drv_ssc_fine_settings.e_dig_aud_mode = e_dig_aud_mode_backup;
      return e_status;
    }
  }


  e_status = ssc_set_BCLK_RX_edge(s_brd_drv_ssc_fine_settings.e_BCLK_RX_edge);
  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.e_dig_aud_mode = e_dig_aud_mode_backup;
    return e_status;
  }

  e_status = ssc_set_BCLK_TX_edge(s_brd_drv_ssc_fine_settings.e_BCLK_TX_edge);
  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.e_dig_aud_mode = e_dig_aud_mode_backup;
    return e_status;
  }

  e_status = ssc_set_FSYNC_RX_edge(s_brd_drv_ssc_fine_settings.e_FSYNC_RX_edge);
  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.e_dig_aud_mode = e_dig_aud_mode_backup;
    return e_status;
  }

  e_status = ssc_set_FSYNC_TX_edge(s_brd_drv_ssc_fine_settings.e_FSYNC_TX_edge);
  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.e_dig_aud_mode = e_dig_aud_mode_backup;
    return e_status;
  }

  // Refresh data length (show warning if needed)
  e_status = brd_drv_set_data_length(s_brd_drv_ssc_fine_settings.i_data_length);
  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.e_dig_aud_mode = e_dig_aud_mode_backup;
    return e_status;
  }

  // BCLK oversampling value is 2x frame_length
  e_status = ssc_set_frame_length(
                             s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling>>1);
  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.e_dig_aud_mode = e_dig_aud_mode_backup;
    return e_status;
  }

  // BCLK direction
  e_status = brd_drv_set_bclk_dir(s_brd_drv_pure_dai_dir.e_bclk_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! BCLK dir fail ! (Set DAI)\n");
    return e_status;
  }

  // FRAME SYNC direction
  e_status=brd_drv_set_fsync_dir(s_brd_drv_pure_dai_dir.e_fsync_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! FS dir fail ! (Set DAI)\n");
    return e_status;
  }

  // Inform about set interface to debug
  switch(e_mode)
  {
  case SSC_I2S:
    brd_drv_send_msg(BRD_DRV_MSG_INFO_FMT_I2S, 1, 0, -1);
    break;
  case SSC_DSP:
    brd_drv_send_msg(BRD_DRV_MSG_INFO_FMT_DSP, 1, 0, -1);
    break;
  case SSC_RIGHT_JUSTIFIED:
    brd_drv_send_msg(BRD_DRV_MSG_INFO_FMT_R_JUS, 1, 0, -1);
    break;
  case SSC_LEFT_JUSTIFIED:
    brd_drv_send_msg(BRD_DRV_MSG_INFO_FMT_L_JUS, 1, 0, -1);
    break;
  default:
    /* This should never happend. Only in case, that code is not complete
     * or badly written
     */
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_UNKNOWN_DIG_ITF_MODE, 1, 1);
    e_status = GD_FAIL;
  }


  // Anyway, return status code
  return e_status;
}


/**
 * @brief Simple function, that allow get digital audio interface mode
 *
 * @param p_e_mode Pointer to memory, where result will be written.
 *         Options: SSC_I2S, SSC_DSP, SSC_LEFT_JUSTIFIED, SSC_RIGHT_JUSTIFIED
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_get_digital_audio_interface_mode(
    e_ssc_digital_audio_interface_t *p_e_mode)
{
  *p_e_mode = s_brd_drv_ssc_fine_settings.e_dig_aud_mode;
  return GD_SUCCESS;
}


/**
 * @brief Set FSYNC frequency
 * Because all is based from external PLL frequency, function set also\n
 * external PLL, MCLK and BCLK dividers (on UC3A). As result of all this\n
 * should be correct FSYNC frequency.
 *
 * @param i_FSYNC_freq FSYNC frequency in Hz
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_set_FSYNC_freq(uint32_t i_FSYNC_freq)
{
  // Store status code
  GD_RES_CODE e_status;

  /* Because function set MCLK oversampling need know frequency, we just backup
   * actual value, and if something fucks up, then just restore it.
   */
  uint32_t i_FSYNC_freq_backup = s_brd_drv_ssc_fine_settings.i_FSYNC_freq;

  // Rewrite with actual value
  s_brd_drv_ssc_fine_settings.i_FSYNC_freq = i_FSYNC_freq;

  // Set MCLK. Because function can change PLL freq, itself set BCLK again
  e_status = brd_drv_set_MCLK_oversampling(
      s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling);
  if(e_status != GD_SUCCESS)
  {
    // If fail, restore previous value
    s_brd_drv_ssc_fine_settings.i_FSYNC_freq = i_FSYNC_freq_backup;
    return e_status;
  }

  return e_status;
}



/**
 * @brief Give last FSYNC value, which was set
 *
 * This is only value, that was set. Due to some drifts and possible problems\n
 * with feedback EP can be frequency slightly different.
 * @param p_i_FSYNC_freq Pointer to memory, where result will be written.\n
 *  Frequency is in Hz.
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_get_FSYNC_freq(uint32_t *p_i_FSYNC_freq)
{
  *p_i_FSYNC_freq = s_brd_drv_ssc_fine_settings.i_FSYNC_freq;
  return GD_SUCCESS;
}



/**
 * @brief Just request to set FSYNC frequency. Will be executed later
 *
 * Because at USB stack we need really fast response, there is no time to set
 * FSYNC frequency through brd_drv_set_FSYNC_freq() function. So we simply
 * send request and frequency will be set shortly.
 * @param i_FSYNC_freq FSYNC frequency in Hz
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_set_FSYNC_freq_rqst(uint32_t i_FSYNC_freq)
{
  s_brd_drv_FSYNC_freq_req.i_new_FSYNC_freq = i_FSYNC_freq;
  s_brd_drv_FSYNC_freq_req.i_request = 1;
  return GD_SUCCESS;
}


/**
 * @brief Set MCLK oversampling frequency in reference to FSYNC
 * Also set again BCLK, because function itself can change PLL frequency,\n
 * but BCLK divider ratio would stayed unchanged -> wrong BCLK. So that is\n
 * why function set also BCLK again.
 *
 * @param i_MCLK_oversampling Options: 16, 32, 64, 128, 256 and 512
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_MCLK_oversampling(uint16_t i_MCLK_oversampling)
{
  uint16_t i_MCLK_ovrsmplng_backup =
              s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling;

  // MCLK can not be 0!
  if(i_MCLK_oversampling == 0)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_MCLK_OVERSAM_CANNOT_BE_0,1,1);
    return GD_INCORRECT_PARAMETER;
  }

  // If FSYNC is 0 -> can not calculate MCLK clock
  if(s_brd_drv_ssc_fine_settings.i_FSYNC_freq == 0)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_FSYNC_CAN_NOT_BE_ZERO,1,1);
    return GD_INCORRECT_PARAMETER;
  }

  /* Check if BCLK is exact multiplier of MCLK. If not we have to change MCLK
   * according to BCLK to be synchronized
   */
  if((i_MCLK_oversampling % s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling)
      != 0)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_BCLK_IS_NOT_MUL_OF_MCLK, 1,1);
    return GD_INCORRECT_PARAMETER;
  }


  /* Oversampling over 512 is possible, but:
   * 1) Usually not supported even by codec
   * 2) Too high frequency is problem for PLL
   * 3) In real I never ever saw device that support this
   * Anyway, when user call this function and value will be higher than 512
   * we just show warning
   */
  if(i_MCLK_oversampling > 512)
  {
    brd_drv_send_warning_msg(BRD_DRV_MSG_WRN_MCLK_OVRSM_HIGH, 1, 1);
  }
  //=======================| End of checking parameters |========================

  /* This function must be very flexible and work with PLL CS2200 (set freq.),
   * set correct divide ratio for GCLK0 and if all OK -> save
   * actual value (MCLK oversampling)
   */

  // Keep result code in memory
  GD_RES_CODE e_status;

  // Temporary variable for calculating MLCLK divider on UC3A GCLK0
  uint16_t i_MCLK_div = 1;

  // Temporary variable to keep calculated PLL frequency
  uint32_t i_PLL_freq;

  // Get some guess - FSYNC freq * MCLK oversampling value
  i_PLL_freq = ((uint32_t)i_MCLK_oversampling) *
                          s_brd_drv_ssc_fine_settings.i_FSYNC_freq;

  // Find correct PLL frequency
  while(1)
  {
    /* Because PLL CS2200 have own limitations, we must check if frequency is
     * not too high or low.
     */
    if(i_PLL_freq > CS2200_MAX_FREQ)
    {
      // PLL can not produce higher frequency :( Return fail
      brd_drv_send_error_msg(BRD_DRV_MSG_ERR_PLL_HIGH_FREQ,1,1);
      return GD_FAIL;
    }
    if(i_PLL_freq < CS2200_MIN_FREQ)
    {
      /* Can not produce such a low frequency, but can set divider. So just
       * double PLL frequency and double MCLK divider.
       */
      i_PLL_freq = i_PLL_freq<<1;
      // Thanks to shifting we never get odd divider (3, 5, 7 and so on)
      i_MCLK_div = i_MCLK_div<<1;
    }
    else // Else PLL frequency is OK
    {
      break;
    }
  }

  // OK, so now we have PLL frequency and MCLK divider value
  // Set MCLK divider
  e_status = brd_drv_set_MCLK_div(i_MCLK_div);
  if(e_status != GD_SUCCESS)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_MCLK_DIV_FAIL,1,1);
    return e_status;
  }

  /* So far so good. So let's set MCLK oversampling value, because following
   * functions will need it. Anyway, if some problem occur, value will be
   * restored
   */
  s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling = i_MCLK_oversampling;

  /* Check if auto tune option enabled. If yes, just set PLL value directly.
   * If not, then we set PLL with MCLK PPM offset
   */
  if(i_auto_tune_pll)
  {
    // Auto tune enabled -> only PLL freq
    e_status = cs2200_set_PLL_freq(i_PLL_freq);
    if(e_status != GD_SUCCESS)
    {
      brd_drv_send_error_msg(BRD_DRV_MSG_ERR_CAN_NOT_SET_PLL_FREQ_MCLK,1,1);
      return e_status;
    }
  }// Auto tune enabled
  else // Auto tune disabled
  {
    // Set MCLK PPM offset + PLL frequency (brd_drv_set_MCLK_ppm will set PLL f)
    e_status = brd_drv_set_MCLK_ppm(
                 s_brd_drv_ssc_fine_settings.i_MCLK_ppm_offset);
    if(e_status != GD_SUCCESS)
    {
      s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling = i_MCLK_ovrsmplng_backup;
      brd_drv_send_error_msg(BRD_DRV_MSG_ERR_CAN_NOT_SET_MCLK_PPM_OFFSET, 1,1);
      return e_status;
    }
  }

  /* Set again BCLK divider (resp. oversampling value)
   * This function SHOULD NOT fail, but just for case there is if condition.
   * If there is any problem, it is probably bug, not user fail.
   */
  e_status = brd_drv_set_BCLK_oversampling(
      s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling);
  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling = i_MCLK_ovrsmplng_backup;
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_MCLK_OVRSAM_CAN_NOT_SET_BCLK_OVRSAM,
                           1,1);
    return e_status;
  }

  return e_status;
}



/**
 * @brief Give MCLK oversampling value in reference to FSYNC
 * @param p_i_MCLK_oversampling Pointer to memmory, where result will be\n
 * written. Options: 16, 32, 64, 128 and 256.
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_get_MCLK_oversampling(uint16_t *p_i_MCLK_oversampling)
{
  *p_i_MCLK_oversampling = s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling;
  return GD_SUCCESS;
}


/**
 * @brief Give MCLK frequency in Hz
 * @param p_i_MCLK_freq Pointer to memory where result will be written
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_get_MCLK_freq(uint32_t *p_i_MCLK_freq)
{
  GD_RES_CODE e_status;

  uint32_t i_PLL_freq;

  uint16_t i_MCLK_div;

  // Get real PLL frequency
  e_status = cs2200_get_PLL_freq(&i_PLL_freq);
  if(e_status != GD_SUCCESS) return e_status;

  // Get MCLK divider
  e_status = brd_drv_get_MCLK_div(&i_MCLK_div);
  if(e_status != GD_SUCCESS) return e_status;

  // Simply calculate MCLK
  *p_i_MCLK_freq = i_PLL_freq / (uint32_t)i_MCLK_div;

  return GD_SUCCESS;
}


/**
 * @brief Set BCLK oversampling in reference to FSYNC
 *
 * Function set GCLK1 (divider) and SSC module to produce right clock
 *
 * @param i_BCLK_ovrsmpling Options: 16, 32, 64, 128 and 256
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_BCLK_oversampling(uint16_t i_BCLK_ovrsmpling)
{
  // Store status code
  GD_RES_CODE e_status;

  // Variable for store settings of MCLK divider
  uint16_t i_MCLK_div;

  // Relative divide (MCLK oversampling)/(BLCK oversampling) ratio
  uint16_t i_MCLK_rel_div;

  // BCLK oversampling value can not be 0
  if(i_BCLK_ovrsmpling == 0)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_BCLK_OVERSAM_CANNOT_BE_0, 1,1);
    return GD_INCORRECT_PARAMETER;
  }


  // Check MCLK oversampling value. BCLK must not be higher than MCLK
  if(i_BCLK_ovrsmpling > s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_MCLK_LOWER_THAN_BCLK, 1,1);
    return GD_INCORRECT_PARAMETER;
  }// Check if BCLK > MCLK

  // Also must check data word size.
  if((i_BCLK_ovrsmpling>>1) < s_brd_drv_ssc_fine_settings.i_data_length )
  {
    /* This case means, that we have for example 24 bit resolution, but
     * due to BCLK oversampling value, we can process less bits. Anyway this
     * cause bit precision loss, so report error.
     */
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_BCLK_LOWER_THAN_DATA_WORD, 1, 1);
    return GD_INCORRECT_PARAMETER;
  }

  /* Check if BCLK is exact multiplier of MCLK. If not we have to change MCLK
   * according to BCLK to be synchronized
   */
  if((s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling % i_BCLK_ovrsmpling) != 0)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_BCLK_IS_NOT_MUL_OF_MCLK, 1,1);
    return GD_INCORRECT_PARAMETER;
  }

  // Our codec does not support BCLK lower than 32 -> in that case show message
  if(i_BCLK_ovrsmpling < 32)
  {
    brd_drv_send_warning_msg(BRD_DRV_MSG_WRN_CODEC_NOT_SUPP_LOWER_THAN_BCLK32,
                             1,1);
  }
  //=======================| End of checking parameters |======================
  // Calculate relative ratio (MCLK/BCLK)
  i_MCLK_rel_div = s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling /
                            i_BCLK_ovrsmpling;

  /* Because divider for MCLK can have already set some ratio, we need to add
   * both values to get correct value for divider
   */
  e_status = brd_drv_get_MCLK_div(&i_MCLK_div);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }


  // Calculate complete ratio
  i_MCLK_rel_div = i_MCLK_rel_div * i_MCLK_div;

  /* Check ratio. If ratio is odd value (3, 5, 7, 9....) we need to double it
   * to get not-odd value. But that means that we need to double PLL frequency
   * and also we need to double MCLK divider
   */
  if(i_MCLK_rel_div & 0x0001)
  {
    // Odd number -> double divider
    brd_drv_send_msg(BRD_DRV_MSG_INFO_DOUBLING_PLL_FREQ_SET_BCLK_ODD_MUL,
                     1,0,-1);

    i_MCLK_rel_div = i_MCLK_rel_div<<1;

    i_MCLK_div = i_MCLK_div<<1;
    e_status = brd_drv_set_MCLK_div(i_MCLK_div);
    if(e_status != GD_SUCCESS)
    {
      return e_status;
    }

    /* Because we changed MCLK we need to change also MCLK itself. And because
     * there is also MCLK PPM offset, which is enabled only if auto tune PLL
     * function is off, we need to check it first. Then we can decide if we
     * change frequency directly of through MCLK_ppm function.
     */
    if(i_auto_tune_pll)
    {
      uint32_t i_pll_freq;
      e_status = cs2200_get_PLL_freq(&i_pll_freq);
      if(e_status != GD_SUCCESS) return e_status;
      // Need to double frequency, because we already double MCLK divider
      i_pll_freq = i_pll_freq<<1;

      e_status = cs2200_set_PLL_freq(i_pll_freq);
      if(e_status != GD_SUCCESS)
      {
        brd_drv_send_error_msg(BRD_DRV_MSG_ERR_CAN_NOT_SET_PLL_FREQ_BCLK,1,1);
        return e_status;
      }
    }// Auto tune enabled
    else // Auto tune disabled
    {
      // We can set MCLK with offset
      e_status = brd_drv_set_MCLK_ppm(
                    s_brd_drv_ssc_fine_settings.i_MCLK_ppm_offset);
      if(e_status != GD_SUCCESS)
      {
        brd_drv_send_error_msg(
            BRD_DRV_MSG_ERR_CAN_NOT_SET_MCLK_PPM_OFFSET, 1,1);
        return e_status;
      }
    }
  }



  // Set correct BCLK
  e_status = brd_drv_set_BCLK_div(i_MCLK_rel_div);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Also need to set SSC module - frame length is 1/2 of BCLK oversampling
  e_status = ssc_set_frame_length(i_BCLK_ovrsmpling>>1);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // If all OK -> save value
  if(e_status == GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling = i_BCLK_ovrsmpling;
    // Also we need to recalculate word offset if RJF is used
    if(s_brd_drv_ssc_fine_settings.e_dig_aud_mode == SSC_RIGHT_JUSTIFIED)
    {
      e_status = brd_drv_set_word_offset(
                    s_brd_drv_ssc_fine_settings.i_word_bit_offset);
      if(e_status != GD_SUCCESS) return e_status;
    }
  }

  return e_status;
}


/**
 * @brief Give BCLK oversampling value in reference to FSYNC
 * @param p_i_BCLK_oversampling Memory address, where result will be written.\n
 * Options: 16, 32, 64, 128 and 256
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE brd_drv_get_BCLK_oversampling(uint16_t *p_i_BCLK_oversampling)
{
  *p_i_BCLK_oversampling = s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling;
  return GD_SUCCESS;
}


/**
 * @brief Give actual volume value in dB
 *
 * Get volume directly from codec as float.
 *
 * @param p_f_volume Pointer to memory, where result will be written. Volume
 *  is in dB.
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE brd_drv_get_volume_dB(float *p_f_volume)
{
  return tlv320aic33_get_headphones_volume_db(p_f_volume);
}


/**
 * @brief Set number behind device name
 *
 * This number is part of device name, so it will be shown in all\n
 * applications. Also change PID value, so every single device can be\n
 * recognized.
 *
 *
 * @param i_number Options: 0-16 set custom number (also PID)
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_number_to_product_name(uint8_t i_number)
{
  // Check input parameter
  if(i_number > 15) return GD_INCORRECT_PARAMETER;

  usb_desc_set_number_to_product_name(i_number);

  uac1_usb_desc_set_PID(AUDIO_PRODUCT_ID_1+i_number);

  // OK, but for taking effect we must reset device. So just for case show
  // error message to make sure, that user will notice.
  brd_drv_send_error_msg(BRD_DRV_MSG_ERR_DEVICE_RESTART_REQUIRED,1,1);

  return GD_SUCCESS;
}


/**
 * @brief Get number behind device name
 *
 * This number is part of device name, so it will be shown in all\n
 * applications. Also change PID value, so every single device can be\n
 * recognized.
 *
 * @param p_i_number Pointer to memory, where result will be written
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_get_number_from_product_name(uint8_t *p_i_number)
{
  *p_i_number = i_product_name_number;
  return GD_SUCCESS;
}

/**
 * \brief Save all needed settings to EEPROM like memory
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_save_all_settings(void){
  /* Save settings
   * Remember: this is 32 bit arch. So even stupid enum is 32 bit long. That is
   * why is saved to flash as 32 bit value even if values is not even 8 bit
   * wide.
   */


  // MUTE direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_mute_dir,      // Where to write
      s_brd_drv_mute.e_mute_dir,                        // From where
      sizeof(s_brd_drv_mute.e_mute_dir),                // Number of Bytes
      1);                                               // Erase?

  // RESET direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_rst_dai_dir,
      s_brd_drv_rst_i2s.e_rst_dai_dir,
      sizeof(s_brd_drv_rst_i2s.e_rst_dai_dir),
      1);

  // MCLK direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_mclk_dir,
      (uint32_t)s_brd_drv_pure_dai_dir.e_mclk_dir,
      sizeof(s_brd_drv_pure_dai_dir.e_mclk_dir),
      1);

  // BCLK direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_bclk_dir,
      s_brd_drv_pure_dai_dir.e_bclk_dir,
      sizeof(s_brd_drv_pure_dai_dir.e_bclk_dir),
      1);

  // FRAME SYNC direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_fsync_dir,
      s_brd_drv_pure_dai_dir.e_fsync_dir,
      sizeof(s_brd_drv_pure_dai_dir.e_fsync_dir),
      1);

  // TX DATA direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_tx_data_dir,
      s_brd_drv_pure_dai_dir.e_tx_data_dir,
      sizeof(s_brd_drv_pure_dai_dir.e_tx_data_dir),
      1);

  // RX DATA direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_rx_data_dir,
      s_brd_drv_pure_dai_dir.e_rx_data_dir,
      sizeof(s_brd_drv_pure_dai_dir.e_rx_data_dir),
      1);

  // Auto tune PLL option
  flashc_memset8(
      (void *)&s_brd_drv_user_settings.i_auto_tune_pll,
      i_auto_tune_pll,
      sizeof(i_auto_tune_pll),
      1);

  // SSC fine settings
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.i_FSYNC_freq,
      s_brd_drv_ssc_fine_settings.i_FSYNC_freq,
      sizeof(s_brd_drv_ssc_fine_settings.i_FSYNC_freq),
      1);
  flashc_memset8(
      (void *)&s_brd_drv_user_settings.i_data_length,
      s_brd_drv_ssc_fine_settings.i_data_length,
      sizeof(s_brd_drv_ssc_fine_settings.i_data_length),
      1);
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_dig_audio_mode,
      s_brd_drv_ssc_fine_settings.e_dig_aud_mode,
      sizeof(s_brd_drv_ssc_fine_settings.e_dig_aud_mode),
      1);
  flashc_memset16(
      (void *)&s_brd_drv_user_settings.i_MCLK_ovrsmpling,
      s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling,
      sizeof(s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling),
      1);
  flashc_memset16(
      (void *)&s_brd_drv_user_settings.i_BCLK_ovrsmpling,
      s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling,
      sizeof(s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling),
      1);
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_BCLK_RX_edge,
      s_brd_drv_ssc_fine_settings.e_BCLK_RX_edge,
      sizeof(s_brd_drv_ssc_fine_settings.e_BCLK_RX_edge),
      1);
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_BCLK_TX_edge,
      s_brd_drv_ssc_fine_settings.e_BCLK_TX_edge,
      sizeof(s_brd_drv_ssc_fine_settings.e_BCLK_TX_edge),
      1);
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_FSYNC_RX_edge,
      s_brd_drv_ssc_fine_settings.e_FSYNC_RX_edge,
      sizeof(s_brd_drv_ssc_fine_settings.e_FSYNC_RX_edge),
      1);
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_FSYNC_TX_edge,
      s_brd_drv_ssc_fine_settings.e_FSYNC_TX_edge,
      sizeof(s_brd_drv_ssc_fine_settings.e_FSYNC_TX_edge),
      1);
  flashc_memset8(
      (void *)&s_brd_drv_user_settings.i_FSYNC_pulse,
      s_brd_drv_ssc_fine_settings.i_FSYNC_pulse,
      sizeof(s_brd_drv_ssc_fine_settings.i_FSYNC_pulse),
      1);
  flashc_memset16(
      (void *)&s_brd_drv_user_settings.i_word_bit_offset,
      s_brd_drv_ssc_fine_settings.i_word_bit_offset,
      sizeof(s_brd_drv_ssc_fine_settings.i_word_bit_offset),
      1);

  flashc_memset32(
      (void *)&s_brd_drv_user_settings.i_MCLK_ppm_offset,
      s_brd_drv_ssc_fine_settings.i_MCLK_ppm_offset,
      sizeof(s_brd_drv_ssc_fine_settings.i_MCLK_ppm_offset),
      1);


  // Save information (code), that settings was saved
  flashc_memset8(
      (void *)&i_brd_drv_settings_check,
      BRD_DRV_FLASH_CHECK_CODE,
      sizeof(BRD_DRV_FLASH_CHECK_CODE),
      1);

  brd_drv_send_msg(BRD_DRV_MSG_INFO_SETTINGS_SAVED, 1,0,-1);
  return GD_SUCCESS;
}








/**
 * \brief Load all needed settings to SRAM memory and apply settings
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_restore_all_settings(void){
  // For storing status
  GD_RES_CODE e_status;

  // Check if settings in user flash is valid (BRD_DRV_FLASH_CHECK_CODE)
  if(i_brd_drv_settings_check != BRD_DRV_FLASH_CHECK_CODE)
  {
    // Not equal -> can not restore settings
    brd_drv_send_warning_msg(BRD_DRV_MSG_WRN_FLASH_NOT_VALID_SETTINGS,1,0);
    return GD_FAIL;
  }

  //========================| Load settings from memory |======================

  //========================| Loading signal direction |=======================
  // Settings is correct
  brd_drv_send_msg(BRD_DRV_MSG_INFO_FLASH_VALID_SETTINGS,1,0,-1);

  // Apply settings
  ///\todo Remove debug messages when all tested and everything work

  // MUTE direction
  e_status = brd_drv_set_mute_dir(s_brd_drv_user_settings.e_mute_dir);
  if(e_status != GD_SUCCESS)    // Check status
  {
    print_dbg("MUTE direction failed!\n");
    return e_status;
  }

  // RESET direction
  e_status = brd_drv_set_rst_dai_dir(s_brd_drv_user_settings.e_rst_dai_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg("RESET direction failed\n");
    return e_status;
  }

  // MCLK direction
  e_status = brd_drv_set_mclk_dir(s_brd_drv_user_settings.e_mclk_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg("MCLK direction failed!\n");
    return e_status;
  }

  /* BCLK and FRAME SYNC direction is part of
   * brd_drv_set_digital_audio_interface_mode(), but we need to make sure
   * there is clock and if not, when fallback to default value. Simply try
   * to set it and if everything is OK, function that set interface just
   * set SSC once again according to actual values
   */
  // BCLK direction
  e_status = brd_drv_set_bclk_dir(s_brd_drv_user_settings.e_bclk_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! BCLK dir fail ! \n");
    return e_status;
  }

  // FRAME SYNC direction
  e_status=brd_drv_set_fsync_dir(s_brd_drv_user_settings.e_fsync_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! FS dir fail ! \n");
    return e_status;
  }

  // TX DATA
  e_status = brd_drv_set_tx_data_dir(s_brd_drv_user_settings.e_tx_data_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! TXD dir fail ! \n");
    return e_status;
  }

  // RX DATA
  e_status = brd_drv_set_rx_data_dir(s_brd_drv_user_settings.e_rx_data_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! RXD dir fail ! \n");
    return e_status;
  }


  //==========================| Loading SSC settings |=========================
  // SSC fine settings
  // Load now, apply later (because of dependencies)
  s_brd_drv_ssc_fine_settings.e_dig_aud_mode =
      s_brd_drv_user_settings.e_dig_audio_mode;

  s_brd_drv_ssc_fine_settings.e_FSYNC_RX_edge =
      s_brd_drv_user_settings.e_FSYNC_RX_edge;
  s_brd_drv_ssc_fine_settings.e_FSYNC_TX_edge =
      s_brd_drv_user_settings.e_FSYNC_TX_edge;

  s_brd_drv_ssc_fine_settings.e_BCLK_RX_edge =
      s_brd_drv_user_settings.e_BCLK_RX_edge;
  s_brd_drv_ssc_fine_settings.e_BCLK_TX_edge =
      s_brd_drv_user_settings.e_BCLK_TX_edge;

  s_brd_drv_ssc_fine_settings.i_FSYNC_pulse =
      s_brd_drv_user_settings.i_FSYNC_pulse;

  s_brd_drv_ssc_fine_settings.i_word_bit_offset =
      s_brd_drv_user_settings.i_word_bit_offset;

  s_brd_drv_ssc_fine_settings.i_data_length =
      s_brd_drv_user_settings.i_data_length;

  s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling =
      s_brd_drv_user_settings.i_MCLK_ovrsmpling;

  s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling =
      s_brd_drv_user_settings.i_BCLK_ovrsmpling;

  s_brd_drv_ssc_fine_settings.i_FSYNC_freq =
      s_brd_drv_user_settings.i_FSYNC_freq;

  s_brd_drv_ssc_fine_settings.i_MCLK_ppm_offset =
      s_brd_drv_user_settings.i_MCLK_ppm_offset;

  i_auto_tune_pll = s_brd_drv_user_settings.i_auto_tune_pll;


  // Using same/similar order as used when loading factory settings
  e_status = brd_drv_set_data_length(
                  s_brd_drv_ssc_fine_settings.i_data_length);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! Set data length failed ! \n");
    return e_status;
  }

  // And call set FSYNC frequency, which set MCLK and BCLK dividers
  e_status = brd_drv_set_FSYNC_freq(s_brd_drv_ssc_fine_settings.i_FSYNC_freq);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! FSYNC failed ! \n");
    return e_status;
  }

  e_status = brd_drv_set_digital_audio_interface_mode(
      s_brd_drv_user_settings.e_dig_audio_mode);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! Set dig. aud. itfce failed ! \n");
    return e_status;
  }

  e_status = brd_drv_set_BCLK_RX_edge(
      s_brd_drv_user_settings.e_BCLK_RX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! BCLK RX edge fail ! \n");
    return e_status;
  }

  e_status = brd_drv_set_BCLK_TX_edge(
      s_brd_drv_user_settings.e_BCLK_TX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! BCLK TX edge fail ! \n");
    return e_status;
  }

  e_status = brd_drv_set_FSYNC_RX_edge(
      s_brd_drv_user_settings.e_FSYNC_RX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! RX FSYNC dir fail ! \n");
    return e_status;
  }

  e_status = brd_drv_set_FSYNC_TX_edge(
      s_brd_drv_user_settings.e_FSYNC_TX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! FSYNC TX edge fail ! \n");
    return e_status;
  }



  e_status = brd_drv_set_word_offset(
      s_brd_drv_user_settings.i_word_bit_offset);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! WORD bit OFFSET fail ! \n");
    return e_status;
  }

  // Auto tune PLL
  e_status = brd_drv_set_auto_tune(s_brd_drv_user_settings.i_auto_tune_pll);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! Auto tune PLL fail ! \n");
    return e_status;
  }


  brd_drv_send_msg(BRD_DRV_MSG_INFO_SETTINGS_LOADED, 1,0,-1);
  // Just return status of last function
  return e_status;
}






/**
 * @brief Load factory settings no matter what
 *
 * This function just load settings, but does not save it! For saving
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE brd_drv_load_default_settings(void)
{
  // Store status codes
  GD_RES_CODE e_status;

  brd_drv_send_msg(BRD_DRV_MSG_INFO_LOAD_FACTRY_STTNGS,1,0,-1);


  //============================| Direction signals |==========================
  e_status = brd_drv_set_mute_dir(BRD_DRV_DEFAULT_MUTE_DIR);
  if(e_status != GD_SUCCESS) return e_status;

  e_status = brd_drv_set_rst_dai_dir(BRD_DRV_DEFAULT_RESET_DAI_DIR);
  if(e_status != GD_SUCCESS) return e_status;
#if BRD_DRV_DEBUG != 1
  // Not debug mode
  e_status = brd_drv_set_mclk_dir(BRD_DRV_DEFAULT_MCLK_DIR);
  if(e_status != GD_SUCCESS) return e_status;

  e_status = brd_drv_set_bclk_dir(BRD_DRV_DEFAULT_BCLK_DIR);
  if(e_status != GD_SUCCESS) return e_status;

  e_status = brd_drv_set_fsync_dir(BRD_DRV_DEFAULT_FSYNC_DIR);
  if(e_status != GD_SUCCESS) return e_status;

  e_status = brd_drv_set_tx_data_dir(BRD_DRV_DEFAULT_DATA_TX_DIR);
  if(e_status != GD_SUCCESS) return e_status;

  e_status = brd_drv_set_rx_data_dir(BRD_DRV_DEFAULT_DATA_RX_DIR);
  if(e_status != GD_SUCCESS) return e_status;
#else
  // If in debug mode
  brd_drv_set_mclk_dir(brd_drv_dir_out);
  brd_drv_set_bclk_dir(brd_drv_dir_out);
  brd_drv_set_fsync_dir(brd_drv_dir_out);
  brd_drv_set_tx_data_dir(brd_drv_dir_out);
  brd_drv_set_rx_data_dir(brd_drv_dir_in);
#endif


  //==========================| Loading SSC settings |=========================
  // SSC fine settings
  // Load now, apply later (because of dependencies)
  s_brd_drv_ssc_fine_settings.e_dig_aud_mode = BRD_DRV_DEFAULT_DAI_MODE;

  s_brd_drv_ssc_fine_settings.e_FSYNC_RX_edge = SSC_EDGE_DEFAULT;
  s_brd_drv_ssc_fine_settings.e_FSYNC_TX_edge = SSC_EDGE_DEFAULT;

  s_brd_drv_ssc_fine_settings.e_BCLK_RX_edge = SSC_EDGE_DEFAULT;
  s_brd_drv_ssc_fine_settings.e_BCLK_TX_edge = SSC_EDGE_DEFAULT;

  ///\todo COMPLETE IMPLEMENTATION. NOT FINISHED YET
  s_brd_drv_ssc_fine_settings.i_FSYNC_pulse =
      s_brd_drv_user_settings.i_FSYNC_pulse;

  s_brd_drv_ssc_fine_settings.i_word_bit_offset = 256;

  s_brd_drv_ssc_fine_settings.i_data_length = BRD_DRV_DEFAULT_DATA_WORD_LENGTH;

  s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling =
      BRD_DRV_DEFAULT_MCLK_OVERSAMPLING;

  s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling =
      BRD_DRV_DEFAULT_BCLK_OVERSAMPLING;

  s_brd_drv_ssc_fine_settings.i_MCLK_ppm_offset = 0 ;
  i_auto_tune_pll = BRD_DRV_DEFAULT_AUTO_TUNE;

  /* FSYNC frequency is kind a special. It can not be zero (can not calculate
   * MCLK). So if FSYNC frequency value is zero, it means we are initializing
   * HW and has not been set yet. In that case we have to define SOME default
   * value.
   * If FSYNC is already defined, we just use it.
   */
  if(s_brd_drv_ssc_fine_settings.i_FSYNC_freq == 0)
  {
    s_brd_drv_ssc_fine_settings.i_FSYNC_freq = BRD_DRV_DEFAULT_FSYNC_FREQ;
  }

  e_status = brd_drv_set_data_length(
                  s_brd_drv_ssc_fine_settings.i_data_length);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! Set data length failed ! ");
    return e_status;
  }

  e_status = brd_drv_set_FSYNC_freq(s_brd_drv_ssc_fine_settings.i_FSYNC_freq);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! FSYNC failed ! ");
    return e_status;
  }

  e_status = brd_drv_set_digital_audio_interface_mode(
      s_brd_drv_ssc_fine_settings.e_dig_aud_mode);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! Set dig. aud. itfce failed ! ");
    return e_status;
  }

  e_status = brd_drv_set_BCLK_RX_edge(
      s_brd_drv_ssc_fine_settings.e_FSYNC_RX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! BCLK RX edge fail ! ");
    return e_status;
  }

  e_status = brd_drv_set_BCLK_TX_edge(
      s_brd_drv_ssc_fine_settings.e_FSYNC_TX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! BCLK TX edge fail ! ");
    return e_status;
  }

  e_status = brd_drv_set_FSYNC_RX_edge(
      s_brd_drv_ssc_fine_settings.e_FSYNC_RX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! RX FSYNC dir fail ! ");
    return e_status;
  }

  e_status = brd_drv_set_FSYNC_TX_edge(
      s_brd_drv_ssc_fine_settings.e_FSYNC_TX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! FSYNC TX edge fail ! ");
    return e_status;
  }

  e_status = brd_drv_set_word_offset(
      s_brd_drv_ssc_fine_settings.i_word_bit_offset);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! WORD bit OFFSET fail ! ");
    return e_status;
  }

  // Auto tune PLL
  e_status = brd_drv_set_auto_tune(i_auto_tune_pll);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! Auto tune PLL fail ! ");
    return e_status;
  }

  brd_drv_send_msg(BRD_DRV_MSG_INFO_FACTRY_STTNGS_LOADED,1,0,-1);
  // Return last code anyway
  return e_status;
}

//===========================| Mid level functions |===========================
/**
 * @brief Set data word length
 * @param i_data_length Data length in bits (one channel)
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_set_data_length(uint8_t i_data_length)
{
  // Status code
  GD_RES_CODE e_status;

  /**\note Because whole system is Big endian (thanks :( ), samples must be
   * shifted right way. But this can not be done on SSC layer, so it is up to
   * higher layer (USB driver) to shift samples correctly.
   */
  // Check if data length in not bigger than 1/2 BCLK oversampling value
  if(i_data_length > (s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling>>1))
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_DATA_LEN_LONG_BCLK_LOW, 1,1);
    return GD_INCORRECT_PARAMETER;
  }


  // Set data length on SSC module
  e_status = ssc_set_data_length(i_data_length);
  if(e_status != GD_SUCCESS) return e_status;

  /* Set data length on TLV codec, but codec
   * support only 16, 20, 24 and 32. Not 8, 9, 12 and so on...
   */
  if((i_data_length != 16) && (i_data_length != 20) && (i_data_length != 24) &&
     (i_data_length != 32))
  {
    // Unsupported value
    if(i_data_length >= 32){
      e_status = tlv320aic33_set_word_length(32);}
    else if(i_data_length >= 24){
      e_status = tlv320aic33_set_word_length(24);}
    else if(i_data_length >= 20){
      e_status = tlv320aic33_set_word_length(20);}
    else{
      e_status = tlv320aic33_set_word_length(16);}

    if(s_brd_drv_ssc_fine_settings.e_dig_aud_mode == SSC_DSP)
    {
      brd_drv_send_warning_msg(BRD_DRV_MSG_WRN_CODEC_NOT_SUPP_WRLD_LEN_DSP,
                               1,1);
    }
  }
  else  // Supported value
  {
    e_status = tlv320aic33_set_word_length(i_data_length);
  }
  // Check status code
  if(e_status != GD_SUCCESS) return e_status;

  // If all OK, then save value
  s_brd_drv_ssc_fine_settings.i_data_length = i_data_length;

  // If RJF -> recalculate word offset
  if(s_brd_drv_ssc_fine_settings.e_dig_aud_mode == SSC_RIGHT_JUSTIFIED)
  {
    e_status = brd_drv_set_word_offset(
                  s_brd_drv_ssc_fine_settings.i_word_bit_offset);
    if(e_status != GD_SUCCESS) return e_status;
  }

  // Anyway, return status
  return e_status;
}


/**
 * @brief Give information about actual settings of data word length
 * @param p_i_data_length Pointer to memory, where result will be written
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_get_data_length(uint8_t *p_i_data_length)
{
  *p_i_data_length = s_brd_drv_ssc_fine_settings.i_data_length;
  return GD_SUCCESS;
}



/**
 * @brief Set on which FSYNC edge will SSC RX unit begin
 * @param e_edge SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_set_FSYNC_RX_edge(e_ssc_edge_t e_edge)
{
  // Variable to store digital interface mode
  e_ssc_digital_audio_interface_t e_mode;

  // Check input parameter
  if((e_edge != SSC_EDGE_DEFAULT) &&
     (e_edge != SSC_EDGE_FALLING) &&
     (e_edge != SSC_EDGE_RISING))
  {
    return GD_INCORRECT_PARAMETER;
  }


  // Because edge is mode dependent (I2S, DSP, ....) we must get current mode
  e_mode = s_brd_drv_ssc_fine_settings.e_dig_aud_mode;

  // OK, so we got mode.
  // OK, so we got mode.
  if(e_mode == SSC_I2S)
  {
    // If rising  edge -> switch channels
    if(e_edge == SSC_EDGE_RISING)
    {
      uac1_device_audio_set_swap_LR_RX(1);
    }
    else // Falling or default ; Do not switch channel
    {
      uac1_device_audio_set_swap_LR_RX(0);
    }
  }
  else if((e_mode == SSC_LEFT_JUSTIFIED) ||
          (e_mode == SSC_RIGHT_JUSTIFIED))
  {
    // If falling edge -> switch channels
    if(e_edge == SSC_EDGE_FALLING)
    {
      uac1_device_audio_set_swap_LR_RX(1);
    }
    else // Rising or default ; Do not switch channel
    {
      uac1_device_audio_set_swap_LR_RX(0);
    }
  }
  else if(e_mode == SSC_DSP)
  {
    /* In DSP does not matter on FSYNC edge. All times first is left channel
     * then right.
     */
  }
  else // if e_mode == ....
  {
    // Unknown mode -> return fail
    return GD_FAIL;
  }


  // OK, save actual value
  s_brd_drv_ssc_fine_settings.e_FSYNC_RX_edge = e_edge;

  // Return last status
  return GD_SUCCESS;
}



/**
 * @brief Set on which FSYNC edge will SSC TX unit begin
 * @param e_edge SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_set_FSYNC_TX_edge(e_ssc_edge_t e_edge)
{
  // Store status
  GD_RES_CODE e_status;

  // Variable to store digital interface mode
  e_ssc_digital_audio_interface_t e_mode;

  // Check input parameter
  if((e_edge != SSC_EDGE_DEFAULT) &&
     (e_edge != SSC_EDGE_FALLING) &&
     (e_edge != SSC_EDGE_RISING))
  {
    return GD_INCORRECT_PARAMETER;
  }


  // Because edge is mode dependent (I2S, DSP, ....) we must get current mode
  e_status = ssc_get_digital_interface_mode(&e_mode);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // OK, so we got mode.
  if(e_mode == SSC_I2S)
  {
    // If rising  edge -> switch channels
    if(e_edge == SSC_EDGE_RISING)
    {
      uac1_device_audio_set_swap_LR_TX(1);
    }
    else // Falling or default ; Do not switch channel
    {
      uac1_device_audio_set_swap_LR_TX(0);
    }
  }
  else if((e_mode == SSC_DSP) ||
          (e_mode == SSC_LEFT_JUSTIFIED) ||
          (e_mode == SSC_RIGHT_JUSTIFIED))
  {
    // If falling edge -> switch channels
    if(e_edge == SSC_EDGE_FALLING)
    {
      uac1_device_audio_set_swap_LR_TX(1);
    }
    else // Rising or default ; Do not switch channel
    {
      uac1_device_audio_set_swap_LR_TX(0);
    }
  }
  else // if e_mode == ....
  {
    // Unknown mode -> return fail
    return GD_FAIL;
  }

  // OK, save actual value
  s_brd_drv_ssc_fine_settings.e_FSYNC_TX_edge = e_edge;

  // Return last status
  return e_status;
}





/**
 * @brief Set BCLK sampling edge for RX module
 * @param e_edge Options: SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE brd_drv_set_BCLK_RX_edge(e_ssc_edge_t e_edge)
{
  // Store status
  GD_RES_CODE e_status;

  e_status = ssc_set_BCLK_RX_edge(e_edge);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // If all OK save actual value
  s_brd_drv_ssc_fine_settings.e_BCLK_RX_edge = e_edge;

  return e_status;
}





/**
 * @ Set BCLK data transmitting edge for TX module
 * @param e_edge Options: SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE brd_drv_set_BCLK_TX_edge(e_ssc_edge_t e_edge)
{
    // Store status
  GD_RES_CODE e_status;

  e_status = ssc_set_BCLK_TX_edge(e_edge);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // If all OK save actual value
  s_brd_drv_ssc_fine_settings.e_BCLK_TX_edge = e_edge;

  return e_status;
}


/**
 * @brief Set word offset
 *
 * Word offset is delay between FSYNC pulse/signal and TX/RX_DATA signal.\n
 * This delay is in BCLK cycles.\n
 * For example I2S have this delay set to 1, DSP also, but right and left\n
 * justify have this delay 0.\n
 * Anyway it is very useful for debugging.
 *
 * @param i_word_offset Word offset between FSYNC and TX/RX_DATA in BCLK\n
 *                      cycles. Because of codec restriction safe values are\n
 *                      from 0 to 17.
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_word_offset(uint16_t i_word_offset)
{
  GD_RES_CODE e_status;

  // Value i_word_offset can be changed. This is backup
  uint16_t i_orig_word_offset = i_word_offset;

  // Check if we want to set default offset.
  if(i_word_offset == 256)
  {
    if(s_brd_drv_ssc_fine_settings.e_dig_aud_mode == SSC_I2S)
    {
      // Set offset to 1
      i_word_offset = 1;
    }
    else if((s_brd_drv_ssc_fine_settings.e_dig_aud_mode == SSC_LEFT_JUSTIFIED)
            ||
            (s_brd_drv_ssc_fine_settings.e_dig_aud_mode == SSC_DSP))
    {
      // Set offset to 0
      i_word_offset = 0;
    }
    else if(s_brd_drv_ssc_fine_settings.e_dig_aud_mode == SSC_RIGHT_JUSTIFIED)
    {
      /* Set offset by counting (Frame size) - (word size)
       * (Frame size) *2 = BCLK_ovesampling
       * -> (BCLK oversampling)/2 - (word size)
       *
       * Anyway max word offset can be set to 255, but BCLK can work at much
       * higher frequency. So that is reason why we should check also this case
       * and if necessary show warning.
       */
      if(s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling >= 512)
      {
        // Problem. BLCK is higher than 511, so we can not set proper offset.
        brd_drv_send_error_msg(BRD_DRV_MSG_ERR_CANT_SET_WORD_OFF_BCLK_HIGH,1,1);
        return GD_FAIL;
      }// BCLK >= 512
      else
      {
        // All OK, should not be problem here
        i_word_offset = (s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling>>1) -
                         s_brd_drv_ssc_fine_settings.i_data_length;
      }
    }
    else
    {
      // Unknown mode - Should not happen
      return GD_FAIL;
    }
  }// offset == 256
  if(i_word_offset >= 257)
  {
    // Unknown number -> error
    return GD_INCORRECT_PARAMETER;
  }

  /* Also check if (offset + data) word is not higher than BCLK/2.
   * In that case overlap occurs. But it is not reason for changing BCLK,
   * because only LSB of data word will be cut. So we just show warning
   */
  if((i_word_offset + s_brd_drv_ssc_fine_settings.i_data_length) >
     (s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling>>1))
  {
    // LSB will be cut -> show at least warning
    brd_drv_send_warning_msg(
        BRD_DRV_MSG_WRN_OFF_PLUS_DATA_HIGHR_THAN_HALF_BCLK, 1, 1);
  }

  // Also codec have some limits
  if(i_word_offset > 16)
  {
    brd_drv_send_warning_msg(
        BRD_DRV_MSG_WRN_CODEC_NOT_SUPPORT_HIGH_WORD_OFFSET, 1,1);
  }

  // Just set offset
  // Set this offset in SSC driver
  e_status = ssc_set_word_offset((uint8_t)i_word_offset);
  if(e_status != GD_SUCCESS) return e_status;

  // Because instead of using RJF or I2S on codec we use LFJ, no problem here
  e_status = tlv320aic33_set_data_offset((uint8_t)i_word_offset);
  if(e_status != GD_SUCCESS) return e_status;


  // If success, write it to our virtual register
  if(e_status == GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.i_word_bit_offset = i_orig_word_offset;
  }

  // Just for sending debug information to UART
  char c[30];
  tfp_sprintf(&c[0],"word_offset set to: %d\n", i_word_offset);
  brd_drv_send_msg(&c[0], 1,0,-1);

  return e_status;
}


/**
 * \brief Give word offset
 *
 * This delay is in BCLK cycles.\n
 * For example I2S have this delay set to 1, DSP also, but right and left\n
 * justify have this delay 0.\n
 * Anyway it is very useful for debugging.
 *
 * @param p_i_word_offset Address, where result will be written.
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE brd_drv_get_word_offset(uint16_t *p_i_word_offset)
{
  *p_i_word_offset = s_brd_drv_ssc_fine_settings.i_word_bit_offset;
  return GD_SUCCESS;
}


/**
 * @brief Set MCLK PPM offset
 *
 * MCLK PPM offset is value which says how much is MCLK frequency different\n
 * than original value.
 * @param i_MCLK_ppm_offset MCLK offset in PPM.\n
 *                          Frequency offset in PPM (1/1e6 th of frequency)
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_MCLK_ppm(int32_t i_MCLK_ppm_offset)
{
  GD_RES_CODE e_status;

  // Original frequency (FSYNC * MCLK_oversampling)
  uint32_t i_orig_freq;

  /* New frequency (with offset). Need to use float, because comparative to
   * frequency, PPM is very small unit and we need to be very precise.
   * But at the end it is better operate with integer -> so we have 2 variables
   */
  float f_new_freq;
  uint32_t i_new_freq;

  // For calculating real offset (need high precision)
  float f_real_off;

  /* Memory space for message. */
  char msg[80];

  // Keep information about MCLK divider value
  uint16_t i_MCLK_div;

  /* Although does not matter if auto tune option is enabled or not, for user
   * is better to said "Turn off auto tune" and when call this function user
   * can be sure, that this offset will not change (even if feedback EP does
   * not work)
   */
  if(i_auto_tune_pll)
  {
    brd_drv_send_error_msg(
        BRD_DRV_MSG_ERR_AUTO_TUNE_ENABLED_MCLK_PPM_OFF_NOT_ALLOW, 1,1);
    return GD_FAIL;
  }
  //============================| Parameters checked |=========================

  // Calculate original frequency
  i_orig_freq = s_brd_drv_ssc_fine_settings.i_FSYNC_freq *
                (uint32_t)s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling;

  // Calculate new frequency
  f_new_freq = (float)i_orig_freq +

               ((float)i_orig_freq * (float)i_MCLK_ppm_offset)/
               (float)(1000000UL);
  i_new_freq = (uint32_t)f_new_freq;

  // Get divider for MCLK
  e_status = brd_drv_get_MCLK_div(&i_MCLK_div);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // We need to show user MCLK value, not real value at PLL
  tfp_sprintf(&msg[0],
              "MCLK orig: %lu Hz | Required (with offset): %lu Hz\n",
              i_orig_freq, i_new_freq);
  brd_drv_send_msg(&msg[0],1,0,-1);


  // Check if frequency for PLL is not too high. Now integer values are OK
  // Calculate real frequency for PLL
  i_new_freq = i_new_freq * i_MCLK_div;
  // Just for case check if frequency is in limits
  if(i_new_freq > CS2200_MAX_FREQ)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_CANT_SET_PLL_FREQ_TOO_HIGH, 1,1);
    return GD_INCORRECT_PARAMETER;
  }
  while(i_new_freq < CS2200_MIN_FREQ)
  {
    // We can bypass this. Just double frequency at PLL and double MCLK div
    i_new_freq = i_new_freq<<1;
    i_MCLK_div = i_MCLK_div<<1;
    e_status = brd_drv_set_MCLK_div(i_MCLK_div);
    // Never should fail, but you never know
    if(e_status != GD_SUCCESS) return e_status;
  }

  /* Set PLL. But for some reason frequency can be too high. So in that case
   * setting fail and we show error.
   */
  e_status = cs2200_set_PLL_freq(i_new_freq);
  if(e_status != GD_SUCCESS)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_CANT_SET_PLL_FREQ_TOO_HIGH, 1,1);
    return GD_FAIL;
  }

  // Get real frequency from PLL and recalculate offset
  e_status = cs2200_get_PLL_freq(&i_new_freq);
  if(e_status != GD_SUCCESS) return GD_FAIL;
  // Divide PLL frequency by MCLK divider value -> get real MCLK frequency
  f_real_off = ((float)(1000000UL) *
               (((float)i_new_freq /(float)i_MCLK_div) - (float)i_orig_freq))/
               (float)i_orig_freq;
  /* Check if is negative or positive and do round. thanks to this we also get
   * most precise result for PLL
   */
  if(f_real_off > 0)
  {
    f_real_off = f_real_off + 0.5;
  }
  else
  {
    f_real_off = f_real_off -0.5;
  }

  // Now save value as integer
  s_brd_drv_ssc_fine_settings.i_MCLK_ppm_offset = i_MCLK_ppm_offset;

  //sprintf(&msg[0], "MCLK real offset: %4.2f PPM\n", f_real_off);
  char c_float_val[20];
  floatToStr(f_real_off, 2, &c_float_val[0]);

  tfp_sprintf(&msg[0], "MCLK real offset: %s PPM\n", c_float_val);
  brd_drv_send_msg(&msg[0],1,0,-1);

  return e_status;
}

/**
 * @brief Give MCLK PPM offset
 *
 * MCLK PPM offset is value which says how much is MCLK frequency different\n
 * than original value.
 * @param p_i_MCLK_ppm_offset Result will be written to this address.\n
 *                            Frequency offset in PPM (1/1e6 th of frequency)
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE brd_drv_get_MCLK_ppm(int32_t *p_i_MCLK_ppm_offset)
{
  *p_i_MCLK_ppm_offset = s_brd_drv_ssc_fine_settings.i_MCLK_ppm_offset;
  return GD_SUCCESS;
}


/**
 * \brief Write message to selected outputs
 *
 * Function do not return any return code. Just hope, that everything\n
 * will work.
 *
 * @param p_msg Pointer to message
 * @param i_write_to_DBG Options: 0 - do not write to debug output ;\n
 *  1 - write to debug output
 * @param i_write_to_LCD 0 - do not write to LCD ;\n
 *  1 - write to LCD
 * @param i_LCD_line Define on which line should be message placed. If value\n
 * is 255 (-1), or invalid (higher than 5) then just write message message on\n
 * the following line. (NOT SUPPORTED FOR MOMENT)
 */
inline void brd_drv_send_msg(
    const char * p_msg,
    const uint8_t i_write_to_DBG,
    const uint8_t i_write_to_LCD,
    const uint8_t i_LCD_line)
{
  // If want write to debug interface
  if(i_write_to_DBG != 0)
  {
    print_dbg("Info > ");
    print_dbg(p_msg);
  }

  // If want write to LCD
  if(i_write_to_LCD != 0)
  {
    disp_show_info(p_msg);
  }
}


/**
 * \brief Write warning message
 *
 * @param p_msg Pointer to message
 * @param i_write_to_DBG Options: 0 - do not write to debug output ;\n
 *  1 - write to debug output
 * @param i_write_to_LCD 0 - do not write to LCD ;\n
 *  1 - write to LCD
 */
inline void brd_drv_send_warning_msg(
    const char * p_msg,
    const uint8_t i_write_to_DBG,
    const uint8_t i_write_to_LCD)
{
  if(i_write_to_DBG != 0)
  {
    print_dbg("Warn > ");
    print_dbg(p_msg);
  }
  if(i_write_to_LCD != 0)
  {
    disp_show_warn(p_msg);
  }
}


/**
 * \brief Write error message and turn on ERROR LED
 *
 * Function do not return any return code. Just hope, that everything else\n
 * will work.
 *
 * @param p_msg Pointer to message
 * @param i_write_to_DBG Options: 0 - do not write to debug output ;\n
 *  1 - write to debug output
 * @param i_write_to_LCD 0 - do not write to LCD ;\n
 *  1 - write to LCD
 */
inline void brd_drv_send_error_msg(
    const char * p_msg,
    const uint8_t i_write_to_DBG,
    const uint8_t i_write_to_LCD)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;
  // Turn-ON ERROR LED
  BRD_DRV_IO_HIGH(BRD_DRV_ERROR_SIG_PIN);

  // If want write to debug interface
  if(i_write_to_DBG != 0)
  {
    print_dbg("Error > ");
    print_dbg(p_msg);
  }

  // If want write to LCD
  if(i_write_to_LCD != 0)
  {
    disp_show_err(p_msg);
  }
}

//===========================| Low level functions |===========================
/**
 * \brief Enable or disable auto tune feature
 * @param i_enable Enable (1) or disable (0) feature
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_set_auto_tune(uint8_t i_enable)
{
  GD_RES_CODE e_status;

  if(i_enable == 0)
  {
    uac1_device_audio_set_auto_tune(0);
    i_auto_tune_pll = 0;
    // And set PLL frequency with offset
    e_status = brd_drv_set_MCLK_ppm(
                  s_brd_drv_ssc_fine_settings.i_MCLK_ppm_offset);
  }
  else
  {
    uac1_device_audio_set_auto_tune(1);
    i_auto_tune_pll = 1;
    /* Reset offset to zero -> just call set MCLK oversampling freq. function.
     * This function set MCLK frequency back (MCLK PPM offset = 0)
     */
    e_status = brd_drv_set_MCLK_oversampling(
                  s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling);
  }

  return e_status;
}

/**
 * @brief Get status of auto tune feature
 *
 * @param p_i_enable Pointer to memory, where result will be written.\n
 *                   Enable (1) or disable (0)
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_get_auto_tune(uint8_t *p_i_enable)
{
  *p_i_enable = i_auto_tune_pll;
  return GD_SUCCESS;
}

/**
 * \brief Set pins, that control signal direction at digital isolators, to low
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_set_isolators_to_HiZ(void)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  //================================| Pure I2S |===============================
  // Set output enables to low -> disable them by default
  brd_drv_set_mclk_dir(brd_drv_dir_hiz);
  brd_drv_set_bclk_dir(brd_drv_dir_hiz);
  brd_drv_set_fsync_dir(brd_drv_dir_hiz);
  brd_drv_set_tx_data_dir(brd_drv_dir_hiz);
  brd_drv_set_rx_data_dir(brd_drv_dir_hiz);
  //=============================| Reset and mute |============================
  brd_drv_set_mute_dir(brd_drv_dir_hiz);
  brd_drv_set_rst_dai_dir(brd_drv_dir_hiz);

  // Button pins - it is set as input after reset but just for sure
  BRD_DRV_IO_AS_INPUT(BRD_DRV_MUTE_BTN_PIN);
  BRD_DRV_IO_AS_INPUT(BRD_DRV_RESET_I2S_BTN_PIN);

  // Signal pins - it is set as input after reset but just for sure
  BRD_DRV_IO_AS_INPUT(BRD_DRV_MUTE_PIN);
  BRD_DRV_IO_AS_INPUT(BRD_DRV_RESET_I2S_PIN);


  return GD_SUCCESS;
}



/**
 * @brief Set all isolators according to given settings
 * @param s_brd_drv_pure_dai_dir Structure of signals with direction
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_set_isolators(
    s_brd_drv_pure_dai_dir_t s_pure_dai_dir)
{
  // Keep status code
  GD_RES_CODE e_status;

  // Set as much as possible and return error code as OR function
  e_status  = brd_drv_set_mclk_dir(   s_pure_dai_dir.e_mclk_dir);
  e_status |= brd_drv_set_bclk_dir(   s_pure_dai_dir.e_bclk_dir);
  e_status |= brd_drv_set_fsync_dir(  s_pure_dai_dir.e_fsync_dir);
  e_status |= brd_drv_set_tx_data_dir(s_pure_dai_dir.e_tx_data_dir);
  e_status |= brd_drv_set_rx_data_dir(s_pure_dai_dir.e_rx_data_dir);

  return e_status;
}



/**
 * \brief Allow set MUTE pin direction
 * @param e_mute_dir Options: 0 (input) ; 1 (output) ; 2 (Hi-Z)
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_mute_dir(e_brd_drv_dir_t e_mute_dir)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  if(e_mute_dir == brd_drv_dir_in)
  {// IN
    // Clear MUTE_EN_B - disable TX mute
    BRD_DRV_IO_LOW(BRD_DRV_MUTE_EN_B_PIN);
    // MUTE as input
    BRD_DRV_IO_AS_INPUT(BRD_DRV_MUTE_PIN);
    // Set MUTE_EN_A - enable RX mute data
    BRD_DRV_IO_HIGH(BRD_DRV_MUTE_EN_A_PIN);

    s_brd_drv_mute.e_mute_dir = brd_drv_dir_in;
  }
  else if(e_mute_dir == brd_drv_dir_out)
  {// OUT
    // Clear MUTE_EN_A - disable RX mute
    BRD_DRV_IO_LOW(BRD_DRV_MUTE_EN_A_PIN);
    // MUTE as output
    BRD_DRV_IO_LOW(BRD_DRV_MUTE_PIN);
    // Set MUTE_EN_B - enable TX mute
    BRD_DRV_IO_HIGH(BRD_DRV_MUTE_EN_B_PIN);

    s_brd_drv_mute.e_mute_dir = brd_drv_dir_out;
  }
  else if(e_mute_dir == brd_drv_dir_hiz)
  {
    // Clear MUTE_EN_B - disable TX mute
    BRD_DRV_IO_LOW(BRD_DRV_MUTE_EN_B_PIN);
    // MUTE as input
    BRD_DRV_IO_AS_INPUT(BRD_DRV_MUTE_PIN);
    // Clear MUTE_EN_A - enable RX mute data
    BRD_DRV_IO_LOW(BRD_DRV_MUTE_EN_A_PIN);

    s_brd_drv_mute.e_mute_dir = brd_drv_dir_hiz;
  }
  else
  {// Unknown
    return GD_INCORRECT_PARAMETER;
  }

  // Default value is 0
  s_brd_drv_mute.i_mute_val = 0;

  return GD_SUCCESS;
}



/**
 * \brief Set mute flag
 *
 * If signal MUTE is input, then just MUTE MCU output. If MUTE signal is\n
 * output then set MUTE signal and MUTE MCU output.
 *
 * @param i_mute_flag 0 - mute off ; 1 - mute on
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_set_mute(uint8_t i_mute_flag)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  if((s_brd_drv_mute.e_mute_dir == brd_drv_dir_in) ||
     (s_brd_drv_mute.e_mute_dir == brd_drv_dir_hiz))
  {
    // Input direction or Hi-Z
    if(i_mute_flag == 0)
    {
      // Mute off
      gpio_enable_module_pin(SSC_TX_DATA,SSC_TX_DATA_FUNCTION);
      // Send message
      brd_drv_send_msg(BRD_DRV_MSG_INFO_MUTE_IN_MUTE_OFF, 1, 0, -1);
    }
    else
    {
      // Mute on - set pin to low
      BRD_DRV_IO_LOW(SSC_TX_DATA);
      // Send message
      brd_drv_send_msg(BRD_DRV_MSG_INFO_MUTE_IN_MUTE_ON, 1, 0, -1);
    }
  }
  else if(s_brd_drv_mute.e_mute_dir == brd_drv_dir_out)
  {
    // Output direction
    if(i_mute_flag == 0)
    {
      // Mute off
      gpio_enable_module_pin(SSC_TX_DATA,SSC_TX_DATA_FUNCTION);
      // Set signal
      BRD_DRV_IO_LOW(BRD_DRV_MUTE_PIN);
      // Send message
      brd_drv_send_msg(BRD_DRV_MSG_INFO_MUTE_OUT_MUTE_OFF, 1, 0, -1);
    }
    else
    {
      // Mute on - set pin to low
      BRD_DRV_IO_LOW(SSC_TX_DATA);
      // Set signal
      BRD_DRV_IO_HIGH(BRD_DRV_MUTE_PIN);
      // Send message
      brd_drv_send_msg(BRD_DRV_MSG_INFO_MUTE_OUT_MUTE_ON, 1, 0, -1);
    }
  }
  else
  {// Unknown direction
    return GD_INCORRECT_PARAMETER;
  }

  // Save value
  s_brd_drv_mute.i_mute_val = i_mute_flag;

  return GD_SUCCESS;
}



/**
 * \brief Allow set RESET_I2S pin direction
 * @param e_rst_i2s_dir Options: 0 (input) ; 1 (output) ; 2 (Hi-Z)
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_rst_dai_dir(e_brd_drv_dir_t e_rst_i2s_dir)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  if(e_rst_i2s_dir == brd_drv_dir_in)
  {// IN
    // Clear RST_EN_B - disable TX mute
    BRD_DRV_IO_LOW(BRD_DRV_RST_EN_B_PIN);
    // RESET_I2S as input
    BRD_DRV_IO_AS_INPUT(BRD_DRV_RESET_I2S_PIN);
    // Set RST_EN_A - enable RX mute data
    BRD_DRV_IO_HIGH(BRD_DRV_RST_EN_A_PIN);

    s_brd_drv_rst_i2s.e_rst_dai_dir = brd_drv_dir_in;
  }
  else if(e_rst_i2s_dir == brd_drv_dir_out)
  {// OUT
    // Clear RST_EN_A - disable RX mute
    BRD_DRV_IO_LOW(BRD_DRV_RST_EN_A_PIN);
    // RESET_I2S as output
    BRD_DRV_IO_LOW(BRD_DRV_RESET_I2S_PIN);
    // Set RST_EN_B - enable TX mute
    BRD_DRV_IO_HIGH(BRD_DRV_RST_EN_B_PIN);

    s_brd_drv_rst_i2s.e_rst_dai_dir = brd_drv_dir_out;
  }
  else if(e_rst_i2s_dir == brd_drv_dir_hiz)
  {// Hi-Z
    // Clear RST_EN_B - disable TX mute
    BRD_DRV_IO_LOW(BRD_DRV_RST_EN_B_PIN);
    // RESET_I2S as input
    BRD_DRV_IO_AS_INPUT(BRD_DRV_RESET_I2S_PIN);
    // Clear RST_EN_A - enable RX mute data
    BRD_DRV_IO_LOW(BRD_DRV_RST_EN_A_PIN);

    s_brd_drv_rst_i2s.e_rst_dai_dir = brd_drv_dir_hiz;
  }
  else
  {// Undefined
    return GD_INCORRECT_PARAMETER;
  }

  // Dfault value is 0
  s_brd_drv_rst_i2s.i_rst_dai_val = 0;
  return GD_SUCCESS;
}



/**
 * \brief Set RESET_I2S pin if configured as output.
 *
 * If signal RESET_I2S is set as output and flag is set, then send RESET_I2S\n
 * pulse and perform reset I2S bus on AVR side too.\n
 * If signal RESET_I2S is set as input, then reset I2S bus on AVR side.
 *
 * @param i_reset_i2s_flag 0 - disable ; 1 - enable reset
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_rst_dai(uint8_t i_reset_i2s_flag)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Check direction
  if((s_brd_drv_rst_i2s.e_rst_dai_dir == brd_drv_dir_in) ||
     (s_brd_drv_rst_i2s.e_rst_dai_dir == brd_drv_dir_hiz))
  {
    // Input or Hi-Z
    // Perform some activity only if set to 1
    if(i_reset_i2s_flag != 0)
    {
      brd_drv_send_msg(BRD_DRV_MSG_INFO_RESET_I2S_INPUT_ON, 1, 0, -1);
      s_brd_drv_rst_i2s.i_rst_dai_val = 1;
      // Perform operations
      brd_drv_reset_i2s();
      // Operation done
      s_brd_drv_rst_i2s.i_rst_dai_val = 0;
    }
    else
    {
      // Set to 0 - turn off
      s_brd_drv_rst_i2s.i_rst_dai_val = 0;
      brd_drv_send_msg(BRD_DRV_MSG_INFO_RESET_I2S_INPUT_OFF, 1, 0, -1);
    }
  }
  else if(s_brd_drv_rst_i2s.e_rst_dai_dir == brd_drv_dir_out)
  {
    // Output
    // Just set RESET_I2S pin
    if(i_reset_i2s_flag == 0)
    {
      // Set RESET I2S signal to 0 - not needed reset I2S connector on AVR side
      BRD_DRV_IO_LOW(BRD_DRV_RESET_I2S_PIN);
      s_brd_drv_rst_i2s.i_rst_dai_val = 0;
      brd_drv_send_msg(BRD_DRV_MSG_INFO_RESET_I2S_SET_TO_LOW, 1, 0, -1);
    }
    else
    {
      // Reset flag = 1

      // Store status value
      GD_RES_CODE e_status;

      // Set RESET I2S signal to 1
      BRD_DRV_IO_HIGH(BRD_DRV_RESET_I2S_PIN);
      s_brd_drv_rst_i2s.i_rst_dai_val = 1;

      // Keep RESET_I2S in HIGH little bit longer
      volatile uint32_t i_cnt = 0;
      while(i_cnt < 5000UL)
      {
        i_cnt++;
      }

      // Do reset I2S bus on AVR side
      e_status = brd_drv_reset_i2s();
      brd_drv_send_msg(BRD_DRV_MSG_INFO_RESET_I2S_SET_TO_HIGH, 1, 0, 0);

      // Anyway set RESET I2S to low
      /* Note: Already done by brd_drv_reset_i2s(), so it is commented, but
       * developer saw this code and will know what happens
       */
      //BRD_DRV_IO_LOW(BRD_DRV_RESET_I2S_PIN);
      s_brd_drv_rst_i2s.i_rst_dai_val = 0;

      // Check if reset function was done without errors
      if(e_status != GD_SUCCESS)
      {
        return e_status;
      }
    }
  }
  else
  {// Unknown direction
    return GD_INCORRECT_PARAMETER;
  }

  return GD_SUCCESS;
}



/**
 * \brief Set TX_DATA direction (OUT/Hi-Z)
 *
 * Long story short: allow set as output or Hi-Z.
 *
 * @param e_tx_data_dir Options: 1 (output) ; 2 (Hi-Z)
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_set_tx_data_dir(e_brd_drv_dir_t e_tx_data_dir)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Check direction
  if(e_tx_data_dir == brd_drv_dir_out)
  {
    BRD_DRV_IO_HIGH(BRD_DRV_TX_EN_B_PIN);
  }
  else if(e_tx_data_dir == brd_drv_dir_hiz)
  {
    BRD_DRV_IO_LOW(BRD_DRV_TX_EN_B_PIN);
  }
  else
  {// Unsupported option
    return GD_INCORRECT_PARAMETER;
  }

  // If parameter correct, then save it
  s_brd_drv_pure_dai_dir.e_tx_data_dir = e_tx_data_dir;

  return GD_SUCCESS;
}



/**
 * \brief Set RX_DATA direction (IN/Hi-Z)
 * @param e_rx_data_dir Options: 0 (input) ; 2 (Hi-Z)
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_set_rx_data_dir(e_brd_drv_dir_t e_rx_data_dir)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Check direction
  if(e_rx_data_dir == brd_drv_dir_in)
  {
    BRD_DRV_IO_HIGH(BRD_DRV_RX_EN_B_PIN);
  }
  else if(e_rx_data_dir == brd_drv_dir_hiz)
  {
    BRD_DRV_IO_LOW(BRD_DRV_RX_EN_B_PIN);
  }
  else
  {
    return GD_INCORRECT_PARAMETER;
  }

  // If parameter correct, then save it
  s_brd_drv_pure_dai_dir.e_rx_data_dir = e_rx_data_dir;

  return GD_SUCCESS;
}


/**
 * \brief Set direction of MCLK
 * @param e_mclk_dir Options: 0 (input) ; 1 (output) ; 2 (Hi-Z)
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_mclk_dir(e_brd_drv_dir_t e_mclk_dir)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;
  switch(e_mclk_dir)
  {
  case brd_drv_dir_in:
    BRD_DRV_IO_LOW(BRD_DRV_MCLK_EN_B_PIN);
    BRD_DRV_IO_AS_INPUT(MCLK_PIN);
    BRD_DRV_IO_HIGH(BRD_DRV_MCLK_EN_A_PIN);
    break;
  case brd_drv_dir_out:
    BRD_DRV_IO_LOW(BRD_DRV_MCLK_EN_A_PIN);
    gpio_enable_module_pin(MCLK_PIN, MCLK_FUNCTION);
    BRD_DRV_IO_HIGH(BRD_DRV_MCLK_EN_B_PIN);
    break;
  case brd_drv_dir_hiz:
    /* Not easy to define "right" option. So disconnect signals from connector
     * side, but allow transmit signals, so codec will work
     */
    BRD_DRV_IO_LOW(BRD_DRV_MCLK_EN_A_PIN);
    BRD_DRV_IO_LOW(BRD_DRV_MCLK_EN_B_PIN);
    gpio_enable_module_pin(MCLK_PIN, MCLK_FUNCTION);
    break;
  default:
    return GD_INCORRECT_PARAMETER;
  }

  // If all OK, save state
  s_brd_drv_pure_dai_dir.e_mclk_dir = e_mclk_dir;

  return GD_SUCCESS;
}


/**
 * \brief Set BCLK direction
 * @param e_bclk_dir Options: 0 (input) ; 1 (output) ; 2 (Hi-Z)
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_bclk_dir(e_brd_drv_dir_t e_bclk_dir)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Following variables are used only when want to switch to input
  int pin_state = 0;
#if BRD_DRV_SUPPORT_RTOS != 0
  portTickType actual_time = xTaskGetTickCount();
  /* Assume that minimum input frequency is BRD_DRV_MIN_DET_FREQ Hz and
   * configTICK_RATE_HZ is higher than BRD_DRV_MIN_DET_FREQ Hz.
   */
  portTickType end_time = actual_time +
                          (configTICK_RATE_HZ/BRD_DRV_MIN_DET_FREQ);
#else
  // No RTOS used. Just use hard-coded values
  uint32_t actual_time = 0;
  uint32_t end_time = 0x1FFFFFFF;
#endif

  switch(e_bclk_dir)
  {
  case brd_drv_dir_in:
    /* If we set BCLK to input, but there are no clocks, it can cause freeze
     * of SSC module (I guess...) and in that all system. So We need to check
     * clock first. If no clock were detected, switch to FAIL safe -> Hi-Z
     * and show error message.
     */
    ssc_set_BCLK_src_int();
    // On connector side turn off output driver (connector -> world)
    BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_B_PIN);
    // Set BCLK pin as input
    BRD_DRV_IO_AS_INPUT(BCLK_PIN);
    // On MCU side turn on output driver (connector -> MCU)
    BRD_DRV_IO_HIGH(BRD_DRV_BCLK_EN_A_PIN);

    brd_drv_send_msg(BRD_DRV_MSG_INFO_BCLK_DTCT_FREQ, 1, 0, -1);


    // Wait for low level
    while(1)
    {
      pin_state = gpio_get_pin_value(BCLK_PIN);
      if(pin_state == 0)
      {
        brd_drv_send_msg(BRD_DRV_MSG_INFO_BCLK_DTCT_FREQ_L_DET, 1, 0, -1);
        break;
      }
#if BRD_DRV_SUPPORT_RTOS != 0
      actual_time = xTaskGetTickCount();
#else
      actual_time++;
#endif
      if(actual_time > end_time)
      {
        brd_drv_send_warning_msg(BRD_DRV_MSG_WRN_BCLK_DTCT_FREQ_L_NDET, 1, 1);
        // Set Hi-Z for BLCK (fail safe)
        BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_A_PIN);
        gpio_enable_module_pin(BCLK_PIN, BCLK_FUNCTION);
        BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_B_PIN);
        return GD_TIMEOUT;
      }
    }


    // Wait for high level
#if BRD_DRV_SUPPORT_RTOS != 0
      end_time = xTaskGetTickCount() + (configTICK_RATE_HZ/BRD_DRV_MIN_DET_FREQ);
#else
      actual_time = 0;
#endif
    while(1)
    {
      pin_state = gpio_get_pin_value(BCLK_PIN);
      if(pin_state != 0)
      {
        brd_drv_send_msg(BRD_DRV_MSG_INFO_BCLK_DTCT_FREQ_H_DET, 1, 0, -1);
        break;
      }
#if BRD_DRV_SUPPORT_RTOS != 0
      actual_time = xTaskGetTickCount();
#else
      actual_time++;
#endif
      if(actual_time > end_time)
      {
        brd_drv_send_warning_msg(BRD_DRV_MSG_WRN_BCLK_DTCT_FREQ_H_NDET, 1, 1);
        // Set Hi-Z for BCLK (fail safe)
        BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_A_PIN);
        gpio_enable_module_pin(BCLK_PIN, BCLK_FUNCTION);
        BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_B_PIN);
        return GD_TIMEOUT;
      }
    }
    /* Some clock detected -> switch SSC module to BCLK pin source. Now it is
     * user's problem if CLK will disappear
     */
    ssc_set_BCLK_src_ext();
    break;

  case brd_drv_dir_out:
    BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_A_PIN);
    gpio_enable_module_pin(BCLK_PIN, BCLK_FUNCTION);
    BRD_DRV_IO_HIGH(BRD_DRV_BCLK_EN_B_PIN);
    break;
  case brd_drv_dir_hiz:
    BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_A_PIN);
    gpio_enable_module_pin(BCLK_PIN, BCLK_FUNCTION);
    BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_B_PIN);
    break;
  default:
    return GD_INCORRECT_PARAMETER;
  }

  // If all OK, save state
  s_brd_drv_pure_dai_dir.e_bclk_dir = e_bclk_dir;

  return GD_SUCCESS;
}



/**
 * \brief Set direction of FSYNC
 * @param e_fsync_dir Options: 0 (input) ; 1 (output) ; 2 (Hi-Z)
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_fsync_dir(e_brd_drv_dir_t e_fsync_dir)
{
  // Store status
  GD_RES_CODE e_status;

  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Following variables are used only when want to switch to input
  int pin_state = 0;
#if BRD_DRV_SUPPORT_RTOS != 0
  portTickType actual_time = xTaskGetTickCount();
  /* Assume that minimum input frequency is BRD_DRV_MIN_DET_FREQ Hz and
   * configTICK_RATE_HZ is higher than BRD_DRV_MIN_DET_FREQ Hz.
   */
  portTickType end_time = actual_time +
                          (configTICK_RATE_HZ/BRD_DRV_MIN_DET_FREQ);
#else
  // No RTOS used. Just use hard-coded values
  uint32_t actual_time = 0;
  uint32_t end_time = 0x1FFFFFFF;
#endif

  switch(e_fsync_dir)
  {
  /* Input is tricky. If there is not CLK for some time, it may cause freezing
   * whole system (because of infinate waiting for some synchronization....).
   * So before switch we check for frequency. If no frequency is there ->
   * -> switch back before something fucks up.
   */
  case brd_drv_dir_in:
    // Turn off output driver at connector side
    BRD_DRV_IO_LOW(BRD_DRV_FS_EN_B_PIN);
    // And also turn off output driver at MCU's side
    BRD_DRV_IO_LOW(BRD_DRV_FS_EN_A_PIN);

    /* Generate few pulses
     */
    e_status = ssc_set_FSYNC_role(SSC_ROLE_TX);
    if(e_status != GD_SUCCESS) return e_status;
    /* There is special pin, which is used by another task (for syncing L and R
     * channel. Anyway, we can use it for detecting frequency as well. It is
     * used only as input.
     */
    ssc_wait_for_FSYNC_RX();
    ssc_wait_for_FSYNC_RX();
    ssc_wait_for_FSYNC_RX();

    // Just in case
    BRD_DRV_IO_AS_INPUT(SSC_EXTERNAL_FSYNC_DETECTION_PIN);
    /* Disable FSYNC pin on SSC (only TX module can generate clock, because
     * SSC set RX module as "read only".
     */
    BRD_DRV_IO_AS_INPUT(SSC_TX_FSYNC);
    // Turn on output driver at MCU side (connector -> MCU)
    BRD_DRV_IO_HIGH(BRD_DRV_FS_EN_A_PIN);

    brd_drv_send_msg(BRD_DRV_MSG_INFO_FSYNC_DTCT_FREQ, 1, 0,-1);
    // Wait for low level
    while(1)
    {
      pin_state = gpio_get_pin_value(SSC_EXTERNAL_FSYNC_DETECTION_PIN);
      if(pin_state == 0)
      {
        brd_drv_send_msg(BRD_DRV_MSG_INFO_FSYNC_DTCT_FREQ_L_DET, 1,0,-1);
        break;
      }
#if BRD_DRV_SUPPORT_RTOS != 0
      actual_time = xTaskGetTickCount();
#else
      actual_time++;
#endif
      if(actual_time > end_time)
      {
        brd_drv_send_warning_msg(BRD_DRV_MSG_WRN_FSYNC_DTCT_FREQ_L_NDET, 1,1);
        // Set Hi-Z option (fail safe)
        BRD_DRV_IO_LOW(BRD_DRV_FS_EN_A_PIN);
        BRD_DRV_IO_LOW(BRD_DRV_FS_EN_B_PIN);
        /* If Hi-Z set we want listen at least through codec. Do not care about
         * error code.
         */
        gpio_enable_module_pin(SSC_TX_FSYNC, SSC_TX_FSYNC_FUNCTION);
        ssc_set_FSYNC_role(SSC_ROLE_TX);
        return GD_TIMEOUT;
      }
    }


    // Wait for high level
#if BRD_DRV_SUPPORT_RTOS != 0
      end_time = xTaskGetTickCount() + (configTICK_RATE_HZ/BRD_DRV_MIN_DET_FREQ);
#else
      actual_time = 0;
#endif
    while(1)
    {
      pin_state = gpio_get_pin_value(SSC_EXTERNAL_FSYNC_DETECTION_PIN);
      if(pin_state != 0)
      {
        brd_drv_send_msg(BRD_DRV_MSG_INFO_FSYNC_DTCT_FREQ_H_DET, 1,0,-1);
        break;
      }
#if BRD_DRV_SUPPORT_RTOS != 0
      actual_time = xTaskGetTickCount();
#else
      actual_time++;
#endif
      if(actual_time > end_time)
      {
        brd_drv_send_warning_msg(BRD_DRV_MSG_WRN_FSYNC_DTCT_FREQ_H_NDET, 1,1);
        // Set Hi-Z option (fail safe)
        BRD_DRV_IO_LOW(BRD_DRV_FS_EN_A_PIN);
        BRD_DRV_IO_LOW(BRD_DRV_FS_EN_B_PIN);
        /* If Hi-Z set we want listen at least through codec. Do not care about
         * error code.
         */
        gpio_enable_module_pin(SSC_TX_FSYNC, SSC_TX_FSYNC_FUNCTION);
        ssc_set_FSYNC_role(SSC_ROLE_TX);
        return GD_TIMEOUT;
      }
    }

    // Allow SSC module to use FSYNC TX pin again
    gpio_enable_module_pin(SSC_TX_FSYNC, SSC_TX_FSYNC_FUNCTION);

    // And switch to
    e_status = ssc_set_FSYNC_role(SSC_ROLE_RX);
    if(e_status != GD_SUCCESS) return e_status;
    break;

  case brd_drv_dir_out:
    BRD_DRV_IO_LOW(BRD_DRV_FS_EN_A_PIN);
    e_status = ssc_set_FSYNC_role(SSC_ROLE_TX);
    if(e_status != GD_SUCCESS)
    {
      return e_status;
    }
    BRD_DRV_IO_HIGH(BRD_DRV_FS_EN_B_PIN);
    break;
  case brd_drv_dir_hiz:
    BRD_DRV_IO_LOW(BRD_DRV_FS_EN_A_PIN);
    BRD_DRV_IO_LOW(BRD_DRV_FS_EN_B_PIN);
    // If Hi-Z set we want listen at least through codec
    e_status = ssc_set_FSYNC_role(SSC_ROLE_TX);
    if(e_status != GD_SUCCESS)
    {
      return e_status;
    }
    break;
  default:
    return GD_INCORRECT_PARAMETER;
  }

  s_brd_drv_pure_dai_dir.e_fsync_dir = e_fsync_dir;

  return GD_SUCCESS;
}


/**
 * @brief Set divider for BCLK
 *
 * Input of divider is connected to PLL, output of divider is used for BCLK.
 * @param i_div Divider ratio. Odd numbers are forbidden except 1 which mean\n
 *              "turn off divider"
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_set_BCLK_div(uint16_t i_div)
{
  // When need to read from PM registers
  volatile avr32_pm_t *p_pm;
  p_pm = (avr32_pm_t*)BRD_DRV_PM_ADDRESS;

  // Check input values
  if(i_div == 0) return GD_INCORRECT_PARAMETER;
  if(i_div > 512) return GD_INCORRECT_PARAMETER;
  /* Also we can not divide by odd values (3, 5, 7 and so on) except 1 (which
   * means "turn off divider"). To recognize odd value is simple: just check
   * LSB.
   */
  if((i_div != 1) &&
     (i_div & 0x0001))
  {
    return GD_INCORRECT_PARAMETER;
  }

  if(i_div == 1)
  {
    // Turn off divider
    p_pm->GCCTRL[1].diven = 0;
    p_pm->GCCTRL[1].div = 0;
  }
  else
  {
    // Turn on divider and set correct value. Note that >>1 is better than /2
    p_pm->GCCTRL[1].diven = 1;
    p_pm->GCCTRL[1].div = (i_div>>1) -1;
  }

  return GD_SUCCESS;
}

/**
 * @brief Give divider for BCLK
 * @param p_i_div Pointer to memory where result will be written
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_get_BCLK_div(uint16_t *p_i_div)
{
  // When need to read from PM registers
  volatile avr32_pm_t *p_pm;
  p_pm = (avr32_pm_t*)BRD_DRV_PM_ADDRESS;

  if(p_pm->GCCTRL[1].diven == 0)
  {
    // If divider disabled -> div = 1;
    *p_i_div = 1;
  }
  else
  {
    // There is some divider set
    *p_i_div = 2*(p_pm->GCCTRL[1].div +1);
  }

  return GD_SUCCESS;
}

/**
 * @brief Set divider for MCLK
 *
 * Input of divider is connected to PLL, output of divider is used for MCLK.
 * @param i_div Divider ratio. Odd numbers are forbidden except 1 which mean\n
 *              "turn off divider"
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_set_MCLK_div(uint16_t i_div)
{
  // When need to read from PM registers
  volatile avr32_pm_t *p_pm;
  p_pm = (avr32_pm_t*)BRD_DRV_PM_ADDRESS;

  // Check input values
  if(i_div == 0) return GD_INCORRECT_PARAMETER;
  if(i_div > 512) return GD_INCORRECT_PARAMETER;
  /* Also we can not divide by odd values (3, 5, 7 and so on) except 1 (which
   * means "turn off divider"). To recognize odd value is simple: just check
   * LSB.
   */
  if((i_div != 1) &&
     (i_div & 0x0001))
  {
    return GD_INCORRECT_PARAMETER;
  }

  if(i_div == 1)
  {
    // Turn off divider
    p_pm->GCCTRL[0].diven = 0;
    p_pm->GCCTRL[0].div = 0;
  }
  else
  {
    // Turn on divider and set correct value. Note that >>1 is better than /2
    p_pm->GCCTRL[0].diven = 1;
    p_pm->GCCTRL[0].div = (i_div>>1) -1;
  }

  return GD_SUCCESS;
}


/**
 * @brief Give divider for MCLK
 * @param p_i_div Pointer to memory where result will be written
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_get_MCLK_div(uint16_t *p_i_div)
{
  // When need to read from PM registers
  volatile avr32_pm_t *p_pm;
  p_pm = (avr32_pm_t*)BRD_DRV_PM_ADDRESS;

  if(p_pm->GCCTRL[0].diven == 0)
  {
    // If divider disabled -> div = 1;
    *p_i_div = 1;
  }
  else
  {
    // There is some divider set. Shifting is better than *2
    *p_i_div = (p_pm->GCCTRL[0].div +1)<<1;
  }

  return GD_SUCCESS;
}


//=========================| Functions not for user |==========================







/**
 * \brief Set TLV codec to default configuration
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_TLV_default(void)
{
  // Variable for storing status
  GD_RES_CODE e_status;

  // Initialize TLV
  e_status = tlv320aic33_init();
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Set codec as slave
  e_status = tlv320aic33_set_digital_interface_mode(0);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Set data interface (I2S - which is left justified with word offset 1)
  e_status = tlv320aic33_set_data_interface(
      serial_data_bus_uses_left_justified_mode);
  if(e_status != GD_SUCCESS) return e_status;

  // Also set word offset
  e_status = tlv320aic33_set_data_offset(1);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }
  brd_drv_send_msg(BRD_DRV_MSG_INFO_CDC_ITF_I2S, 1, 0, -1);

  // Set word length
  e_status = tlv320aic33_set_word_length(BRD_DRV_DEFAULT_DATA_WORD_LENGTH);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Want DAC to play stereo
  e_status = tlv320aic33_set_DAC_play_input_data(1);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Headphones output as signel ended
  e_status = tlv320aic33_set_headphones_single_ended(1);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Set headphones volume to some normal value.
  e_status = tlv320aic33_set_headphones_volume_dB(
      BRD_DRV_DEFAULT_HEADPHONES_VOLUME_IN_DB);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Set output driver delay
  e_status = tlv320aic33_set_output_driver_power_on_delay(
      driver_power_on_time_800ms);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Set ramp-up step timing
  e_status = tlv320aic33_set_driver_ramp_up_step_time(
      driver_ramp_up_step_time_2ms);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Enable DACs
  e_status = tlv320aic33_set_DAC_power(1);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Disable mute on DACs
  e_status = tlv320aic33_set_DAC_mute(0);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  return e_status;
}





/**
 * \brief Prepare ADC
 *
 * Set I/O, enable clock to ADC module (thru PM), reset ADC settings, set\n
 * active channels, prescaller and so on.
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_adc_init(void)
{
  // For backup IRQ flags
  uint32_t flags;

  // Variable as mask
  uint32_t mask;

  // Pointer to ADC structure
  volatile avr32_adc_t *p_adc;
  p_adc = (avr32_adc_t*)BDR_DRV_ADC_ADDRESS;

  //=================================| GPIO |==================================
  static const gpio_map_t brd_drv_adc_pin_map = {
      {BRD_DRV_ADC_VOLUME_CONTROL_PIN, BRD_DRV_ADC_VOLUME_CONTROL_FUNCTION},
      {BRD_DRV_ADC_CON_VOLTAGE_PIN   , BRD_DRV_ADC_CON_VOLTAGE_FUNCTION}
  };
  // Map ADC pin in GPIO module
  if(gpio_enable_module(brd_drv_adc_pin_map,
      sizeof(brd_drv_adc_pin_map)/sizeof(brd_drv_adc_pin_map[0])) != 0)
  {
    // If something goes wrong
    return GD_FAIL;
  }


  //==================================| PM |===================================
  // Enable clock to device (in default should be on, but just for case)
  // Get and clear global interrupt
  flags = __builtin_mfsr(AVR32_SR);
  // Disable IRQ
  __builtin_ssrf(AVR32_SR_GM_OFFSET);
  asm volatile("" ::: "memory");

  /*
   * Poll MSKRDY before changing mask rather than after, as it's
   * highly unlikely to actually be cleared at this point.
   */
  while (!(AVR32_PM.poscsr & (1U << AVR32_PM_POSCSR_MSKRDY))) {
          /* Do nothing */
  }

  // Enable the clock to flash and PBA bridge
  mask = *(&AVR32_PM.cpumask + AVR32_PM_CLK_GRP_HSB);
  mask |= 1U << (AVR32_FLASHC_CLK_HSB % 32);
  mask |= 1U << (AVR32_HMATRIX_CLK_HSB_PBA_BRIDGE % 32);
  *(&AVR32_PM.cpumask + AVR32_PM_CLK_GRP_HSB) = mask;

  // Enable clock to ADC and GPIO in PBA
  mask = *(&AVR32_PM.cpumask + AVR32_PM_CLK_GRP_PBA);
  mask |= 1U << (AVR32_ADC_CLK_PBA % 32);
  mask |= 1U << (AVR32_GPIO_CLK_PBA % 32);
  *(&AVR32_PM.cpumask + AVR32_PM_CLK_GRP_PBA) = mask;

  // Restore global interrupt flags
  asm volatile("" ::: "memory");
  __builtin_csrf(AVR32_SR_GM_OFFSET);
  asm volatile("" ::: "memory");


  //==================================| ADC |==================================
  // Backup registers that will be changed

  // Reset ADC -> known state
  p_adc->CR.swrst = 1;
  // Set ADC sample and hold time
  p_adc->MR.shtim = BRD_DRV_ADC_SAMPLE_HOLD_TIME;
  // Set start-up time
  p_adc->MR.startup = BRD_DRV_ADC_START_UP_TIME;
  // Set prescaller
  p_adc->MR.prescal = BRD_DRV_ADC_PRESCAL;
  // Turn off sleep mode - by default normal mode
  //p_adc->MR.sleep = 0;
  // 10 bit resolution
  p_adc->MR.lowres = 0;
  // Do not enable hardware triggers - by default off
  //p_adc->MR.trgen = 0;
  // Enable selected channels
  BRD_DRV_EN_ADC_CHANNEL(BRD_DRV_ADC_VOLUME_CONTROL_CHANNEL);
  BRD_DRV_EN_ADC_CHANNEL(BRD_DRV_ADC_CON_VOLTAGE_CHANNEL);

  // Everything should be ready, start conversion
  p_adc->CR.start = 1;

  return GD_SUCCESS;
}



/**
 * \brief Easily process MUTE_BTN signal
 *
 * Process only if changes on button (rising/falling edge)
 */
inline void brd_drv_process_mute_button(void)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Previous value of MUTE_BTN
  static uint8_t i_mute_btn_previous = 0;

  // Actual value of MUTE_BTN
  uint8_t i_mute_btn;

  BRD_DRV_IO_READ(i_mute_btn, BRD_DRV_MUTE_BTN_PIN);

  // Check for rising edge
  if((i_mute_btn != 0) && (i_mute_btn_previous == 0))
  {
    // Pressed button
    brd_drv_send_msg(BRD_DRV_MSG_INFO_MSG_MUTE_BTN_PRESSED, 1, 0, -1);
    brd_drv_set_mute(1);
  }

  // Check for falling edge
  if((i_mute_btn == 0) && (i_mute_btn_previous != 0))
  {
    // Released button
    brd_drv_send_msg(BRD_DRV_MSG_INFO_MSG_MUTE_BTN_RELEASED, 1, 0, -1);
    brd_drv_set_mute(0);
  }

  // Save actual button value
  i_mute_btn_previous = i_mute_btn;
}



/**
 * \brief Scan MUTE signal
 *
 * Process only if rising/falling edge
 */
inline void brd_drv_process_mute_signal(void)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Previous value of MUTE
  static uint8_t i_mute_previous = 0;

  // Actual MUTE value
  uint8_t i_mute;

  BRD_DRV_IO_READ(i_mute, BRD_DRV_MUTE_PIN);

  // Check rising edge
  if((i_mute != 0) && (i_mute_previous == 0))
  {
    brd_drv_send_msg(BRD_DRV_MSG_INFO_MUTE_RISING_EDGE, 1, 0, -1);
    // Mute on
    brd_drv_set_mute(1);
  }

  // Check falling edge
  if((i_mute == 0) && (i_mute_previous != 0))
  {
    brd_drv_send_msg(BRD_DRV_MSG_INFO_MUTE_FALLING_EDGE, 1, 0, -1);
    // Mute off
    brd_drv_set_mute(0);
  }

  // Save actual MUTE value
  i_mute_previous = i_mute;
}



/**
 * \brief Scan RESET_I2S_BTN and set/clear reset I2S signals
 */
inline void brd_drv_process_rst_i2s_button(void)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Previous value of RESET_I2S_BTN
  static uint8_t i_rst_i2s_btn_previous = 0;

  // For measuring "time"
  static uint32_t i_time = 0;

  // Flag that show, that reset was already set
  static uint8_t i_rst_i2s_already_set = 0;

  // Actual value of RESET_I2S_BTN
  uint8_t i_rst_i2s_btn;

  BRD_DRV_IO_READ(i_rst_i2s_btn, BRD_DRV_RESET_I2S_BTN_PIN);

  // Check if button is still pressed - measure time
  if(i_rst_i2s_btn != 0)
  {
    i_time++;
  }

  // Check for rising edge - pressed button
  if((i_rst_i2s_btn != 0) && (i_rst_i2s_btn_previous == 0))
  {
    brd_drv_send_msg(BRD_DRV_MSG_INFO_RST_I2S_BTN_PRESSED, 1, 0, -1);
  }


  // Check for falling edge - released button
  if((i_rst_i2s_btn == 0) && (i_rst_i2s_btn_previous != 0))
  {
    // Falling edge
    brd_drv_send_msg(BRD_DRV_MSG_INFO_RST_I2S_BTN_RELEASED, 1, 0, -1);

    if(i_time < BRD_DRV_SHORT_PRESS)
    {
      // Short press - just clean LCD, show logo and so on
      brd_drv_clean_up_error_msg();
    }// Short button press
    else
    {
      // Long button press
      // Reset I2S set to 0 (clear)
      brd_drv_set_rst_dai(0);
      i_rst_i2s_already_set = 0;
    }// Long button press
    // Always Reset timer
    i_time = 0;
  }// Falling edge

  // Time > short? If yes, then perform reset
  if((i_time > BRD_DRV_SHORT_PRESS) && (i_rst_i2s_already_set == 0))
  {
    brd_drv_set_rst_dai(1);
    i_rst_i2s_already_set = 1;
  }


  // Save actual button value
  i_rst_i2s_btn_previous = i_rst_i2s_btn;
}

/**
 * \brief Scan RESET_I2S signal
 *
 * Process only if rising/falling edge
 */
inline void brd_drv_process_rst_i2s_signal(void)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Previous value of RESET_I2S
  static uint8_t i_rst_i2s_previous = 0;

  // Actual RESET_I2S value
  uint8_t i_rst_i2s;

  // Check if pin is set as input
  if(s_brd_drv_rst_i2s.e_rst_dai_dir == brd_drv_dir_in)
  {
    BRD_DRV_IO_READ(i_rst_i2s, BRD_DRV_RESET_I2S_PIN);

    // Check rising edge
    if((i_rst_i2s != 0) && (i_rst_i2s_previous == 0))
    {
      brd_drv_send_msg(BRD_DRV_MSG_INFO_RST_I2S_RISING_EDGE, 1, 0, -1);
      // Reset on
      brd_drv_set_rst_dai(1);
    }

    // Check falling edge
    if((i_rst_i2s == 0) && (i_rst_i2s_previous != 0))
    {
      brd_drv_send_msg(BRD_DRV_MSG_INFO_RST_I2S_FALLING_EDGE, 1, 0, -1);
      // Reset off
      brd_drv_set_rst_dai(0);
    }

    // Save actual RESET_I2S value
    i_rst_i2s_previous = i_rst_i2s;
  }// Check if pin is set as input
}


inline void brd_drv_clean_up_info_msg(void)
{
  disp_clear_info();
}

inline void brd_drv_clean_up_error_msg(void)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;


  disp_clear_warn();
  disp_clear_err();

  // Turn off ERROR_SIG LED
  BRD_DRV_IO_LOW(BRD_DRV_ERROR_SIG_PIN);

  brd_drv_send_msg(BRD_DRV_MSG_INFO_ERROR_CLEANED, 1, 0, -1);
}
