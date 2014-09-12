/**
 * \file
 *
 * \brief Board driver for "Sonochan mkII" HW version 03
 *
 * Allow set basic settings on board. Also support "generic driver".
 * Written only for AVR32 UC3A3.
 *
 * Created:  23.04.2014\n
 * Modified: 09.09.2014
 *
 * \version 0.3
 * \author  Martin Stejskal
 */

#include "brd_driver_hw_03.h"
//============================| Global variables |=============================
/**
 * \brief Mute structure
 *
 * Contains mute direction and mute value
 */
static s_brd_drv_mute_t s_brd_drv_mute;

/**
 * \brief Reset I2S structure
 *
 * Contains reset I2S direction and value
 */
static s_brd_drv_rst_i2s_t s_brd_drv_rst_i2s;

/**
 * \brief Store directions for MCLK, BCLK, FRAME_SYNC, TX_DATA, RX_DATA
 */
static s_brd_drv_pure_i2s_dir_t s_brd_drv_pure_i2s_dir;

/**
 * \brief Store fine settings of SSC module
 */
static s_brd_drv_ssc_fine_setting_t s_brd_drv_ssc_fine_settings;

/**
 * \brief Store auto tune PLL value
 */
static uint8_t i_auto_tune_pll;

/**
 * \brief Store information about actual UAC (USB Audio Class)
 *
 * If value is non zero, then UAC1 is used. Else UAC2 (need more specific\n
 * settings).
 */
static uint8_t i_uac1 = 1;

/**
 * \brief Store information if error occurred
 *
 * Values: 0 - not error  ; else error occurred
 *
 */
static uint8_t i_error_occurred;

//===========================| "EEPROM" variables |============================
// Set up NVRAM (EEPROM) storage
#if defined (__GNUC__)
__attribute__((__section__(".userpage")))
#endif
/**
 * \brief Structure for saving and restoring settings
 */
s_brd_drv_user_settings_t s_brd_drv_user_settings;

/**
 * \brief Variable that allow detect correct or incorrect settings in user\n
 * flash
 */
#if defined (__GNUC__)
__attribute__((__section__(".userpage")))
#endif
uint8_t i_brd_drv_settings_check;

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
      (GD_DATA_VALUE*)&s_brd_drv_pure_i2s_dir.e_mclk_dir,
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
      (GD_DATA_VALUE*)&s_brd_drv_pure_i2s_dir.e_bclk_dir,
      brd_drv_set_bclk_dir
    },
    {
#define BRD_DRV_CMD_FSYNC_DIR   BRD_DRV_CMD_BCLK_DIR+1
      BRD_DRV_CMD_FSYNC_DIR,
      "FSYNC role and direction",
      "0 - slave, FSYNC (word clock) is input ; 1 - master, FSYNC (word clock) is output ; 2 - hi-z",
      uint32_type,      // Cause it is enum and 32 bit system
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 2},
      (GD_DATA_VALUE*)&s_brd_drv_pure_i2s_dir.e_frame_sync_dir,
      brd_drv_set_frame_sync_dir
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
      (GD_DATA_VALUE*)&s_brd_drv_rst_i2s.e_rst_i2s_dir,
      brd_drv_set_rst_i2s_dir
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
      (GD_DATA_VALUE*)&s_brd_drv_rst_i2s.i_rst_i2s_val,
      brd_drv_set_rst_i2s
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
      (GD_DATA_VALUE*)&s_brd_drv_pure_i2s_dir.e_tx_data_dir,
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
      (GD_DATA_VALUE*)&s_brd_drv_pure_i2s_dir.e_rx_data_dir,
      brd_drv_set_rx_data_dir
    },
    {
#define BRD_DRV_CMD_MCLK_OVERSAM        BRD_DRV_CMD_RX_DATA_DIR+1
      BRD_DRV_CMD_MCLK_OVERSAM,
      "MCLK frequency",
      "Options: 16, 32, 64, 128, 256 FSYNC",
      uint16_type,
      {.data_uint16 = 1},
      {.data_uint16 = 512},
      uint16_type,
      {.data_uint16 = 1},
      {.data_uint16 = 512},
      (GD_DATA_VALUE*)&s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling,
      brd_drv_set_MCLK_oversampling
    },
    {
#define BRD_DRV_CMD_BCLK_OVERSAM        BRD_DRV_CMD_MCLK_OVERSAM+1
      BRD_DRV_CMD_BCLK_OVERSAM,
      "BCLK frequency",
      "Options: 16, 32, 64, 128, 256 FSYNC",
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
#define BRD_DRV_CMD_AUTO_TUNE           BRD_DRV_CMD_BCLK_OVERSAM+1
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
      brd_drv_auto_tune
    },
    {
#define BRD_DRV_CMD_RX_FSYNC_EDGE    BRD_DRV_CMD_AUTO_TUNE+1
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
      "TX FSYNC sync edge ; 0 - falling ; 1- rising ; 2 - default",
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
#define BRD_DRV_CMD_DIG_AUD_INTERFACE   BRD_DRV_CMD_TX_BCLK_EDGE+1
      BRD_DRV_CMD_DIG_AUD_INTERFACE,
      "Digital audio interface mode",
      "NOT TESTED! ; 0 - I2S ; 1 - DSP ; 2 - Left justified ; 3 - Right justified",
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
#define BRD_DRV_CMD_WORD_SIZE   BRD_DRV_CMD_DIG_AUD_INTERFACE+1
      BRD_DRV_CMD_WORD_SIZE,
      "Set word size",
      "NOT TESTED! Data size in bits. Usually 16, 20, 24 and 32",
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
#define BRD_DRV_CMD_TEST_FUNC   BRD_DRV_CMD_WORD_SIZE+1
      BRD_DRV_CMD_TEST_FUNC,
      "Test function",
      "For testing",
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0xFFFFFFFF},
      void_type,
      {.data_uint32 = 0},
      {.data_uint32 = 0},
      (GD_DATA_VALUE*)&gd_void_value,
      brd_drv_test_f
    },
    {
#define BRD_DRV_CMD_RESTORE_SETTING     BRD_DRV_CMD_TEST_FUNC+1
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
        "Board driver for Sonochan mkII v0.3",     // Description
        (gd_config_struct*)&BRD_DRV_config_table[0],
        0x0F    // Serial number (0~255)
};
#endif
//[DEBUG]
///\todo REMOVE THIS DEBUG STUFF
#include "ssc.h"
#include "sync_control.h"
#include "pdca.h"
#include "taskAK5394A.h"


GD_RES_CODE brd_drv_test_f(uint32_t i32)
{
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;
  //char c[150];

  print_dbg("TEST FUNCTION ...\n\n");


  return GD_SUCCESS;
}

//[/DEBUG]

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
  portTickType xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();
  while (TRUE)
  {
    vTaskDelayUntil(&xLastWakeTime, configTSK_HW_bridge_uniprot_PERIOD);

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

  // Set I/O - this should be OK
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

  // So far so good
  i_error_occurred = 0;

  //=================================| IO pins |===============================
  // Set isolators I/O (recommended default)
  brd_drv_set_isolators_to_HiZ();

  //======================| Route MCLK and set prescaller |====================
  // OSC1 is clocked by 12.288Mhz Osc (Clock from external PLL)
  pm_enable_osc1_ext_clock(&AVR32_PM);
  // from Xtal Oscillator
  pm_enable_clk1(&AVR32_PM, AVR32_PM_OSCCTRL1_STARTUP_2048_RCOSC);

  /*[Martin] Use GCLK0 as master clock because PLL can not produce
   * frequency lower than 6 MHz. So we can use divider. In default we use
   * 48 kHz sampling rate, so divider will be disabled. So far support is
   * only for UAC1
   */
  gpio_enable_module_pin(GCLK0, GCLK0_FUNCTION);
  if(i_uac1 == 0)
  {
    // UAC2
    pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK0,   // GCLK0
        0,        // OSC source
        1,        // OSC1
        0,        // Divider disabled
        0);       // Divider ratio -> Fout = Fin/( 2*(DIV+1) )
    ///\todo Test USB Audio Class 2 and check if THIS CODE is correct
    brd_drv_send_error_msg("BRD DRV: UAC2 not tested yet!\n", 1, 1);
  }
  else
  {
    // UAC1
    pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK0,   // GCLK0
        0,        // OSC source
        1,        // OSC1
        0,        // Divider disabled
        0);       // Divider ratio -> Fout = Fin/( 2*(DIV+1) )
  }
  pm_gc_enable(&AVR32_PM, AVR32_PM_GCLK_GCLK0);

  //======================| Route BCLK and set prescaller |====================
  // Route BCLK to GCLK1 module
  gpio_enable_module_pin(GCLK1, GCLK1_FUNCTION);
  // LRCK is SCLK / 64 generated by TX_SSC
  // so SCLK of 6.144Mhz ===> 96khz
  if(i_uac1 == 0)
  {
    // UAC2
    ///\todo Simplify - Just write to registers
    pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1, // gc
          0,                  // osc_or_pll: use Osc (if 0) or PLL (if 1)
          1,                  // pll_osc: select Osc0/PLL0 or Osc1/PLL1
          1,                  // diven - enabled
          0);                 // divided by 2.  Therefore GCLK1 = 6.144Mhz
  }
  else
  {
    // UAC1
    ///\todo Simplify - Just write to registers
    pm_gc_setup(&AVR32_PM, AVR32_PM_GCLK_GCLK1, // gc
          0,                  // osc_or_pll: use Osc (if 0) or PLL (if 1)
          1,                  // pll_osc: select Osc0/PLL0 or Osc1/PLL1
          1,                  // diven - enabled
          1);                 // divided by 4.  Therefore GCLK1 = 3.072Mhz
  }
  // Anyway enable clock
  pm_gc_enable(&AVR32_PM, AVR32_PM_GCLK_GCLK1);

  //================================| LCD stuff |==============================
  // Initialize LCD
  e_status = LCD_5110_init();
  if(e_status != GD_SUCCESS)
  {
    const char msg_lcd_init_fail[] = BRD_DRV_MSG_LCD_INIT_FAIL;
    // Send message over debug interface
    brd_drv_send_error_msg(&msg_lcd_init_fail[0], 1, 0);
    return e_status;
  }

  // Configure LCD
  // We do not want "book style" LCD
  LCD_5110_auto_clear_display(0);
  // We want auto clean line
  LCD_5110_auto_clear_line(1);
  /* Do not need automatic newline, because all messages have '\n' because of
   * debug output
   */
  LCD_5110_auto_newline(0);

  // Draw logo (function, easy to change in future)
  e_status = brd_drv_draw_logo();
  if(e_status != GD_SUCCESS)
  {
    const char msg_draw_logo_fail[] = BRD_DRV_MSG_DRAW_LOGO_FAIL;
    brd_drv_send_error_msg(&msg_draw_logo_fail[0], 1, 0);
    return e_status;
  }

//==============================| External PLL |===============================
  // Precise setting of PLL
  /* Turn of save setting when changing frequency by 1. We want continuous
   * clock at "any price". PLL CLK OUT should work as expected.
   */
  e_status = cs2200_set_safe_change_ratio_by_1(0);
  if(e_status != GD_SUCCESS)
  {
    const char msg_set_pll_save_flag_when_change_by_1[] =
        BRD_DRV_MSG_PLL_SET_SAVE_FLAG_FAIL;
    brd_drv_send_error_msg(&msg_set_pll_save_flag_when_change_by_1[0], 1 ,1);
    return e_status;
  }


//===================================| ADC |===================================
  // Take control over ADC
  BRD_DRV_LOCK_ADC_MODULE_IF_RTOS
  // ADC for volume control
  e_status = brd_drv_adc_init();
  // Give back control on ADC
  BRD_DRV_UNLOCK_ADC_MODULE_IF_RTOS

  if(e_status != GD_SUCCESS)
  {
    const char msg_adc_init_fail[] = BRD_DRV_MSG_ADC_INIT_FAIL;
    brd_drv_send_error_msg(&msg_adc_init_fail[0], 1, 1);
    return e_status;
  }


//===============================| SSC module |================================
  // Preset SSC
  e_status = ssc_init();
    if(e_status != GD_SUCCESS)
      return e_status;

//==================================| Codec |==================================
  // Prepare TLV (CS2200 have to be already initialized)
  e_status = brd_drv_TLV_default();
  if(e_status != GD_SUCCESS)
  {
    const char msg_codec_init_fail[] = BRD_DRV_MSG_CODEC_INIT_FAIL;
    brd_drv_send_error_msg(&msg_codec_init_fail[0], 1, 1);
    return e_status;
  }


//======================| Try load settings from flash |=======================
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



  /* If FreeRTOS is used, then create task. Note that
   * "configTSK_brd_drv_*" should be defined in FreeRTOSConfig.h
   * file to keep code clear. However it should be possible set settings
   * here.
   */
#if BRD_DRV_SUPPORT_RTOS != 0
  xTaskCreate(brd_drv_task_FreeRTOS,             // Task function
      configTSK_brd_drv_NAME,       // String name
      configTSK_brd_drv_STACK_SIZE, // Stack size (2^N)
      NULL,
      configTSK_brd_drv_PRIORITY,   // 0 is lowest
      NULL);
#endif  // FREERTOS_USED

  return GD_SUCCESS;
}








/**
 * \brief Board driver task. Watch ADC and buttons
 *
 * If detected some change, then start processing. Else return
 */
void brd_drv_task(void)
{
  // Error/warning/info messages
  const char msg_tlv_failed_set_headphone_volume_dB[] =
      BRD_DRV_TLV_FAILED_SET_HEADPHONE_VOL_DB;
  const char msg_con_vol_low[] = BRD_DRV_CON_VOL_LOW;
  const char msg_con_vol_save[] = BRD_DRV_CON_VOL_SAVE;
  const char msg_con_vol_high[] = BRD_DRV_CON_VOL_HIGH;
  const char msg_con_not_powered[] = BRD_DRV_CON_NOT_POWERED;


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

  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;


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
          brd_drv_send_error_msg(&msg_tlv_failed_set_headphone_volume_dB[0],1,1);
        }
        // Show volume value
        brd_drv_show_volume();
      }
    }
    else
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
          brd_drv_send_error_msg(&msg_tlv_failed_set_headphone_volume_dB[0],1,1);
        }
        // Show volume value
        brd_drv_show_volume();
      }
    }// Test if volume value has been changed






    //=========================| Check connector voltage |=====================
    // Process mute and reset_i2s signals

    // Check CON_VOLTAGE value
    if(i_voltage_connector_side >= BRD_DRV_ADC_SAVE_VOL_MIN)
    {
      // Power off or very low voltage. Do not scan mute and reset_i2s signals
      // Check if state is different and not high voltage
      if((e_con_vol != brd_drv_con_low_vol) &&
         (e_con_vol != brd_drv_con_high_vol))
      {
        // If actual status is not same -> change
        e_con_vol = brd_drv_con_low_vol;

        // Send message
        brd_drv_send_msg(&msg_con_vol_low[0], 1, 0, -1);

        // Nothing more to do
      }// Check if state is different
    }// Check CON_VOLTAGE value
    else if(i_voltage_connector_side >BRD_DRV_ADC_HIGH_VOL_MIN)
    {
      // Connector side is powered and voltage is in save area
      // Check if state is not high voltage (previous state) and save voltage
      if((e_con_vol != brd_drv_con_high_vol) &&
         (e_con_vol != brd_drv_con_save_vol))
      {
        // Save actual state
        e_con_vol = brd_drv_con_save_vol;

        // Send message
        brd_drv_send_msg(&msg_con_vol_save[0], 1, 0, -1);
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
        brd_drv_send_warning_msg(&msg_con_vol_high[0], 1, 1);
      }
    }

    /* If previous state was high voltage and voltage is much
     * lower (approx. +5.0 V)
     */
    if((e_con_vol == brd_drv_con_high_vol) &&
            ((i_voltage_connector_side >(BRD_DRV_ADC_HIGH_VOL_MIN+1) )))
    {
      // Clear lines if voltage is in save range
      brd_drv_draw_logo();
      // Set status to undefined. Next "round" check voltage again...
      e_con_vol = brd_drv_con_undefined;
    }// Check CON_VOLTAGE value


    //=============================| Process signals |=========================
    // Process only if connector side is powered
    if((e_con_vol == brd_drv_con_save_vol) ||
       (e_con_vol == brd_drv_con_high_vol))
    {
      // Mute signal
      brd_drv_process_mute_signal();

      // Reset signal
      brd_drv_process_rst_i2s_signal();
    }
    // Process when connector side is not powered
    if(e_con_vol == brd_drv_con_low_vol)
    {
      // Load button values
      uint8_t i_mute_btn;
      uint8_t i_rst_i2s_btn;
      BRD_DRV_IO_READ(i_mute_btn, BRD_DRV_MUTE_BTN_PIN);
      BRD_DRV_IO_READ(i_rst_i2s_btn, BRD_DRV_RESET_I2S_BTN_PIN);

      if(i_mute_btn != 0)
      {
        // Mute button pressed - write to LCD info
        brd_drv_send_msg(&msg_con_not_powered[0],0,1,
            BRD_DRV_LCD_INFO_MSG_LINE);
      }
    }
    // Anyway process mute button. Maybe user set up volume too high
    brd_drv_process_mute_button();

    // Reset button process
    brd_drv_process_rst_i2s_button();


  }/* Check if all ADC conversions are done and if there is "time" for some
    * data processing
    */


  // Increase counter (always)
  i_time_cnt++;
}







/**
 * \brief Preset I2S connector to default state. Also reset codec.
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_reset_i2s(void)
{
  const char msg_restart_i2s_done[] = BRD_DRV_MSG_RESET_I2S_DONE;

  // Reset to fail save mode -> Hi-Z
  brd_drv_set_isolators_to_HiZ();
  brd_drv_TLV_default();

  // Send message to debug interface
  brd_drv_send_msg(&msg_restart_i2s_done[0], 1, 0, -1);

  return GD_SUCCESS;
}



/**
 * @brief Simple function, that allow set digital audio interface
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

  e_status = ssc_set_digital_interface_mode(e_mode);
  if(e_status == GD_SUCCESS)
  {
    // If all OK -> save actual mode value
    s_brd_drv_ssc_fine_settings.e_dig_aud_mode = e_mode;
  }
  // Anyway, return status code
  return e_status;
}



/**
 * @brief Set FSYNC frequency
 * Because all is based from external PLL frequency, function set also\n
 * external PLL, MCLK and BCLK dividers (on UC3A). As result of all this\n
 * should be correct FSYNC frequency.
 * @note This function is used by USB driver, so that is main reason why all\n
 * messages (debug/info) are commened
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

  // Set MCLK. Because function can change PLL freq, itlsef set BCLK again
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
GD_RES_CODE brd_drv_get_FSYNC_freq(uint32_t *p_i_FSYNC_freq)
{
  *p_i_FSYNC_freq = s_brd_drv_ssc_fine_settings.i_FSYNC_freq;
  return GD_SUCCESS;
}


/**
 * @brief Set MCLK oversampling frequency in reference to FSYNC
 * Also set again BCLK, because function itself can change PLL frequency,\n
 * but BCLK divider ratio would stayed unchanged -> wrong BCLK. So that is\n
 * why function set also BCLK again.
 *
 * @param i_MCLK_oversampling Options: 16, 32, 64, 128 and 256
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_MCLK_oversampling(uint16_t i_MCLK_oversampling)
{
  ///\todo Check input parameter (512 max???)

  /* [Martin]
   * This function must be very flexible and work with PLL CS2200 (set freq.),
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
    if(i_PLL_freq > 75000000UL)
    {
      // PLL can not produce higher frequency :( Return fail
      brd_drv_send_error_msg(BRD_DRV_MSG_ERR_PLL_HIGH_FREQ,1,1);
      return GD_FAIL;
    }
    if(i_PLL_freq < 6000000UL)
    {
      /* Can not produce such a low frequency, but can set divider. So just
       * double PLL frequency and double MCLK divider.
       */
      i_PLL_freq = i_PLL_freq<<1;
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

  // Set PLL frequency
  e_status = cs2200_set_PLL_freq(i_PLL_freq);
  if(e_status != GD_SUCCESS)
  {
    brd_drv_send_error_msg(BRD_DRV_MSG_ERR_CAN_NOT_SET_PLL_FREQ,1,1);
    return e_status;
  }

  // So far so good. So let's set MCLK oversampling value
  s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling = i_MCLK_oversampling;

  /* Set again BCLK divider (resp. oversampling value)
   * This function SHOULD NOT fail, but just for case there is if condition.
   * If there is any problem, it is probably bug, not user fail.
   */
  e_status = brd_drv_set_BCLK_oversampling(
      s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling);
  if(e_status != GD_SUCCESS)
  {
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


  // BCLK oversampling value backup (If something goes wrong, restore it)
  uint16_t i_BCLK_ovrsmpling_backup =
      s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling;
  s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling = i_BCLK_ovrsmpling;


  // Check MCLK oversampling value. BCLK must not be higher than MCLK
  if(i_BCLK_ovrsmpling > s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling)
  {
    // We must set correct oversampling value for MCLK
    brd_drv_send_warning_msg(BRD_DRV_MSG_WRN_MCLK_RAISED_UP_BCLK,1,1);
    // So, MCLK should be equal BCLK
    e_status = brd_drv_set_MCLK_oversampling(i_BCLK_ovrsmpling);
    if(e_status != GD_SUCCESS)
    {
      s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling = i_BCLK_ovrsmpling_backup;
      return e_status;
    }
  }
  // Also must check data word size.
  if((i_BCLK_ovrsmpling>>1) < s_brd_drv_ssc_fine_settings.i_data_length )
  {
    /* This case means, that we have for example 24 bit resolution, but
     * due to BCLK oversampling value, we can process less bits. So we must
     * set also SSC, PDCA and so on..... This is not "normal" behavior but user
     * may not check all input parameters. So write warning message
     */
    brd_drv_send_warning_msg(BRD_DRV_MSG_WRN_DATA_LEN_DECREASED,1,1);
    e_status = brd_drv_set_data_length(i_BCLK_ovrsmpling>>1);
    if(e_status != GD_SUCCESS)
    {
      s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling = i_BCLK_ovrsmpling_backup;
      return e_status;
    }
  }

  // Calculate relative ratio
  i_MCLK_rel_div = s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling /
                            i_BCLK_ovrsmpling;

  /* Because divider for MCLK can have already set some ratio, we need to add
   * both values to get correct value for divider
   */
  e_status = brd_drv_get_MCLK_div(&i_MCLK_div);
  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling = i_BCLK_ovrsmpling_backup;
    return e_status;
  }

  // Calculate complete ratio
  i_MCLK_rel_div = i_MCLK_rel_div * i_MCLK_div;

  // Set correct BCLK
  e_status = brd_drv_set_BCLK_div(i_MCLK_rel_div);
  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling = i_BCLK_ovrsmpling_backup;
    return e_status;
  }

  // Also need to set SSC module - frame length is 1/2 of BCLK oversampling
  e_status = ssc_set_frame_length(i_BCLK_ovrsmpling>>1);
  if(e_status != GD_SUCCESS)
  {
    s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling = i_BCLK_ovrsmpling_backup;
    return e_status;
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
      (void *)&s_brd_drv_user_settings.e_rst_i2s_dir,
      s_brd_drv_rst_i2s.e_rst_i2s_dir,
      sizeof(s_brd_drv_rst_i2s.e_rst_i2s_dir),
      1);

  // MCLK direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_mclk_dir,
      (uint32_t)s_brd_drv_pure_i2s_dir.e_mclk_dir,
      sizeof(s_brd_drv_pure_i2s_dir.e_mclk_dir),
      1);

  // BCLK direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_bclk_dir,
      s_brd_drv_pure_i2s_dir.e_bclk_dir,
      sizeof(s_brd_drv_pure_i2s_dir.e_bclk_dir),
      1);

  // FRAME SYNC direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_frame_sync_dir,
      s_brd_drv_pure_i2s_dir.e_frame_sync_dir,
      sizeof(s_brd_drv_pure_i2s_dir.e_frame_sync_dir),
      1);

  // TX DATA direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_tx_data_dir,
      s_brd_drv_pure_i2s_dir.e_tx_data_dir,
      sizeof(s_brd_drv_pure_i2s_dir.e_tx_data_dir),
      1);

  // RX DATA direction
  flashc_memset32(
      (void *)&s_brd_drv_user_settings.e_rx_data_dir,
      s_brd_drv_pure_i2s_dir.e_rx_data_dir,
      sizeof(s_brd_drv_pure_i2s_dir.e_rx_data_dir),
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
  flashc_memset8(
      (void *)&s_brd_drv_user_settings.i_word_bit_offset,
      s_brd_drv_ssc_fine_settings.i_word_bit_offset,
      sizeof(s_brd_drv_ssc_fine_settings.i_word_bit_offset),
      1);


  // Save information (code), that settings was saved
  flashc_memset8(
      (void *)&i_brd_drv_settings_check,
      BRD_DRV_FLASH_CHECK_CODE,
      sizeof(BRD_DRV_FLASH_CHECK_CODE),
      1);

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
  e_status = brd_drv_set_rst_i2s_dir(s_brd_drv_user_settings.e_rst_i2s_dir);
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

  // BCLK direction
  e_status = brd_drv_set_bclk_dir(s_brd_drv_user_settings.e_bclk_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! BCLK dir fail ! ");
    return e_status;
  }

  // FRAME SYNC direction
  e_status=brd_drv_set_frame_sync_dir(s_brd_drv_user_settings.e_frame_sync_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! FS dir fail ! ");
    return e_status;
  }

  // TX DATA
  e_status = brd_drv_set_tx_data_dir(s_brd_drv_user_settings.e_tx_data_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! TXD dir fail ! ");
    return e_status;
  }

  // RX DATA
  e_status = brd_drv_set_rx_data_dir(s_brd_drv_user_settings.e_rx_data_dir);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! RXD dir fail ! ");
    return e_status;
  }



  // Auto tune PLL
  e_status = brd_drv_auto_tune(s_brd_drv_user_settings.i_auto_tune_pll);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! Auto tune PLL fail ! ");
    return e_status;
  }



  // SSC fine settings
  e_status = brd_drv_set_data_length(s_brd_drv_user_settings.i_data_length);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! Set data length failed ! ");
    return e_status;
  }
  e_status = brd_drv_set_digital_audio_interface_mode(
      s_brd_drv_user_settings.e_dig_audio_mode);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! Set dig. aud. itfce failed ! ");
    return e_status;
  }
  /* Set FSYNC frequency. Also function set MCLK and BCLK oversampling values
   * so we need to set them first
   */
  s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling =
      s_brd_drv_user_settings.i_MCLK_ovrsmpling;
  s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling =
      s_brd_drv_user_settings.i_BCLK_ovrsmpling;
  e_status = brd_drv_set_FSYNC_freq(s_brd_drv_user_settings.i_FSYNC_freq);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! Set FSYNC FREQ failed ! ");
    return e_status;
  }

  e_status = brd_drv_set_BCLK_RX_edge(
      s_brd_drv_user_settings.e_BCLK_RX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! BCLK RX edge fail ! ");
    return e_status;
  }
  e_status = brd_drv_set_BCLK_TX_edge(
      s_brd_drv_user_settings.e_BCLK_TX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! BCLK TX edge fail ! ");
    return e_status;
  }
  e_status = brd_drv_set_FSYNC_RX_edge(
      s_brd_drv_user_settings.e_FSYNC_RX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! RX FSYNC dir fail ! ");
    return e_status;
  }
  e_status = brd_drv_set_FSYNC_TX_edge(
      s_brd_drv_user_settings.e_FSYNC_TX_edge);
  if(e_status != GD_SUCCESS)
  {
    print_dbg(" ! FSYNC TX edge fail ! ");
    return e_status;
  }
  ///\todo Set through FSYNC pulse function
  ///\todo Set through word offset function

  // Just return status of last function
  return e_status;
}







inline GD_RES_CODE brd_drv_load_default_settings(void)
{
  // Store status codes
  GD_RES_CODE e_status;

  brd_drv_send_msg(BRD_DRV_MSG_INFO_LOAD_FACTRY_STTNGS,1,0,-1);
  // Delete actual settings by changing check Byte
  flashc_memset8(
      (void *)&i_brd_drv_settings_check,
      BRD_DRV_FLASH_CHECK_CODE+1, // Make sure, that code will be different
      sizeof(BRD_DRV_FLASH_CHECK_CODE),
      1);

  // Load value i_auto_tune_pll
  uac1_device_audio_get_auto_tune(&i_auto_tune_pll);

  // Set as default I2S mode
  e_status = ssc_set_digital_interface_mode(SSC_I2S);
  if(e_status != GD_SUCCESS) return e_status;

  // Load SSC default values
  // Data length
  e_status = brd_drv_set_data_length(BRD_DRV_DEFAULT_DATA_WORD_LENGTH);
  if(e_status != GD_SUCCESS) return e_status;
  // FSYNC RX
  e_status = brd_drv_set_FSYNC_RX_edge(SSC_EDGE_DEFAULT);
  if(e_status != GD_SUCCESS) return e_status;
  // FSYNC TX
  e_status = brd_drv_set_FSYNC_TX_edge(SSC_EDGE_DEFAULT);
  if(e_status != GD_SUCCESS) return e_status;
  // BCLK RX
  e_status = ssc_get_BCLK_RX_edge(
      &s_brd_drv_ssc_fine_settings.e_BCLK_RX_edge);
  if(e_status != GD_SUCCESS) return e_status;
  // BCLK TX
  e_status = ssc_get_BCLK_TX_edge(
      &s_brd_drv_ssc_fine_settings.e_BCLK_TX_edge);
  if(e_status != GD_SUCCESS) return e_status;
  // MCLK and BCLK oversampling (Default);
  s_brd_drv_ssc_fine_settings.i_MCLK_ovrsmpling =
      BRD_DRV_DEFAULT_MCLK_OVERSAMPLING;
  s_brd_drv_ssc_fine_settings.i_BCLK_ovrsmpling =
      BRD_DRV_DEFAULT_BCLK_OVERSAMPLING;
  // And call set FSYNC frequency, which set MCLK and BCLK dividers
  e_status = brd_drv_set_FSYNC_freq(BRD_DRV_DEFAULT_FSYNC_FREQ);
  if(e_status != GD_SUCCESS) return e_status;


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

  ///\todo More processing (set also PDCA!!!)

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
  }
  else  // Supported value
  {
    e_status = tlv320aic33_set_word_length(i_data_length);
  }
  // Check status code
  if(e_status != GD_SUCCESS) return e_status;


  // If all OK, then save value
  s_brd_drv_ssc_fine_settings.i_data_length = i_data_length;

  // Anyway, return status
  return e_status;
}


/**
 * @brief Give information about actual settings of data word length
 * @param p_i_data_length Pointer to memory, where result will be written
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_get_data_length(uint8_t *p_i_data_length)
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

  // OK, so we got mode. So now it is time for switch
  switch(e_mode)
  {
  // I2S
  case SSC_I2S:
    // If rising  edge -> switch channels
    if(e_edge == SSC_EDGE_RISING)
    {
      uac1_device_audio_set_swap_LR_RX(1);
    }
    else // Do not switch channel
    {
      uac1_device_audio_set_swap_LR_RX(0);
    }
    break; // I2S
  // Mode not known/not supported
  default:
    return GD_FAIL;
  }

  // OK, save actual value
  s_brd_drv_ssc_fine_settings.e_FSYNC_RX_edge = e_edge;

  // Return last status
  return e_status;
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

  // OK, so we got mode. So now it is time for switch
  switch(e_mode)
  {
  // I2S
  case SSC_I2S:
    // If rising  edge -> switch channels
    if(e_edge == SSC_EDGE_RISING)
    {
      uac1_device_audio_set_swap_LR_TX(1);
    }
    else // Do not switch channel
    {
      uac1_device_audio_set_swap_LR_TX(0);
    }
    break; // I2S
  // Mode not known/not supported
  default:
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
GD_RES_CODE brd_drv_set_BCLK_RX_edge(e_ssc_edge_t e_edge)
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
GD_RES_CODE brd_drv_set_BCLK_TX_edge(e_ssc_edge_t e_edge)
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
 * \brief Show volume value on LCD
 * @return GD_SUCCESS (0) if all right
 */
inline GD_RES_CODE brd_drv_show_volume(void)
{
  // Show actual headphone volume settings
  // Volume as float value
  float f_volume;
  /* Temporary string for text. Assume that text will not be longer than one
   * line on LCD
   */
  char c[15];
  tlv320aic33_get_headphones_volume_db(&f_volume);
  sprintf(&c[0], "Vol: %.1f dB\n", f_volume);
  brd_drv_send_msg(&c[0], 0, 1, BRD_DRV_LCD_HP_VOLUME_LINE);

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
 * the following line.
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
    // If user want write on defined line
    if(i_LCD_line < 6)
    {
      LCD_5110_set_line(i_LCD_line);
      // Write message
      LCD_5110_write(p_msg);
    }
    else
    {
      // Just continue writing...
      LCD_5110_write(p_msg);
    }
  }
}




/**
 * \brief Draw Sonochan mk II logo on display
 *
 * Function assume, that LCD_init() od LCD_clear() was called
 *
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_draw_logo(void)
{
  // For return result code
  GD_RES_CODE e_status;

  ///\todo Audo DATE and time only for debug. In release version number
  char snchn_date[] = {__DATE__};
  char snchn_time[] = {__TIME__};
  // Put null to "corrct place"
  snchn_date[6] = 0x00;
  snchn_time[5] = 0x00;


  e_status = LCD_5110_write_xy(BRD_DRV_MSG_INFO_SONOCHAN_MK_II,
      1,
      BRD_DRV_LCD_LOGO_LINE);
  if(e_status != GD_SUCCESS) return e_status;


  e_status = LCD_5110_write_xy(&snchn_date[0], 0, BRD_DRV_LCD_LOGO_LINE+1);
  if(e_status != GD_SUCCESS) return e_status;
  e_status = LCD_5110_write_xy(&snchn_time[0], 8*6, BRD_DRV_LCD_LOGO_LINE+1);
  if(e_status != GD_SUCCESS) return e_status;

  return e_status;
}



/**
 * \brief Write warning message
 *
 * When writing warning message on LCD, then is there reserved 2 lines for\n
 * message. It can be longer, but 2 lines will be cleared. First line is\n
 * defined in .h file as BRD_DRV_LCD_WARN_MSG_LINE
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
  // Blink LCD backlight - turn off
  LCD_5110_backlight_off();

  if(i_write_to_DBG != 0)
  {
    print_dbg("Warn > ");
    print_dbg(p_msg);
  }
  if(i_write_to_LCD != 0)
  {
    // Select line
    LCD_5110_set_line(BRD_DRV_LCD_WARN_MSG_LINE);
    LCD_5110_write(p_msg);
    // Enable auto clear LCD when writing again to line 0
  }

  // Turn on
  LCD_5110_backlight_on();
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

  // Error - increase counter
  i_error_occurred++;

  // If want write to debug interface
  if(i_write_to_DBG != 0)
  {
    print_dbg("Error > ");
    print_dbg(p_msg);
  }

  // If want write to LCD
  if(i_write_to_LCD != 0)
  {
    // Because of LCD dimensions it should be cleaned first
    LCD_5110_clear();
    LCD_5110_write(p_msg);
  }
}

//===========================| Low level functions |===========================
/**
 * @brief Set USB Audio Class mode
 * Because device has availability to support UAC1 and UAC2 when initializing\n
 * program should know actual settings, because of right setting MCLK and BCLK.
 *
 * @param i_uac1_enable Write non zero value to enable USB Audio Class 1.\n
 * Else USB Audio Class 2 will be selected as option.
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_uac1(uint8_t i_uac1_enable)
{
  if(i_uac1_enable == 0)
  {
    i_uac1 = 0;
  }
  else
  {
    i_uac1 = 1;
  }
  return GD_SUCCESS;
}

/**
 * @brief Give actual value of USB Audio Class mode
 * @param p_i_uac1_enable Address, where result will be written.\n
 * Zero means UAC2. Other values mean UAC1.
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_get_uac1(uint8_t *p_i_uac1_enable)
{
  *p_i_uac1_enable = i_uac1;
  return GD_SUCCESS;
}


/**
 * \brief Enable or disable auto tune feature
 * @param i_enable Enable (1) or disable (0) feature
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_auto_tune(uint8_t i_enable)
{
  if(i_enable == 0)
  {
    uac1_device_audio_set_auto_tune(0);
    i_auto_tune_pll = 0;
  }
  else
  {
    uac1_device_audio_set_auto_tune(1);
    i_auto_tune_pll = 1;
  }

  return GD_SUCCESS;
}

/**
 * \brief Set pins, that control signal direction at digital isolators, to low
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_set_isolators_to_HiZ(void)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  //================================| Pure I2S |===============================
  // Set output enables to low -> disable them by default
  brd_drv_set_mclk_dir(brd_drv_dir_hiz);
  brd_drv_set_bclk_dir(brd_drv_dir_hiz);
  brd_drv_set_frame_sync_dir(brd_drv_dir_hiz);
  brd_drv_set_tx_data_dir(brd_drv_dir_hiz);
  brd_drv_set_rx_data_dir(brd_drv_dir_hiz);
  //=============================| Reset and mute |============================
  brd_drv_set_mute_dir(brd_drv_dir_hiz);
  brd_drv_set_rst_i2s_dir(brd_drv_dir_hiz);

  // Button pins - it is set as input after reset but just for sure
  BRD_DRV_IO_AS_INPUT(BRD_DRV_MUTE_BTN_PIN);
  BRD_DRV_IO_AS_INPUT(BRD_DRV_RESET_I2S_BTN_PIN);

  // Signal pins - it is set as input after reset but just for sure
  BRD_DRV_IO_AS_INPUT(BRD_DRV_MUTE_PIN);
  BRD_DRV_IO_AS_INPUT(BRD_DRV_RESET_I2S_PIN);


  return GD_SUCCESS;
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
  // Error/warning/info messages
  const char msg_mute_in_off[] =            BRD_DRV_MUTE_IN_MUTE_OFF;
  const char msg_mute_in_on[] =             BRD_DRV_MUTE_IN_MUTE_ON;
  const char msg_mute_out_off[] =           BRD_DRV_MUTE_OUT_MUTE_OFF;
  const char msg_mute_out_on[] =            BRD_DRV_MUTE_OUT_MUTE_ON;

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
      brd_drv_send_msg(&msg_mute_in_off[0], 1, 0, -1);
    }
    else
    {
      // Mute on - set pin to low
      BRD_DRV_IO_LOW(SSC_TX_DATA);
      // Send message
      brd_drv_send_msg(&msg_mute_in_on[0], 1, 0, -1);
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
      brd_drv_send_msg(&msg_mute_out_off[0], 1, 0, -1);
    }
    else
    {
      // Mute on - set pin to low
      BRD_DRV_IO_LOW(SSC_TX_DATA);
      // Set signal
      BRD_DRV_IO_HIGH(BRD_DRV_MUTE_PIN);
      // Send message
      brd_drv_send_msg(&msg_mute_out_on[0], 1, 0, -1);
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
GD_RES_CODE brd_drv_set_rst_i2s_dir(e_brd_drv_dir_t e_rst_i2s_dir)
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

    s_brd_drv_rst_i2s.e_rst_i2s_dir = brd_drv_dir_in;
  }
  else if(e_rst_i2s_dir == brd_drv_dir_out)
  {// OUT
    // Clear RST_EN_A - disable RX mute
    BRD_DRV_IO_LOW(BRD_DRV_RST_EN_A_PIN);
    // RESET_I2S as output
    BRD_DRV_IO_LOW(BRD_DRV_RESET_I2S_PIN);
    // Set RST_EN_B - enable TX mute
    BRD_DRV_IO_HIGH(BRD_DRV_RST_EN_B_PIN);

    s_brd_drv_rst_i2s.e_rst_i2s_dir = brd_drv_dir_out;
  }
  else if(e_rst_i2s_dir == brd_drv_dir_hiz)
  {// Hi-Z
    // Clear RST_EN_B - disable TX mute
    BRD_DRV_IO_LOW(BRD_DRV_RST_EN_B_PIN);
    // RESET_I2S as input
    BRD_DRV_IO_AS_INPUT(BRD_DRV_RESET_I2S_PIN);
    // Clear RST_EN_A - enable RX mute data
    BRD_DRV_IO_LOW(BRD_DRV_RST_EN_A_PIN);

    s_brd_drv_rst_i2s.e_rst_i2s_dir = brd_drv_dir_hiz;
  }
  else
  {// Undefined
    return GD_INCORRECT_PARAMETER;
  }

  // Dfault value is 0
  s_brd_drv_rst_i2s.i_rst_i2s_val = 0;
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
GD_RES_CODE brd_drv_set_rst_i2s(uint8_t i_reset_i2s_flag)
{
  // Messages
  const char msg_reset_i2s_high[] = BRD_DRV_MSG_RESET_I2S_SET_TO_HIGH;
  const char msg_reset_i2s_low[]  = BRD_DRV_MSG_RESET_I2S_SET_TO_LOW;
  const char msg_reset_i2s_in_off[] = BRD_DRV_MSG_RESET_I2S_INPUT_OFF;
  const char msg_reset_i2s_in_on[] = BRD_DRV_MSG_RESET_I2S_INPUT_ON;


  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Check direction
  if((s_brd_drv_rst_i2s.e_rst_i2s_dir == brd_drv_dir_in) ||
     (s_brd_drv_rst_i2s.e_rst_i2s_dir == brd_drv_dir_hiz))
  {
    // Input or Hi-Z
    // Perform some activity only if set to 1
    if(i_reset_i2s_flag != 0)
    {
      brd_drv_send_msg(&msg_reset_i2s_in_on[0], 1, 0, -1);
      s_brd_drv_rst_i2s.i_rst_i2s_val = 1;
      // Perform operations
      brd_drv_reset_i2s();
      // Operation done
      s_brd_drv_rst_i2s.i_rst_i2s_val = 0;
    }
    else
    {
      // Set to 0 - turn off
      s_brd_drv_rst_i2s.i_rst_i2s_val = 0;
      brd_drv_send_msg(&msg_reset_i2s_in_off[0], 1, 0, -1);
    }
  }
  else if(s_brd_drv_rst_i2s.e_rst_i2s_dir == brd_drv_dir_out)
  {
    // Output
    // Just set RESET_I2S pin
    if(i_reset_i2s_flag == 0)
    {
      // Set RESET I2S signal to 0 - not needed reset I2S connector on AVR side
      BRD_DRV_IO_LOW(BRD_DRV_RESET_I2S_PIN);
      s_brd_drv_rst_i2s.i_rst_i2s_val = 0;
      brd_drv_send_msg(&msg_reset_i2s_low[0], 1, 0, -1);
    }
    else
    {
      // Reset flag = 1

      // Store status value
      GD_RES_CODE e_status;

      // Set RESET I2S signal to 1
      BRD_DRV_IO_HIGH(BRD_DRV_RESET_I2S_PIN);
      s_brd_drv_rst_i2s.i_rst_i2s_val = 1;

      // Keep RESET_I2S in HIGH little bit longer
      volatile uint32_t i_cnt = 0;
      while(i_cnt < 5000UL)
      {
        i_cnt++;
      }

      // Do reset I2S bus on AVR side
      e_status = brd_drv_reset_i2s();
      brd_drv_send_msg(&msg_reset_i2s_high[0], 1, 0, 0);

      // Anyway set RESET I2S to low
      /* Note: Already done by brd_drv_reset_i2s(), so it is commented, but
       * developer saw this code and will know what happens
       */
      //BRD_DRV_IO_LOW(BRD_DRV_RESET_I2S_PIN);
      s_brd_drv_rst_i2s.i_rst_i2s_val = 0;

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
  s_brd_drv_pure_i2s_dir.e_tx_data_dir = e_tx_data_dir;

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
  s_brd_drv_pure_i2s_dir.e_rx_data_dir = e_rx_data_dir;

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
    BRD_DRV_IO_AS_INPUT(GCLK0);
    BRD_DRV_IO_HIGH(BRD_DRV_MCLK_EN_A_PIN);
    break;
  case brd_drv_dir_out:
    BRD_DRV_IO_LOW(BRD_DRV_MCLK_EN_A_PIN);
    gpio_enable_module_pin(GCLK0, GCLK0_FUNCTION);
    BRD_DRV_IO_HIGH(BRD_DRV_MCLK_EN_B_PIN);
    break;
  case brd_drv_dir_hiz:
    /* Not easy to define "right" option. So disconnect signals from connector
     * side, but allow transmit signals, so codec will work
     */
    BRD_DRV_IO_LOW(BRD_DRV_MCLK_EN_A_PIN);
    BRD_DRV_IO_LOW(BRD_DRV_MCLK_EN_B_PIN);
    gpio_enable_module_pin(GCLK0, GCLK0_FUNCTION);
    break;
  default:
    return GD_INCORRECT_PARAMETER;
  }

  // If all OK, save state
  s_brd_drv_pure_i2s_dir.e_mclk_dir = e_mclk_dir;

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
  switch(e_bclk_dir)
  {
  case brd_drv_dir_in:
    BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_B_PIN);
    BRD_DRV_IO_AS_INPUT(GCLK1);
    BRD_DRV_IO_HIGH(BRD_DRV_BCLK_EN_A_PIN);
    break;
  case brd_drv_dir_out:
    BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_A_PIN);
    gpio_enable_module_pin(GCLK1, GCLK1_FUNCTION);
    BRD_DRV_IO_HIGH(BRD_DRV_BCLK_EN_B_PIN);
    break;
  case brd_drv_dir_hiz:
    BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_A_PIN);
    gpio_enable_module_pin(GCLK1, GCLK1_FUNCTION);
    BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_B_PIN);
    break;
  default:
    return GD_INCORRECT_PARAMETER;
  }

  // If all OK, save state
  s_brd_drv_pure_i2s_dir.e_bclk_dir = e_bclk_dir;

  return GD_SUCCESS;
}



/**
 * \brief Set direction of FRAME_SYNC
 * @param e_frame_sync_dir Options: 0 (input) ; 1 (output) ; 2 (Hi-Z)
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_frame_sync_dir(e_brd_drv_dir_t e_frame_sync_dir)
{
  // Store status
  GD_RES_CODE e_status;

  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  switch(e_frame_sync_dir)
  {
  case brd_drv_dir_in:
    BRD_DRV_IO_LOW(BRD_DRV_FS_EN_B_PIN);
    // SSC as slave. So TX module will not generate FRAME_SYNC
    e_status = ssc_set_FSYNC_role(SSC_ROLE_RX);
    if(e_status != GD_SUCCESS)
    {
      return e_status;
    }
    BRD_DRV_IO_HIGH(BRD_DRV_FS_EN_A_PIN);
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
      return GD_SUCCESS;
    }
    break;
  default:
    return GD_INCORRECT_PARAMETER;
  }

  s_brd_drv_pure_i2s_dir.e_frame_sync_dir = e_frame_sync_dir;

  return GD_SUCCESS;
}



inline GD_RES_CODE brd_drv_set_BCLK_div(uint16_t i_div)
{
  // When need to read from PM registers
  volatile avr32_pm_t *p_pm;
  p_pm = (avr32_pm_t*)BRD_DRV_PM_ADDRESS;

  // Check input values
  if(i_div == 0) return GD_INCORRECT_PARAMETER;
  if(i_div > 512) return GD_INCORRECT_PARAMETER;

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

inline GD_RES_CODE brd_drv_set_MCLK_div(uint16_t i_div)
{
  // When need to read from PM registers
  volatile avr32_pm_t *p_pm;
  p_pm = (avr32_pm_t*)BRD_DRV_PM_ADDRESS;

  // Check input values
  if(i_div == 0) return GD_INCORRECT_PARAMETER;
  if(i_div > 512) return GD_INCORRECT_PARAMETER;

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
  e_status = tlv320aic33_set_digital_interface_as_master(0);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Set data interface to I2S
  e_status = tlv320aic33_set_data_interface_mode(
      serial_data_bus_uses_I2S_mode);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Set word length to 32 bit
  e_status = tlv320aic33_set_word_length(24);
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

  // Show actual headphone volume settings
  return brd_drv_show_volume();
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
  // Error/warning/info messages
  const char msg_mute_btn_pressed[] =  BRD_DRV_MSG_MUTE_BTN_PRESSED;
  const char msg_mute_btn_released[] = BRD_DRV_MSG_MUTE_BTN_RELEASED;

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
    brd_drv_send_msg(&msg_mute_btn_pressed[0], 1, 0, -1);
    brd_drv_set_mute(1);
  }

  // Check for falling edge
  if((i_mute_btn == 0) && (i_mute_btn_previous != 0))
  {
    // Released button
    brd_drv_send_msg(&msg_mute_btn_released[0], 1, 0, -1);
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
  // Error/warning/info messages
  const char msg_mute_signal_rising[] =  BRD_DRV_MSG_MUTE_RISING_EDGE;
  const char msg_mute_signal_falling[] = BRD_DRV_MSG_MUTE_FALLING_EDGE;


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
    brd_drv_send_msg(&msg_mute_signal_rising[0], 1, 0, -1);
    // Mute on
    brd_drv_set_mute(1);
  }

  // Check falling edge
  if((i_mute == 0) && (i_mute_previous != 0))
  {
    brd_drv_send_msg(&msg_mute_signal_falling[0], 1, 0, -1);
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
  // Error/warning/info messages
  const char msg_rst_i2s_btn_pressed[] =  BRD_DRV_MSG_RST_I2S_BTN_PRESSED;
  const char msg_rst_i2s_btn_released[] = BRD_DRV_MSG_RST_I2S_BTN_RELEASED;
  const char msg_rst_i2s_err_cleared[] =  BRD_DRV_MSG_ERROR_CLEANED;

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
    brd_drv_send_msg(&msg_rst_i2s_btn_pressed[0], 1, 0, -1);
  }


  // Check for falling edge - released button
  if((i_rst_i2s_btn == 0) && (i_rst_i2s_btn_previous != 0))
  {
    // Falling edge
    brd_drv_send_msg(&msg_rst_i2s_btn_released[0], 1, 0, -1);

    if(i_time < BRD_DRV_SHORT_PRESS)
    {
      // Short press - just clean LCD
      LCD_5110_clear();
      brd_drv_draw_logo();
      brd_drv_show_volume();

      // Turn off ERROR_SIG LED
      BRD_DRV_IO_LOW(BRD_DRV_ERROR_SIG_PIN);

      brd_drv_send_msg(&msg_rst_i2s_err_cleared[0], 1, 0, -1);
    }// Short button press
    else
    {
      // Long button press
      // Reset I2S set to 0 (clear)
      brd_drv_set_rst_i2s(0);
      i_rst_i2s_already_set = 0;
    }// Long button press
    // Reset timer
    i_time = 0;
  }// Falling edge

  // Time > short? If yes, then perform reset
  if((i_time > BRD_DRV_SHORT_PRESS) && (i_rst_i2s_already_set == 0))
  {
    brd_drv_set_rst_i2s(1);
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
  // Error/warning/info messages
  const char msg_rst_i2s_signal_rising[] =  BRD_DRV_MSG_RST_I2S_RISING_EDGE;
  const char msg_rst_i2s_signal_falling[] = BRD_DRV_MSG_RST_I2S_FALLING_EDGE;

  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Previous value of RESET_I2S
  static uint8_t i_rst_i2s_previous = 0;

  // Actual RESET_I2S value
  uint8_t i_rst_i2s;

  // Check if pin is set as input
  if(s_brd_drv_rst_i2s.e_rst_i2s_dir == brd_drv_dir_in)
  {
    BRD_DRV_IO_READ(i_rst_i2s, BRD_DRV_RESET_I2S_PIN);

    // Check rising edge
    if((i_rst_i2s != 0) && (i_rst_i2s_previous == 0))
    {
      brd_drv_send_msg(&msg_rst_i2s_signal_rising[0], 1, 0, -1);
      // Reset on
      brd_drv_set_rst_i2s(1);
    }

    // Check falling edge
    if((i_rst_i2s == 0) && (i_rst_i2s_previous != 0))
    {
      brd_drv_send_msg(&msg_rst_i2s_signal_falling[0], 1, 0, -1);
      // Reset off
      brd_drv_set_rst_i2s(0);
    }

    // Save actual RESET_I2S value
    i_rst_i2s_previous = i_rst_i2s;
  }// Check if pin is set as input
}

