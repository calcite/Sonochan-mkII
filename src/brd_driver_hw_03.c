/**
 * \file
 *
 * \brief Board driver for "Sonochan mkII" HW version 03
 *
 * Allow set basic settings on board. Also support "generic driver"
 *
 * Created:  23.04.2014\n
 * Modified: 07.05.2014
 *
 * \version 0.1
 * \author  Martin Stejskal
 */

#include "brd_driver_hw_03.h"
//============================| Global variables |=============================
/**
 * \brief Mute structure
 *
 * Contains mute direction and mute value
 */
s_brd_drv_mute_t s_brd_drv_mute;

/**
 * \brief Reset I2S structure
 *
 * Contains reset I2S direction and value
 */
s_brd_drv_rst_i2s_t s_brd_drv_rst_i2s;

/**
 * \brief Store information if error occurred
 *
 * Values: 0 - not error  ; else error occurred
 *
 */
uint8_t i_error_occurred;
//=========================| Generic driver support |==========================
#if BRD_DRV_SUPPORT_GENERIC_DRIVER == 1
/**
 * \brief Configure table for device
 */

// Do not test architecture. This is mainly for AVR32.
const gd_config_struct BRD_DRV_config_table[] =
  {
    {
      0,                      // Command ID
      "Initialize board hardware",  // Name
      "Initialize I/O and prepare codec",      // Descriptor
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
      1,
      "Reset I2S",
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
      2,
      "MUTE direction",
      "0 - input ; 1 - output",
      uint32_type,      // Cause it is enum and 32 bit system
      {.data_uint32 = 0},
      {.data_uint32 = 1},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 1},
      (GD_DATA_VALUE*)&s_brd_drv_mute.e_mute_dir,
      brd_drv_set_mute_direction
    },
    {
      3,
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
      4,
      "Set RESET_I2S direction",
      "0 - input ; 1 - output",
      uint32_type,      // Cause it is enum and 32 bit system
      {.data_uint32 = 0},
      {.data_uint32 = 1},
      uint32_type,
      {.data_uint32 = 0},
      {.data_uint32 = 1},
      (GD_DATA_VALUE*)&s_brd_drv_rst_i2s.e_rst_i2s_dir,
      brd_drv_set_rst_i2s_direction
    },
    {
      5,
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

  };
/// \brief Maximum command ID (is defined by last command)
#define BRD_DRV_MAX_CMD_ID          5


const gd_metadata BRD_DRV_metadata =
{
        BRD_DRV_MAX_CMD_ID,              // Max CMD ID
        "Board driver for Sonochan mkII v0.1",     // Description
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
  brd_drv_set_mute_reset_i2s_pins_as_input();
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
     * just a garbage.
     */
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

  // Set all mute and reset_i2s pins as input (recommended default)
  brd_drv_set_mute_reset_i2s_pins_as_input();
  // Set isolators I/O (recommended default)
  brd_drv_set_isolators_to_HiZ();

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

  // Draw logo
  e_status = brd_drv_draw_logo();
  if(e_status != GD_SUCCESS)
  {
    const char msg_draw_logo_fail[] = BRD_DRV_MSG_DRAW_LOGO_FAIL;
    brd_drv_send_error_msg(&msg_draw_logo_fail[0], 1, 0);
    return e_status;
  }


  // Precise setting of PLL
  /* Turn of save setting when changing frequency by 1. We want continuous
   * clock at "any price". PLL CLK OUT should work as expected.
   */
  e_status = cs2200_set_save_change_ratio_by_1(0);
  if(e_status != GD_SUCCESS)
  {
    const char msg_set_pll_save_flag_when_change_by_1[] =
        BRD_DRV_MSG_PLL_SET_SAVE_FLAG_FAIL;
    brd_drv_send_error_msg(&msg_set_pll_save_flag_when_change_by_1[0], 1 ,1);
    return e_status;
  }



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



  // Prepare TLV (CS2200 have to be already initialized)
  e_status = brd_drv_TLV_default();
  if(e_status != GD_SUCCESS)
  {
    const char msg_codec_init_fail[] = BRD_DRV_MSG_CODEC_INIT_FAIL;
    brd_drv_send_error_msg(&msg_codec_init_fail[0], 1, 1);
    return e_status;
  }




  brd_drv_set_mute_direction(brd_drv_dir_in);
  brd_drv_set_rst_i2s_direction(brd_drv_dir_in);



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
GD_RES_CODE brd_drv_reset_i2s(void)
{
  const char msg_restart_i2s_done[] = BRD_DRV_MSG_RESET_I2S_DONE;

  ///\todo Complete this function. it need functions to set I/O for isolators

  brd_drv_set_isolators_to_HiZ();
  brd_drv_set_mute_reset_i2s_pins_as_input();
  brd_drv_TLV_default();

  // Send message to debug interface
  brd_drv_send_msg(&msg_restart_i2s_done[0], 1, 0, -1);

  return GD_SUCCESS;
}
//===========================| Mid level functions |===========================
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
    uint8_t i_write_to_DBG,
    uint8_t i_write_to_LCD,
    uint8_t i_LCD_line)
{
  // If want write to debug interface
  if(i_write_to_DBG != 0)
  {
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

  const char snchn_txt[] = BRD_DRV_MSG_SONOCHAN_MK_II;
  // Select line
  e_status = LCD_5110_set_line(BRD_DRV_LCD_LOGO_LINE);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  e_status = LCD_5110_write(&snchn_txt[0]);

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
    uint8_t i_write_to_DBG,
    uint8_t i_write_to_LCD)
{
  // Blink LCD backlight - turn off
  LCD_5110_backlight_off();

  if(i_write_to_DBG != 0)
  {
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
    uint8_t i_write_to_DBG,
    uint8_t i_write_to_LCD)
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
 * \brief Set pins, that control signal direction at digital isolators, to low
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_set_isolators_to_HiZ(void)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;
  // Set output enables to low -> disable them by default
  BRD_DRV_IO_LOW(BRD_DRV_MUTE_EN_A_PIN);
  BRD_DRV_IO_LOW(BRD_DRV_RST_EN_A_PIN);
  BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_A_PIN);
  BRD_DRV_IO_LOW(BRD_DRV_FS_EN_A_PIN);
  BRD_DRV_IO_LOW(BRD_DRV_MCLK_EN_A_PIN);
  BRD_DRV_IO_LOW(BRD_DRV_MUTE_EN_B_PIN);
  BRD_DRV_IO_LOW(BRD_DRV_RST_EN_B_PIN);
  BRD_DRV_IO_LOW(BRD_DRV_TX_EN_B_PIN);
  BRD_DRV_IO_LOW(BRD_DRV_MCLK_EN_B_PIN);
  BRD_DRV_IO_LOW(BRD_DRV_BCLK_EN_B_PIN);
  BRD_DRV_IO_LOW(BRD_DRV_FS_EN_B_PIN);

  return GD_SUCCESS;
}

/**
 * \brief Just set all mute and reset_i2s pins as input
 *
 * Set mute, mute_btn, reset_i2s, reset_i2s_btn pins as input.
 *
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_set_mute_reset_i2s_pins_as_input(void)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Button pins
  BRD_DRV_IO_AS_INPUT(BRD_DRV_MUTE_BTN_PIN);
  BRD_DRV_IO_AS_INPUT(BRD_DRV_RESET_I2S_BTN_PIN);

  // Signal pins
  BRD_DRV_IO_AS_INPUT(BRD_DRV_MUTE_PIN);
  BRD_DRV_IO_AS_INPUT(BRD_DRV_RESET_I2S_PIN);

  // Set values in mute and reset I2S structures
  s_brd_drv_mute.e_mute_dir = brd_drv_dir_in;
  s_brd_drv_mute.i_mute_val = 0;        // brd_drv_task() rewrite if needed

  s_brd_drv_rst_i2s.e_rst_i2s_dir = brd_drv_dir_in;
  s_brd_drv_rst_i2s.i_rst_i2s_val = 0;  // brd_drv_task() rewrite if needed

  return GD_SUCCESS;
}


/**
 * \brief Allow set MUTE pin direction
 * @param e_mute_dir Options: 0 (input) ; 1 (output)
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_mute_direction(e_brd_drv_dir_t e_mute_dir)
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
  else
  {// OUT
    // Clear MUTE_EN_A - disable RX mute
    BRD_DRV_IO_LOW(BRD_DRV_MUTE_EN_A_PIN);
    // MUTE as output
    BRD_DRV_IO_LOW(BRD_DRV_MUTE_PIN);
    // Set MUTE_EN_B - enable TX mute
    BRD_DRV_IO_HIGH(BRD_DRV_MUTE_EN_B_PIN);

    s_brd_drv_mute.e_mute_dir = brd_drv_dir_out;
  }

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

  if(s_brd_drv_mute.e_mute_dir == brd_drv_dir_in)
  {
    // Input direction
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
  else
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

  // Save value
  s_brd_drv_mute.i_mute_val = i_mute_flag;

  return GD_SUCCESS;
}



/**
 * \brief Allow set RESET_I2S pin direction
 * @param e_rst_i2s_dir Options: 0 (input) ; 1 (output)
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE brd_drv_set_rst_i2s_direction(e_brd_drv_dir_t e_rst_i2s_dir)
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
  else
  {// OUT
    // Clear RST_EN_A - disable RX mute
    BRD_DRV_IO_LOW(BRD_DRV_RST_EN_A_PIN);
    // RESET_I2S as output
    BRD_DRV_IO_LOW(BRD_DRV_RESET_I2S_PIN);
    // Set RST_EN_B - enable TX mute
    BRD_DRV_IO_HIGH(BRD_DRV_RST_EN_B_PIN);

    s_brd_drv_rst_i2s.e_rst_i2s_dir = brd_drv_dir_out;
  }

  return GD_SUCCESS;
}



/**
 * \brief Set RESET_I2S pin if configured as output.
 *
 * If signal RESET_I2S is set as output, then just set signal RESET_I2S.\n
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
  if(s_brd_drv_rst_i2s.e_rst_i2s_dir == brd_drv_dir_in)
  {
    // Input
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
  else
  {
    // Output
    // Just set RESET_I2S pin
    if(i_reset_i2s_flag == 0)
    {
      BRD_DRV_IO_LOW(BRD_DRV_RESET_I2S_PIN);
      s_brd_drv_rst_i2s.i_rst_i2s_val = 0;
      brd_drv_send_msg(&msg_reset_i2s_low[0], 1, 0, -1);
    }
    else
    {
      BRD_DRV_IO_HIGH(BRD_DRV_RESET_I2S_PIN);
      s_brd_drv_rst_i2s.i_rst_i2s_val = 1;
      brd_drv_send_msg(&msg_reset_i2s_high[0], 1, 0, 0);
    }
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
  e_status = tlv320aic33_set_word_length(32);
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
      driver_power_on_time_10ms);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  // Set ramp-up step timing
  e_status = tlv320aic33_set_driver_ramp_up_step_time(
      driver_ramp_up_step_time_1ms);
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

