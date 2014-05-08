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
    }
  };
/// \brief Maximum command ID (is defined by last command)
#define BRD_DRV_MAX_CMD_ID          0


const gd_metadata BRD_DRV_metadata =
{
        BRD_DRV_MAX_CMD_ID,              // Max CMD ID
        "Board driver for Sonochan mkII v0.1",     // Description
        (gd_config_struct*)&BRD_DRV_config_table[0],
        0x0F    // Serial number (0~255)
};
#endif

//====================| Function prototypes not for user |=====================

GD_RES_CODE brd_drv_draw_logo(void);


GD_RES_CODE brd_drv_TLV_default(void);


GD_RES_CODE brd_drv_adc_init(void);




void brd_drv_set_mute_reset_i2s_pins_as_input(void);

void brd_drv_send_msg(
    const char * p_msg,
    uint8_t i_write_to_DBG,
    uint8_t i_write_to_LCD,
    uint8_t i_LCD_line);

void brd_drv_send_error_msg(
    const char * p_msg,
    uint8_t i_write_to_DBG,
    uint8_t i_write_to_LCD);
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
  e_status = brd_drv_set_isolators_to_HiZ();
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

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

  // Set all mute and reset_i2s pins as input (default)
  brd_drv_set_mute_reset_i2s_pins_as_input();

  // Initialize LCD
  e_status = LCD_5110_init();
  if(e_status != GD_SUCCESS)
  {
    const char msg_lcd_init_fail[] = BRD_DRV_MSG_LCD_INIT_FAIL;
    // Send message over debug interface
    brd_drv_send_error_msg(&msg_lcd_init_fail[0], 1, 0);
    return e_status;
  }

  // Draw logo
  e_status = brd_drv_draw_logo();
  if(e_status != GD_SUCCESS)
  {
    const char msg_draw_logo_fail[] = BRD_DRV_MSG_DRAW_LOGO_FAIL;
    brd_drv_send_error_msg(&msg_draw_logo_fail[0], 1, 0);
    return e_status;
  }

  // Set isolators I/O
  e_status = brd_drv_set_isolators_to_HiZ();
  if(e_status != GD_SUCCESS)
  {
    const char msg_set_isolators_failed[] = BRD_DRV_MSG_ISOLATOR_FAIL;
    // Send message over debug interface and LCD
    brd_drv_send_error_msg(&msg_set_isolators_failed[0], 1, 1);
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



  // Show actual headphone volume settings
  // Volume as float value
  float f_volume;
  // Temporary string for text
  char c[20];
  tlv320aic33_get_headphones_volume_db(&f_volume);
  sprintf(&c[0], "Vol: %.1f dB\n", f_volume);
  brd_drv_send_msg(&c[0], 0, 1, BRD_DRV_LCD_HP_VOLUME_LINE);


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
  // Error messages
  const char msg_tlv_failed_set_headphone_volume_dB[] =
      BRD_DRV_TLV_FAILED_SET_HEADPHONE_VOL_DB;


  // For saving status
  GD_RES_CODE e_status;

  // Simple time counter
  static uint32_t i_time_cnt = 0;

  // Store ADC volume value
  static uint16_t  i_saved_volume_value = 0;

  // Variable for store actual volume value
  uint16_t i_actual_volume;

  // Variable for store voltage on connector side
  uint16_t i_voltage_connector_side;

  // Pointer to ADC structure
  volatile avr32_adc_t *p_adc;
  p_adc = (avr32_adc_t*)BDR_DRV_ADC_ADDRESS;

  // For sprintf(). LCD capable show 84 Bytes
  char c[84];

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
        // Get real volume value - do not need check error code.
        tlv320aic33_get_headphones_volume_db(&f_volume);
        sprintf(&c[0], "Vol: %.1f dB\n", f_volume);
        brd_drv_send_msg(&c[0], 1, 1, BRD_DRV_LCD_HP_VOLUME_LINE);
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
        // Get real volume value - do not need check error code.
        tlv320aic33_get_headphones_volume_db(&f_volume);
        sprintf(&c[0], "Vol: %.1f dB\n", f_volume);
        brd_drv_send_msg(&c[0], 1, 1, BRD_DRV_LCD_HP_VOLUME_LINE);
      }
    }// Test if volume value has been changed




  }/* Check if all ADC conversions are done and if there is "time" for some
    * data processing
    */


  // Increase counter (always)
  i_time_cnt++;
}


//===========================| Mid level functions |===========================


//===========================| Low level functions |===========================
/**
 * \brief Set pins, that control signal direction at digital isolators, to low
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE brd_drv_set_isolators_to_HiZ(void)
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


//=========================| Functions not for user |==========================
/**
 * \brief Draw Sonochan mk II logo on display
 *
 * Function assume, that LCD_init() od LCD_clear() was called
 *
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE brd_drv_draw_logo(void)
{
  // For return result code
  GD_RES_CODE e_status;

  const char snchn_txt[] = BRD_DRV_MSG_SONOCHAN_MK_II;
  e_status = LCD_5110_write(&snchn_txt[0]);

  return e_status;
}


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
 * \brief Just set all mute and reset_i2s pins as input
 *
 * Set mute, mute_btn, reset_i2s, reset_i2s_btn pins as input.
 */
inline void brd_drv_set_mute_reset_i2s_pins_as_input(void)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;

  // Button pins
  BRD_DRV_IO_AS_INPUT(BRD_DRV_MUTE_BTN_PIN);
  BRD_DRV_IO_AS_INPUT(BRD_DRV_RESET_I2S_BTN_PIN);

  // Signal pins
  BRD_DRV_IO_AS_INPUT(BRD_DRV_MUTE_PIN);
  BRD_DRV_IO_AS_INPUT(BRD_DRV_RESET_I2S_PIN);
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
      // Select line
      LCD_5110_set_line(i_LCD_line);
      // Clear line
      LCD_5110_write_to_line(0x00);
      // Write message
      LCD_5110_write(p_msg);
    }
    else
    {
      LCD_5110_write(p_msg);
    }
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
    uint8_t i_write_to_DBG,
    uint8_t i_write_to_LCD)
{
  // Pointer to GPIO memory
  volatile avr32_gpio_port_t *gpio_port;
  // Turn-ON ERROR LED
  BRD_DRV_IO_HIGH(BRD_DRV_ERROR_INDICATION_PIN);

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
