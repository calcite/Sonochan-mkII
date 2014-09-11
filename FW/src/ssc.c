/**
 * \file
 *
 * \brief Driver for SSC module on UC3A3256
 *
 * Assume that RX module is ALWAYS set as slave and TX module is configured\n
 * as master or even slave. Also assume, that some pins are wired:\n
 * (BCLK)  SSC-RX_CLOCK      <-----> SSC-TX_CLOCK\n
 * (FSYNC) SSC-RX_FRAME_SYNC <-----> SSC-TX_FRAME_SYNC\n
 * \n
 * Created:  20.08.2014\n
 * Modified: 09.09.2014
 *
 * \version 0.2
 * \author Martin Stejskal
 */
#include "ssc.h"
//============================| Global variables |=============================
/**
 * @brief Structure that keep actual settings in memory
 */
static s_ssc_settings_t s_ssc_settings;

/**
 * @brief Pin mapping for SSC module
 */
static const gpio_map_t SSC_GPIO_MAP = {
  {SSC_RX_CLOCK, SSC_RX_CLOCK_FUNCTION},
  {SSC_RX_DATA, SSC_RX_DATA_FUNCTION},
  {SSC_RX_FRAME_SYNC, SSC_RX_FRAME_SYNC_FUNCTION},
  {SSC_TX_CLOCK, SSC_TX_CLOCK_FUNCTION},
  {SSC_TX_DATA, SSC_TX_DATA_FUNCTION},
  {SSC_TX_FRAME_SYNC, SSC_TX_FRAME_SYNC_FUNCTION}
};
//================================| Functions |================================
//==========================| High level functions |===========================
/**
 * @brief Preset SSC unit to default setting
 * @return SSC_SUCCESS (0) if all right
 */
SSC_RES_CODE ssc_init(void)
{
  // For storing status codes
  SSC_RES_CODE e_status;

  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;

  // Write info message
  SSC_INFO_FUNC(SSC_MSG_INFO_SSC_INIT);

  //==============================| Set I/O pins |=============================
  // Assign GPIO to SSC.
  gpio_enable_module(SSC_GPIO_MAP,
                     sizeof(SSC_GPIO_MAP) / sizeof(SSC_GPIO_MAP[0]));

  gpio_enable_pin_glitch_filter(SSC_RX_CLOCK);
  gpio_enable_pin_glitch_filter(SSC_RX_DATA);
  gpio_enable_pin_glitch_filter(SSC_RX_FRAME_SYNC);
  gpio_enable_pin_glitch_filter(SSC_TX_CLOCK);
  gpio_enable_pin_glitch_filter(SSC_TX_DATA);
  gpio_enable_pin_glitch_filter(SSC_TX_FRAME_SYNC);

  //===============================| Reset codec |=============================
  // First, we must reset codec
  e_status = ssc_reset();
  if(e_status != SSC_SUCCESS)
  {
    SSC_ERR_FUNC(SSC_MSG_ERR_RESET_FAILED);
    return e_status;
  }

  //===========================| Set default values |==========================
  // Default data length (in bits)
  s_ssc_settings.i_data_length = SSC_DEFAULT_DATA_LENGTH;
  // Default frame length (in bits)
  s_ssc_settings.i_frame_length = SSC_DEFAULT_FRAME_LENGTH;
  // Preset default values (because some function need to have set these)
  s_ssc_settings.e_dig_aud_mode = SSC_DEFAULT_DIGITAL_AUDIO_INTERFACE;
  s_ssc_settings.e_FSYNC_RX_edge = SSC_DEFAULT_FSYNC_RX_EDGE;
  s_ssc_settings.e_FSYNC_TX_edge = SSC_DEFAULT_FSYNC_TX_EDGE;
  s_ssc_settings.e_FSYNC_role = SSC_DEFAULT_FSYNC_ROLE;
  // Sample data on rising edge (standard)
  s_ssc_settings.e_BCLK_RX_edge = SSC_DEFAULT_BCLK_RX_EDGE;
  // Transmit data on falling edge (standard)
  s_ssc_settings.e_BCLK_TX_edge = SSC_DEFAULT_BCLK_TX_EDGE;

  /* Always get BCLK from "outside world", no matter in which mode operate.
   * RX module work always as slave.
   */
  // No divider active
  p_ssc->CMR.div = AVR32_SSC_CMR_DIV_NOT_ACTIVE;

  // Transmit clock Selection - use signal from TX_CLOCK
  p_ssc->TCMR.cks = AVR32_SSC_TCMR_CKS_TK_PIN;
  // Receive clock selection - RX_CLOCK pin
  p_ssc->RCMR.cks = AVR32_SSC_RCMR_CKS_RK_PIN;

  // Transmit clock Output mode selection - input only (not generate BCLK)
  p_ssc->TCMR.cko = AVR32_SSC_TCMR_CKO_INPUT_ONLY;
  // Receive clock output mode selection - RX_CLOCK as input
  p_ssc->RCMR.cko = AVR32_SSC_RCMR_CKO_INPUT_ONLY;

  // Transmit clock gating - None, continuous clock
  p_ssc->TCMR.ckg = AVR32_SSC_TCMR_CKG_NONE;
    // Receive clock gating selection
  p_ssc->RCMR.ckg = AVR32_SSC_RCMR_CKG_NONE;
  // When no data, default value 0
  p_ssc->TFMR.datdef = 0;

  // RX Frame sync as input only
  p_ssc->RFMR.fsos = AVR32_SSC_RFMR_FSOS_INPUT_ONLY;

  // Digital interface (also enable TX and RX module)
  e_status = ssc_set_digital_interface_mode(
                                        SSC_DEFAULT_DIGITAL_AUDIO_INTERFACE);
  if(e_status != SSC_SUCCESS)
  {
    SSC_ERR_FUNC(SSC_MSG_ERR_SET_DIG_INTFCE_MODE);
    return e_status;
  }


  // And enable SSC module
  e_status = ssc_enable_RX_TX();
  if(e_status != SSC_SUCCESS)
  {
    SSC_ERR_FUNC(SSC_MSG_ERR_SSC_ENABLE_FAILED);
    return e_status;
  }

  return e_status;
}


/**
 * \brief Reset SSC module
 * @return SSC_SUCCESS (0) if all OK
 */
SSC_RES_CODE ssc_reset(void)
{
  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  // Set pointer
  p_ssc = SSC_DEVICE;

  // Perform software reset
  p_ssc->CR.swrst = 1;

  // No problem.....
  return SSC_SUCCESS;
}



/**
 * @brief Set one of supported digital interfaces
 * @param e_mode Options: SSC_I2S, SSC_DSP, SSC_LEFT_JUSTIFIED or\n
 *  SSC_RIGHT_JUSTIFIED
 * @return SSC_SUCCESS (0) if all OK
 */
SSC_RES_CODE ssc_set_digital_interface_mode(
    e_ssc_digital_audio_interface_t e_mode)
{
  // Store status
  SSC_RES_CODE e_status = SSC_FAIL;

  ///\todo REMOVE - NOW only for debug
  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;


  // Mode specific settings
  ///\ todo complete this function
  switch(e_mode)
  {
  case SSC_I2S:
    SSC_INFO_FUNC(SSC_MSG_INFO_MODE_I2S);
    e_status =  ssc_set_digital_interface_mode_I2S();
    break; // SSC_I2S
  case SSC_DSP:
    SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);

    if(s_ssc_settings.e_FSYNC_TX_edge == SSC_EDGE_FALLING)
    {
      p_ssc->TCMR.start = AVR32_SSC_TCMR_START_DETECT_FALLING_TF;
    }
    else if(s_ssc_settings.e_FSYNC_TX_edge == SSC_EDGE_RISING)
    {
      p_ssc->TCMR.start = AVR32_SSC_TCMR_START_DETECT_RISING_TF;
    }
    else
    {
      // Undefined state
      // Reset mode to default
      s_ssc_settings.e_dig_aud_mode = SSC_DEFAULT_DIGITAL_AUDIO_INTERFACE;
      return SSC_FAIL;
    }

    break;
  case SSC_LEFT_JUSTIFIED:
    SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);
    break;
  case SSC_RIGHT_JUSTIFIED:
    SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);
    break;
  // Else mode is unknown
  default:
    return SSC_INCORRECT_PARAMETER;
  }


  return e_status;
}


/**
 * @brief Get current setting of digital interface
 * @param p_e_mode Pointer to memory, where result will be written.\n
 * Options: SSC_I2S, SSC_DSP, SSC_LEFT_JUSTIFIED or SSC_RIGHT_JUSTIFIED
 * @return SSC_SUCCESS (0) if all OK
 */
inline SSC_RES_CODE ssc_get_digital_interface_mode(
    e_ssc_digital_audio_interface_t *p_e_mode)
{
  // Just save value to correct address
  *p_e_mode = s_ssc_settings.e_dig_aud_mode;

  return SSC_SUCCESS;
}



//===========================| Mid level functions |===========================

/* [Martin] It looks like this function is not being used in future, because of
 * PDCA synchronization (synchronize once, then do not need SSC -> changes not
 * applied immediately)
 */
/**
 * @brief Wait until correct FSYNC edge is detected
 * @return SSC_SUCCESS (0) if all OK
 */
inline SSC_RES_CODE ssc_wait_for_FSYNC_RX(void)
{
  switch(s_ssc_settings.e_dig_aud_mode)
  {
  case SSC_I2S:
    if((s_ssc_settings.e_FSYNC_RX_edge == SSC_EDGE_FALLING) ||
       (s_ssc_settings.e_FSYNC_RX_edge == SSC_EDGE_DEFAULT))
    {// Falling edge
      while (!gpio_get_pin_value(SSC_EXTERNAL_FSYNC_DETECTION_PIN));
      while (gpio_get_pin_value(SSC_EXTERNAL_FSYNC_DETECTION_PIN));
    }
    else if(s_ssc_settings.e_FSYNC_RX_edge == SSC_EDGE_RISING)
    {// Rising edge
      while (gpio_get_pin_value(SSC_EXTERNAL_FSYNC_DETECTION_PIN));
      while (!gpio_get_pin_value(SSC_EXTERNAL_FSYNC_DETECTION_PIN));
    }
    break;  // SSC_I2S
  case SSC_DSP:
    SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);
    ///\todo Complete
    break;
  case SSC_LEFT_JUSTIFIED:
    SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);
    ///\todo Complete
    break;
  case SSC_RIGHT_JUSTIFIED:
    SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);
    ///\todo Complete
    break;
  // Else mode is unknown
  default:
    return SSC_INCORRECT_PARAMETER;
  }
  return SSC_SUCCESS;
}






/**
 * @brief Set SSC module for I2S digital interface
 * Also preset all necessary variables to default value, except e_FSYNC_role\n
 * which should be set by higher layer.
 * @return SSC_SUCCESS (0) if all OK
 */
inline SSC_RES_CODE ssc_set_digital_interface_mode_I2S(void)
{
  // Store status codes
  SSC_RES_CODE e_status;

  // Backup actual mode value
  e_ssc_digital_audio_interface_t e_mode_backup =
      s_ssc_settings.e_dig_aud_mode;

  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;


  // Start TX on any FSYNC change
  p_ssc->TCMR.start = AVR32_SSC_TCMR_START_DETECT_LEVEL_CHANGE_TF;
  // Transmit transmit delay
  p_ssc->TCMR.sttdly = 0;
  // Set frame length
  p_ssc->TCMR.period = s_ssc_settings.i_frame_length -1;
  // Set data length (one sample per one channel)
  p_ssc->TFMR.datlen = s_ssc_settings.i_data_length -1;
  // MSB first - true
  p_ssc->TFMR.msbf = 1;
  // Transfer two words (left+right). Calculation is: DATNB+1, so there is 0
  p_ssc->TFMR.datnb = 0;
  // Set frame size (Frame sync is entire left channel)
  p_ssc->TFMR.fslen = (s_ssc_settings.i_frame_length -1)
      & ((1<<AVR32_SSC_TFMR_FSLEN_SIZE) -1);
  p_ssc->TFMR.fslenhi = (s_ssc_settings.i_frame_length -1)
      >>AVR32_SSC_TFMR_FSLEN_SIZE;
  // Frame sync data enable (Do not use transmit frame sync data)
  p_ssc->TFMR.fsden = 1;
  // Looks like FSYNC edge, but it is not
  p_ssc->TFMR.fsedge = 1;

  // Receive start selection - start on any edge
  p_ssc->RCMR.start = AVR32_SSC_RCMR_START_DETECT_LEVEL_CHANGE_RF;
  // Receive start delay - I2S data start 0 SCKL (immediately on FSYNC)
  p_ssc->RCMR.sttdly = 0;
  // Period offset
  p_ssc->RCMR.period = s_ssc_settings.i_frame_length -1;

  // Data length
  p_ssc->RFMR.datlen = s_ssc_settings.i_data_length -1;
  // MSB first - true
  p_ssc->RFMR.msbf = 1;
  // Transfer two words (left+right). Calculation is: DATNB+1, so there is 0
  p_ssc->RFMR.datnb = (1 - 1);
  // Set frame size (Frame sync is entire left channel)
  p_ssc->RFMR.fslen = (s_ssc_settings.i_frame_length -1) &
      ((1<<AVR32_SSC_TFMR_FSLEN_SIZE) -1);
  p_ssc->RFMR.fslenhi = (s_ssc_settings.i_frame_length -1)
      >>AVR32_SSC_TFMR_FSLEN_SIZE;
  // Negative edge detection
  p_ssc->RFMR.fsedge = 1;


  /* Following functions need to know which mode is active. So we set as
   * active I2S_SSC and if some problem occurs, then we load back backup.
   */
  s_ssc_settings.e_dig_aud_mode = SSC_I2S;
  // Set BCLK TX edge
  e_status = ssc_set_BCLK_TX_edge(SSC_EDGE_DEFAULT);
  if(e_status != SSC_SUCCESS)
  {
    s_ssc_settings.e_dig_aud_mode = e_mode_backup;
    return e_status;
  }

  // Set last set role (this is the "best way")
  // FSYNC TX edge only if FSYNC role is output - in FSYNC_role() function
  e_status = ssc_set_FSYNC_role(s_ssc_settings.e_FSYNC_role);
  if(e_status != SSC_SUCCESS)
  {
    s_ssc_settings.e_dig_aud_mode = e_mode_backup;
    return e_status;
  }

  // RX FSYNC edge
  e_status = ssc_set_FSYNC_RX_edge(SSC_EDGE_DEFAULT);
  if(e_status != SSC_SUCCESS)
  {
    s_ssc_settings.e_dig_aud_mode = e_mode_backup;
    return e_status;
  }
  // RX BCLK edge
  e_status = ssc_set_BCLK_RX_edge(SSC_EDGE_DEFAULT);
  if(e_status != SSC_SUCCESS)
  {
    s_ssc_settings.e_dig_aud_mode = e_mode_backup;
    return e_status;
  }

  return SSC_SUCCESS;
}

//===========================| Low level functions |===========================


/**
 * @brief Set length of data word
 * @param i_data_length Data length in bits
 * @return SSC_SUCCESS (0) if all OK
 */
SSC_RES_CODE ssc_set_data_length(uint8_t i_data_length)
{
  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;


  // Check input parameter
  if(i_data_length > 32)
  {
    return SSC_INCORRECT_PARAMETER;
  }

  // Switch according actual interface
  //[Martin] Not sure if different at all modes, but just for case....
  switch(s_ssc_settings.e_dig_aud_mode)
  {
  case SSC_I2S:
    p_ssc->TFMR.datlen = i_data_length -1;
    p_ssc->RFMR.datlen = i_data_length -1;
    break;
  case SSC_DSP:
    ///\todo Complete
    SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);
    break;
  case SSC_LEFT_JUSTIFIED:
    ///\todo Complete
    SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);
    break;
  case SSC_RIGHT_JUSTIFIED:
    ///\todo Complete
    SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);
    break;
  // Undefined/not supported mode
  default:
    return SSC_FAIL;
  }

  // And save value
  s_ssc_settings.i_data_length = i_data_length;

  // OK, everything looks OK
  return SSC_SUCCESS;
}

/**
 * @brief Give length of data word
 * @param p_i_data_length Memory address, where result will be written
 * @return SSC_SUCCESS (0) if all right
 */
SSC_RES_CODE ssc_get_data_length(uint8_t *p_i_data_length)
{
  *p_i_data_length = s_ssc_settings.i_data_length;
  return SSC_SUCCESS;
}



/**
 * @brief Set length of frame sync
 * @param i_frame_length Frame sync length in bits
 * @return SSC_SUCCESS (0) if all right
 */
SSC_RES_CODE ssc_set_frame_length(uint8_t i_frame_length)
{
  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;

  // Switch according to actual interface
  switch(s_ssc_settings.e_dig_aud_mode)
  {
  case SSC_I2S:
    // TX module
    p_ssc->TCMR.period = i_frame_length -1;
    p_ssc->TFMR.fslen = (i_frame_length -1)
        & ((1<<AVR32_SSC_TFMR_FSLEN_SIZE) -1);
    p_ssc->TFMR.fslenhi = (i_frame_length -1) >>AVR32_SSC_TFMR_FSLEN_SIZE;
    // RX module
    p_ssc->RCMR.period = i_frame_length -1;
    p_ssc->RFMR.fslen = (i_frame_length -1)
        & ((1<<AVR32_SSC_TFMR_FSLEN_SIZE) -1);
    p_ssc->RFMR.fslenhi = (i_frame_length -1) >>AVR32_SSC_TFMR_FSLEN_SIZE;
    break;
  case SSC_DSP:
    break;
  case SSC_LEFT_JUSTIFIED:
    break;
  case SSC_RIGHT_JUSTIFIED:
    break;
  // Undefined/not supported mode
  default:
    return SSC_FAIL;
  }

  // Everything look fine, so save setting
  s_ssc_settings.i_frame_length = i_frame_length;

  return SSC_SUCCESS;
}


/**
 * @brief Give length of data word
 * @param p_i_frame_length Memory address, where result will be written
 * @return SSC_SUCCESS (0) if all right
 */
SSC_RES_CODE ssc_get_frame_length(uint8_t *p_i_frame_length)
{
  *p_i_frame_length = s_ssc_settings.i_frame_length;
  return SSC_SUCCESS;
}



/* [Martin] It looks like this function is not being used in future, because of
 * PDCA synchronization (synchronize once, then do not need SSC -> changes not
 * applied immediately)
 */
/**
 * @brief Set on which edge will RX module synchronized
 *
 * For synchronization must be used function ssc_wait_for_FSYNC_RX() which\n
 * return when correct FSYNC edge was detected.
 *
 * @param e_edge Options: SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 * @return SSC_SUCCESS (0) if all OK
 */
inline SSC_RES_CODE ssc_set_FSYNC_RX_edge(e_ssc_edge_t e_edge)
{
  // Check input parameter
  if( (e_edge != SSC_EDGE_FALLING) &&
      (e_edge != SSC_EDGE_RISING) &&
      (e_edge != SSC_EDGE_DEFAULT))
  {
    return SSC_INCORRECT_PARAMETER;
  }

  // Else is all OK -> save value
  s_ssc_settings.e_FSYNC_RX_edge = e_edge;

  return SSC_SUCCESS;
}


/**
 * @brief Get information on which edge is RX module synchronized
 * @param p_e_edge Address, where result will be written.\n
 * Options: SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 *
 * @return SSC_SUCCESS (0) if all OK
 */
inline SSC_RES_CODE ssc_get_FSYNC_RX_edge(e_ssc_edge_t *p_e_edge)
{
  // Just save value to correct address
  *p_e_edge = s_ssc_settings.e_FSYNC_RX_edge;
  return SSC_SUCCESS;
}




/* [Martin] It looks like this function is not being used in future, because of
 * PDCA synchronization (synchronize once, then do not need SSC -> changes not
 * applied immediately)
 */
/**
 * @brief Set FSYNC synchronization edge for TX module
 * Edge can be set "on the fly", but when is used PDCA and specific type\n
 * synchronization, change could not be applied from user point of view.
 *
 * @param e_edge Options: SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 * @return SSC_SUCCESS (0) if all OK
 */
inline SSC_RES_CODE ssc_set_FSYNC_TX_edge(e_ssc_edge_t e_edge)
{
  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;


  switch(e_edge)
  {
  case SSC_EDGE_FALLING:
    p_ssc->TFMR.fsos = AVR32_SSC_TFMR_FSOS_NEG_PULSE;
    break;
  case SSC_EDGE_RISING:
    p_ssc->TFMR.fsos = AVR32_SSC_TFMR_FSOS_POS_PULSE;
    break;
  case SSC_EDGE_DEFAULT:
    // This is mode dependent
    switch(s_ssc_settings.e_dig_aud_mode)
    {
    case SSC_I2S:
      p_ssc->TFMR.fsos = AVR32_SSC_TFMR_FSOS_NEG_PULSE;
      break;
    case SSC_DSP:
      SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);
      ///\todo Complete
      break;
    case SSC_LEFT_JUSTIFIED:
      SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);
      ///\todo Complete
      break;
    case SSC_RIGHT_JUSTIFIED:
      SSC_ERR_FUNC(SSC_MSG_ERR_FEATURE_NOT_IMPLEMENTED);
      ///\todo Complete
      break;
    // Default case should never happen
    default:
      return SSC_FAIL;
    }
    break;
  // Unknown case -> incorrect parameter
  default:
    return SSC_INCORRECT_PARAMETER;
  }


  // Else is all OK -> save value
  s_ssc_settings.e_FSYNC_TX_edge = e_edge;
  return SSC_SUCCESS;
}

/**
 * @brief Get information on which edge is TX module synchronized
 * @param p_e_edge Address, where result will be written.\n
 * Options: SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 *
 * @return SSC_SUCCESS (0) if all OK
 */
inline SSC_RES_CODE ssc_get_FSYNC_TX_edge(e_ssc_edge_t *p_e_edge)
{
  // Just save value to correct address
  *p_e_edge = s_ssc_settings.e_FSYNC_TX_edge;
  return SSC_SUCCESS;
}





/**
 * @brief Set FSYNC role (master, slave)
 * In master mode FSYNC is generated by TX module. In slave mode, even TX\n
 * module just receive FSYNC and it is up to external hardware to supply\n
 * correct FSYNC.
 *
 * @param e_role Options: SSC_RX (slave mode), SSC_TX (master mode)
 * @return SSC_SUCCESS (0) if all OK
 */
SSC_RES_CODE ssc_set_FSYNC_role(e_ssc_Role_Rx_Tx_t e_role)
{
  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;

  switch(e_role)
  {
    // Slave (receive FSYNC)
    case SSC_ROLE_RX:
      p_ssc->TFMR.fsos = AVR32_SSC_TFMR_FSOS_INPUT_ONLY;
      break;
    // Master (generate FSYNC with default edge)
    case SSC_ROLE_TX:
      ssc_set_FSYNC_TX_edge(SSC_EDGE_DEFAULT);
      break;
    // Unknown parameter -> fail
    default:
      return SSC_INCORRECT_PARAMETER;
  }

  // If all OK -> save variable
  s_ssc_settings.e_FSYNC_role = e_role;

  // Everything look OK
  return SSC_SUCCESS;
}


/**
 * @brief Get FSYNC role
 *
 * @param p_e_role Address, where result will be written.\n
 * Options: SSC_RX (slave mode), SSC_TX (master mode)
 * @return SSC_SUCCESS (0) if all OK
 */
inline SSC_RES_CODE ssc_get_FSYNC_role(e_ssc_Role_Rx_Tx_t *p_e_role)
{
  // Just load value to defined address
  *p_e_role = s_ssc_settings.e_FSYNC_role;
  return SSC_SUCCESS;
}






/**
 * @brief Set BCLK synchronization edge for RX module
 * Due to flexibility of system, there can be request to sample RX data on\n
 * different edges.
 *
 * @param e_edge Options: SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 * @return SSC_SUCCESS (0) if all OK
 */
SSC_RES_CODE ssc_set_BCLK_RX_edge(e_ssc_edge_t e_edge)
{
  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;


  // Check input parameter
  if( (e_edge != SSC_EDGE_FALLING) &&
      (e_edge != SSC_EDGE_RISING) &&
      (e_edge != SSC_EDGE_DEFAULT))
  {
    return SSC_INCORRECT_PARAMETER;
  }

  // Process value itself
  switch(e_edge)
  {
  case SSC_EDGE_FALLING:
    p_ssc->RCMR.cki = 0;
    break;
  case SSC_EDGE_RISING:
    p_ssc->RCMR.cki = 1;
    break;
  case SSC_EDGE_DEFAULT:
    // At all modes always sample on rising edge
    p_ssc->RCMR.cki = 1;
    break;
  // Some fail -> error
  default:
    return SSC_FAIL;
  }

  // Save value
  s_ssc_settings.e_BCLK_RX_edge = e_edge;

  return SSC_SUCCESS;
}

/**
 * @brief Get BCLK synchronization edge for RX module
 * Due to flexibility of system, there can be request to sample RX data on\n
 * different edges. This function give actual setting.
 *
 * @param p_e_edge Address where result will be written\n
 * Options: SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 * @return SSC_SUCCESS (0) if all OK
 */
inline SSC_RES_CODE ssc_get_BCLK_RX_edge(e_ssc_edge_t *p_e_edge)
{
  *p_e_edge = s_ssc_settings.e_BCLK_RX_edge;
  return SSC_SUCCESS;
}



/**
 * @brief Set BCLK synchronization edge for TX module
 * Due to flexibility of system, there can be request to transmit TX data on\n
 * different edges.
 *
 * @param e_edge Options: SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 * @return SSC_SUCCESS (0) if all OK
 */
SSC_RES_CODE ssc_set_BCLK_TX_edge(e_ssc_edge_t e_edge)
{
  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;


  // Check input parameter
  if( (e_edge != SSC_EDGE_FALLING) &&
      (e_edge != SSC_EDGE_RISING) &&
      (e_edge != SSC_EDGE_DEFAULT))
  {
    return SSC_INCORRECT_PARAMETER;
  }

  // Process value itself
  switch(e_edge)
  {
  case SSC_EDGE_FALLING:
    p_ssc->TCMR.cki = 0;
    break;
  case SSC_EDGE_RISING:
    p_ssc->TCMR.cki = 1;
    break;
  case SSC_EDGE_DEFAULT:
    // At all modes always transmit samples on falling edge
    p_ssc->TCMR.cki = 0;
    break;
  // Some fail -> error
  default:
    return SSC_FAIL;
  }

  // Save value
  s_ssc_settings.e_BCLK_TX_edge = e_edge;
  return SSC_SUCCESS;
}


/**
 * @brief Get BCLK synchronization edge for TX module
 * Due to flexibility of system, there can be request to transmit TX data on\n
 * different edges. This function give actual setting.
 *
 * @param p_e_edge Address where result will be written\n
 * Options: SSC_FALLING, SSC_RISING or SSC_EDGE_DEFAULT
 * @return SSC_SUCCESS (0) if all OK
 */
inline SSC_RES_CODE ssc_get_BCLK_TX_edge(e_ssc_edge_t *p_e_edge)
{
  *p_e_edge = s_ssc_settings.e_BCLK_TX_edge;
  return SSC_SUCCESS;
}





/**
 * @brief Simply disable RX and TX module
 * @return SSC_SUCCESS (0) if all right
 */
inline SSC_RES_CODE ssc_disable_RX_TX(void)
{
  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;

  // Disable TX and RX module
  p_ssc->CR.txdis = 1;
  p_ssc->CR.rxdis = 1;

  return SSC_SUCCESS;
}


/**
 * Just enable RX and TX module
 * @return SSC_SUCCESS (0) if all right
 */
inline SSC_RES_CODE ssc_enable_RX_TX(void)
{
  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;

  // Enable TX and RX module
  p_ssc->CR.txen = 1;
  p_ssc->CR.rxen = 1;

  return SSC_SUCCESS;
}
