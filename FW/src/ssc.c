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
 * Modified: 25.08.2014
 *
 * \version 0.1
 * \author Martin Stejskal
 */

#include "ssc.h"

///\todo REMOVE - DEBUG
#include <stdio.h>
#include "print_funcs.h"
//============================| Global variables |=============================
static s_ssc_settings_t settings;
//================================| Functions |================================
//==========================| High level functions |===========================
SSC_RES_CODE ssc_init(void)
{
  // For storing status codes
  SSC_RES_CODE e_status;

  // First, we must reset codec
  e_status = ssc_reset();
  if(e_status != SSC_SUCCESS)
  {
    return e_status;
  }

  // Set default values
  e_status = ssc_FSYNC_RX_edge(SSC_DEFAULT_FSYNC_EDGE);
  if(e_status != SSC_SUCCESS)
  {
    return e_status;
  }


  e_status = ssc_set_digital_interface_mode(
                                        SSC_DEFAULT_DIGITAL_AUDIO_INTERFACE);
  if(e_status != SSC_SUCCESS)
  {
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



//===========================| Mid level functions |===========================
inline SSC_RES_CODE ssc_wait_for_FSYNC_RX(void)
{
  switch(settings.e_dig_aud_mode)
  {
  case SSC_I2S:
    if(settings.e_FSYNC_RX_edge == SSC_FALLING)
    {// Falling edge
      while (!gpio_get_pin_value(SSC_EXTERNAL_FSYNC_DETECTION_PIN));
      while (gpio_get_pin_value(SSC_EXTERNAL_FSYNC_DETECTION_PIN));
    }
    else
    {// Rising edge
      while (gpio_get_pin_value(SSC_EXTERNAL_FSYNC_DETECTION_PIN));
      while (!gpio_get_pin_value(SSC_EXTERNAL_FSYNC_DETECTION_PIN));
    }
    break;
  case SSC_DSP:
    ///\todo Complete
    break;
  case SSC_LEFT_JUSTIFIED:
    ///\todo Complete
    break;
  case SSC_RIGHT_JUSTIFIED:
    ///\todo Complete
    break;
  // Else mode is unknown
  default:
    return SSC_INCORRECT_PARAMETER;
  }
  return SSC_SUCCESS;
}



SSC_RES_CODE ssc_set_digital_interface_mode(
    e_ssc_digital_audio_interface_t e_mode)
{
  ///\ todo complete this function
  switch(e_mode)
  {
  case SSC_I2S:
    break;
  case SSC_DSP:
    break;
  case SSC_LEFT_JUSTIFIED:
    break;
  case SSC_RIGHT_JUSTIFIED:
    break;
  // Else mode is unknown
  default:
    return SSC_INCORRECT_PARAMETER;
  }

  // Save setting
  settings.e_dig_aud_mode = e_mode;

  return SSC_SUCCESS;
}

//===========================| Low level functions |===========================
SSC_RES_CODE ssc_FSYNC_RX_edge(e_ssc_edge_t e_edge)
{
  // Check input parameter
  if( (e_edge != SSC_FALLING) &&
      (e_edge != SSC_RISING) )
  {
    return SSC_INCORRECT_PARAMETER;
  }

  // Else is all OK -> save value
  settings.e_FSYNC_RX_edge = e_edge;

  return SSC_SUCCESS;
}



SSC_RES_CODE ssc_FSYNC_role(e_ssc_Role_Rx_Tx_t e_role)
{
  // Pointer to SSC (memory and structure)
  volatile avr32_ssc_t *p_ssc;
  p_ssc = SSC_DEVICE;

  switch(e_role)
  {
    // Slave (receive FSYNC)
    case SSC_RX:
      p_ssc->TFMR.fsos = AVR32_SSC_TFMR_FSOS_INPUT_ONLY;
      break;
    // Master (generate FSYNC)
    case SSC_TX:
      p_ssc->TFMR.fsos = AVR32_SSC_TFMR_FSOS_NEG_PULSE;
      break;
    // UNknown parameter -> fail
    default:
      return SSC_INCORRECT_PARAMETER;
  }

  // Everything look OK
  return SSC_SUCCESS;
}


