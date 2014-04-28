/**
 * \file
 *
 * \brief Board driver for "Sonochan mkII" HW version 03
 *
 * Allow set basic settings on board. Also support "generic driver"
 *
 * Created:  23.04.2014\n
 * Modified: 23.04.2014\n
 *
 * \version 0.1
 * \author  Martin Stejskal
 */

#include "brd_driver_hw_03.h"

///\todo REMOVE
#include <stdio.h>
#include "print_funcs.h"

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
GD_RES_CODE brd_drv_TLV_default(void);

//==========================| High level functions |===========================
GD_RES_CODE brd_drv_init(void)
{
  // Variable for storing status
  GD_RES_CODE e_status;

  // Set I/O


  // Prepare TLV
  e_status = brd_drv_TLV_default();
  if(e_status != GD_SUCCESS)

  print_dbg("R DAC OK\n");

  return GD_SUCCESS;
}
//===========================| Mid level functions |===========================


//===========================| Low level functions |===========================



//=========================| Functions not for user |==========================
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
