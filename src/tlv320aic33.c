/**
 * \file
 *
 * \brief Driver for codec TLV320AIC33
 *
 * Created:  02.04.2014\n
 * Modified: 21.04.2014
 *
 * \version 0.1a
 * \author Martin Stejskal
 */

#include "tlv320aic33.h"

///todo REMOVE
#include <stdio.h>
#include "print_funcs.h"
//===============================| Structures |================================

//============================| Global variables |=============================

/**
 * \brief Table with volume values in float type
 *
 * This table is used to find float value to raw volume value. Index is raw\n
 * volume value and content is float value, so it is easy to find float\n
 * number or raw value equal to float value.
 */
// Test architecture (AVR8 versus other)
#ifdef __AVR_ARCH__
const float TLV320AIC33_volume_table_float[] PROGMEM =
#else
const float TLV320AIC33_volume_table_float[] =
#endif
    {    0,  -0.5,    -1,  -1.5,    -2,  -2.5,    -3,  -3.5,    -4,  -4.5,
        -5,  -5.5,    -6,  -6.5,    -7,  -7.5,    -8,  -8.5,    -9,  -9.5,
       -10, -10.5,   -11, -11.5,   -12, -12.5,   -13, -13.5,   -14, -14.5,
       -15, -15.5,   -16, -16.5,   -17, -17.5,   -18, -18.6, -19.1, -19.6,
     -20.1, -20.6, -21.1, -21.6, -22.1, -22.6, -23.1, -23.6, -24.1, -24.6,
     -25.1, -25.6, -26.1, -26.6, -27.1, -27.6, -28.1, -28.6, -29.1, -29.6,
     -30.1, -30.6, -31.1, -31.6, -32.1, -32.6, -33.1, -33.6, -34.1, -34.6,
     -35.1, -35.7, -36.1, -36.7, -37.1, -37.7, -38.2, -38.7, -39.2, -39.7,
     -40.2, -40.7, -41.2, -41.7, -42.2, -42.7, -43.2, -43.8, -44.3, -44.8,
     -45.2, -45.8, -46.2, -46.7, -47.4, -47.9, -48.2, -48.7, -49.3,   -50,
     -50.3,   -51, -51.4, -51.8, -52.2, -52.7, -53.7, -54.2, -55.3, -56.7,
     -58.3, -60.2, -62.7, -64.3, -66.2, -68.7, -72.2, -78.3
    };

//=========================| Generic driver support |==========================
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
/**
 * \brief Virtual register image
 *
 * This values codec directly do not have in memory. However, there are in\n
 * more human readable format. Also some values can use generic driver (need\n
 * to point to result variables and so on).\n
 * It is static, so it is "global" only for this file.
 *
 * \note This structure have meaning only if generic driver is enable
 */
static tlv320aic33_virtual_reg_img_t s_virtual_reg_img;


/**
 * \brief Configure table for device
 */

// Test architecture (AVR8 versus other)
#ifdef __AVR_ARCH__
const gd_config_struct TLV320AIC33_config_table[] PROGMEM =
#else
const gd_config_struct TLV320AIC33_config_table[] =
#endif
  {
      {
        0,                      // Command ID
        "Initialize TLV320AIC33 hardware",  // Name
        "Initialize I/O and TWI module (if used)",      // Descriptor
        void_type,              // Input data type
        {.data_uint32 = 0},     // Minimum input value
        {.data_uint32 = 0},     // Maximum input value
        void_type,              // Output data type
        {.data_uint32 = 0},     // Minimum output value
        {.data_uint32 = 0},     // Maximum output value
        (GD_DATA_VALUE*)&gd_void_value, // Output value
        tlv320aic33_init                /* Function, that should be
                                         * called
                                         */
      },
      {
        1,
        "Digital interface master/slave",
        "Options: 0 - slave (BCLK,WCLK in); 1 - master (BLCK,WCLK out)",
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        (GD_DATA_VALUE*)&s_virtual_reg_img.i_digital_interface_master_slave,
        tlv320aic33_set_digital_interface_as_master
      },
      {
        2,
        "Digital interface data mode",
        "0  - I2S ; 1 - DSP ; 2 - right justified ; 3 - left justified",
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 3},
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 3},
        (GD_DATA_VALUE*)&s_virtual_reg_img.i_digital_inteface_data_mode,
        tlv320aic33_set_data_interface_mode
      },
      {
        3,
        "Word length",
        "Options: 16, 20, 24, 32",
        uint8_type,
        {.data_uint8 = 16},
        {.data_uint8 = 32},
        uint8_type,
        {.data_uint8 = 16},
        {.data_uint8 = 32},
        (GD_DATA_VALUE*)&s_virtual_reg_img.i_word_length,
        tlv320aic33_set_word_length
      },
      {
        4,
        "Set DAC path from digital input to headphones and line out",
        "Options: 0 - play mono (mix); 1 - play stereo ; 2 - not initialized",
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 2},
        (GD_DATA_VALUE*)&s_virtual_reg_img.i_dac_play_stereo,
        tlv320aic33_set_DAC_play_input_data
      },
      {
        5,
        "Headphones volume",
        "Range: 0 to -78.3 dB (when -79 dB -> mute)",
        float_type,
        {.data_float = -79},
        {.data_float = 0},
        float_type,
        {.data_float = -79},
        {.data_float = 0},
        (GD_DATA_VALUE*)&s_virtual_reg_img.i_headphones_volume_db,
        tlv320aic33_set_headphones_volume_dB
      },
      {
        6,
        "Headphones output mode",
        "Options: 0 - differential ; 1 - single ended",
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        (GD_DATA_VALUE*)&s_virtual_reg_img.i_headphones_output_single_ended,
        tlv320aic33_set_headphones_single_ended
      }
  };
/// \brief Maximum command ID (is defined by last command)
#define TLV320AIC33_MAX_CMD_ID          6


const gd_metadata TLV320AIC33_metadata =
{
  TLV320AIC33_MAX_CMD_ID,              // Max CMD ID
  "Audio codec TLV320AIC33 v0.1a",     // Description
  (gd_config_struct*)&TLV320AIC33_config_table[0],      // Pointer to table
  0x66    // Serial number (0~255)
};
#endif

//=============================| FreeRTOS stuff |==============================
// If RTOS support is enabled, create this
#if TLV320AIC33_SUPPORT_RTOS != 0
portBASE_TYPE xStatus;
xSemaphoreHandle mutexI2C;
#endif


//====================| Function prototypes not for user |=====================
GD_RES_CODE tlv320aic33_find_raw_volume_value(
    float f_value,
    uint8_t *p_raw_volume);

GD_RES_CODE tlv320aic33_find_float_volume_value(
    uint8_t i_raw_volume,
    float *p_float);
//==========================| High level functions |===========================

/**
 * \brief Initialize TLV320AIC33
 *
 * Must be called \b before any \b other \b function from this library!
 *
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE tlv320aic33_init(void)
{
  // If RTOS support enable and create flag is set then create mutex
#if (TLV320AIC33_SUPPORT_RTOS != 0) && (TLV320AIC33_RTOS_CREATE_MUTEX != 0)
  mutexI2C = xSemaphoreCreateMutex();
#endif


  // Lock TWI module if RTOS used
  TLV320AIC33_LOCK_TWI_MODULE_IF_RTOS
  // Initialize low-level driver and test status
  if(tlv320aic33_HAL_init() != TLV320AIC33_OK)
  {
    /* If not OK (do not care about error), just return FAIL (because of limit
     * return values GD_RES_CODE). Also unlock TWI module
     */
    TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS
    return GD_FAIL;
  }
  // Unlock TWI module
  TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS


  // So, now reset codec (to get it to known state). Return state
  return tlv320aic33_reset();
}



/**
 * \brief Set DAC data path to input data
 * @param i_play_stereo Options: 0 - mix left and right input data channel ;\n
 *  1 - input data are send directly to DAC
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE tlv320aic33_set_DAC_play_input_data(uint8_t i_play_stereo)
{
  // For result codes
  GD_RES_CODE e_status;

  // Page 0, register 7 structure
  p0_r7_Codec_Datapath_Setup_t p0_r7;

  // Get setting from codec
  e_status = tlv320aic33_read_data(7, &p0_r7.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  if(i_play_stereo == 0)
  {
    // User want play mono
    p0_r7.s.LeftDACDatapathControl =
        left_dac_datapath_plays_mono_mix_left_and_right_channel_input_data;
    p0_r7.s.RightDACDatapathControl =
        right_dac_datapath_plays_mono_mix_left_and_right_channel_input_data;
  }
  else
  {
    // User want stereo
    p0_r7.s.LeftDACDatapathControl =
        left_dac_datapath_plays_left_channel_input_data;
    p0_r7.s.RightDACDatapathControl =
        right_dac_datapath_plays_right_channel_input_data;
  }
  // Set register
  e_status = tlv320aic33_write_data(7, p0_r7.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  /* Set output DAC path to L1 R1 (allow to set independent volume). Also
   * route DAC_(L/R)1 to HP(L/R)OUT.
   */
  // Page 0, register 41 structure
  p0_r41_DAC_Output_Switching_Control_t p0_r41;
  // Page 0, register 47 structure
  p0_r47_DAC_L1_To_HPLOUT_Volume_Control_t p0_r47;
  // Page 0, register 64 structure
  p0_r64_DAC_R1_To_HPROUT_Volume_Control_t p0_r64;



  e_status = tlv320aic33_read_data(41, &p0_r41.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(47, &p0_r47.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(64, &p0_r64.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  // Configure
  p0_r41.s.LeftDACOutputSwitchingControl = left_DAC_output_DAC_L1_path;
  p0_r41.s.RightDACOutputSwitchingControl = right_DAC_output_DAC_R1_path;

  p0_r47.s.DACL1RouteToHPLOUTEnable = 1;
  p0_r64.s.DACR1RouteToHPROUTEnable = 1;
  // Set register
  e_status = tlv320aic33_write_data(41, p0_r41.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_write_data(47, p0_r47.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  return tlv320aic33_write_data(64, p0_r64.i_reg);
}



/**
 * \brief Set digital interface as master or slave
 *
 * When set as master BCLK and WCLK are configured as output. Else BCLK and\n
 * WCLK is configured as input.
 * @param i_master Options: 0 - configure as slave ; 1 - configure as master
 * @return GD_SUCCESS (0) if all OKs
 */
GD_RES_CODE tlv320aic33_set_digital_interface_as_master(uint8_t i_master)
{
  // For result codes
  GD_RES_CODE e_status;

  // Page 0, register 8 structure
  p0_r8_Audio_Serial_Interface_Control_A_t p0_r8;

  // Get setting from codec
  e_status = tlv320aic33_read_data(8, &p0_r8.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  if(i_master == 0)
  {
    // Configure as slave
    p0_r8.s.BitClockDirectionalControl = bit_clock_is_an_input;
    p0_r8.s.WordClockDirectionalControl = word_clock_is_an_input;
  }
  else
  {
    // Configure as master
    p0_r8.s.BitClockDirectionalControl = bit_clock_is_an_output;
    p0_r8.s.WordClockDirectionalControl = word_clock_is_an_output;
  }
  // Set register
  e_status = tlv320aic33_write_data(8, p0_r8.i_reg);

  // Only if generic driver support is enabled...
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
  if(e_status != GD_SUCCESS)
    return e_status;

  // If all right, write to virtual register
  s_virtual_reg_img.i_digital_interface_master_slave = i_master;
#endif
  return e_status;
}



/**
 * \brief Set data interface mode
 * @param i_mode Options are defined in AudioSerialDataInterfaceTransferMode_e
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_data_interface_mode(
    AudioSerialDataInterfaceTransferMode_e e_mode)
{
  // For result codes
  GD_RES_CODE e_status;

  // Page 0, register 9 structure
  p0_r9_Audio_Serial_Interface_Control_B_t p0_r9;

  // Get setting from codec
  e_status = tlv320aic33_read_data(9, &p0_r9.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Set configuration
  p0_r9.s.AudioSerialDataInterfaceTransferMode = e_mode;

  // Set register
  e_status = tlv320aic33_write_data(9, p0_r9.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
  if(e_status != GD_SUCCESS)
    return e_status;
  s_virtual_reg_img.i_digital_inteface_data_mode = e_mode;
#endif
  return e_status;
}



/**
 * \brief Set word length
 * @param i_word_length Options: 16, 20, 24, 32
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_word_length(uint8_t i_word_length)
{
  // For result codes
  GD_RES_CODE e_status;

  // Word length as enum
  AudioSerialDataWordLengthControl_e e_word_length;

  char tmp[20];
  sprintf(&tmp[0], "I wor: %d\n", i_word_length);
  print_dbg(&tmp[0]);
  // Assign input value correct value for codec
  switch(i_word_length)
  {
  case 16:
    e_word_length = audio_data_word_length_16bit;
    break;
  case 20:
    e_word_length = audio_data_word_length_20bit;
    break;
  case 24:
    e_word_length = audio_data_word_length_24bit;
    break;
  case 32:
    e_word_length = audio_data_word_length_32bit;
    break;
  default:
    // Unknown option
    return GD_INCORRECT_PARAMETER;
  }

  // Page 0, register 9 structure
  p0_r9_Audio_Serial_Interface_Control_B_t p0_r9;

  // Get setting from codec
  e_status = tlv320aic33_read_data(9, &p0_r9.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Set configuration
  p0_r9.s.AudioSerialDataWordLengthControl = e_word_length;

  // Set register
  e_status = tlv320aic33_write_data(9, p0_r9.i_reg);
  // Only if generic driver support is enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
  if(e_status != GD_SUCCESS)
    return e_status;
  // If all OK -> save actual value
  s_virtual_reg_img.i_word_length = i_word_length;
#endif
  return e_status;
}



/**
 * \brief Enable or disable DACs
 * @param enable_DACs Options: 0 - disable DACs ; 1 - enable DACs
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_DAC_power(uint8_t enable_DACs)
{
  // For result codes
  GD_RES_CODE e_status;

  // Page 0, register 37
  p0_r37_DAC_Power_And_Output_Driver_Control_t p0_r37;

  // Get setting from codec
  e_status = tlv320aic33_read_data(37, &p0_r37.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Set configuration
  if(enable_DACs == 0)
  {
    p0_r37.s.LeftDACPower = 0;
    p0_r37.s.RightDACPower = 0;
  }
  else
  {
    p0_r37.s.LeftDACPower = 1;
    p0_r37.s.RightDACPower = 1;
  }
  // Set register
  return tlv320aic33_write_data(37, p0_r37.i_reg);
}



/**
 * \brief Set headphone COM pins as single ended or differential
 * @param single_ended 0 - HPxCOM as differential ; 1 - HPxCOM as single ended
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_headphones_single_ended(uint8_t i_single_ended)
{
  // For result codes
  GD_RES_CODE e_status;

  // Page 0, register 37
  p0_r37_DAC_Power_And_Output_Driver_Control_t p0_r37;
  // Page 0, register 38
  p0_r38_High_Power_Output_Driver_Control_t p0_r38;

  // Get setting from codec
  e_status = tlv320aic33_read_data(37, &p0_r37.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(38, &p0_r38.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Set configuration
  if(i_single_ended == 0)
  {
    // User want differential out
    p0_r37.s.HPLCOMOutputDriverConfigurationControl =
        HPLCOM_is_differential_of_HPLOUT;
    p0_r38.s.HPRCOMOutputDriverConfigurationControl =
        HPRCOM_is_differential_of_HPROUT;
  }
  else
  {
    // User want single ended out
    p0_r37.s.HPLCOMOutputDriverConfigurationControl =
        HPLCOM_is_single_ended;
    p0_r38.s.HPRCOMOutputDriverConfigurationControl=
        HPRCOM_is_single_ended;
  }
  // Set registers
  e_status = tlv320aic33_write_data(37, p0_r37.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_write_data(38, p0_r38.i_reg);
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER !=0
  if(e_status != GD_SUCCESS)
    return e_status;
  // If all OK, save status to virtual register
  s_virtual_reg_img.i_headphones_output_single_ended = i_single_ended;
#endif
  return e_status;
}



/**
 * \brief Enable or disable mute flag on DAC
 * @param i_mute_flag 0 - disable mute ; 1 - enable mute
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_DAC_mute(uint8_t i_mute_flag)
{
  // For result codes
  GD_RES_CODE e_status;

  // Page 0, register 43 (DAC), 44 (DAC), 51 (headphones), 65 (headphones)
  p0_r43_Left_DAC_Digital_Volume_Control_t p0_r43;
  p0_r44_Right_DAC_Digital_Volume_Control_t p0_r44;
  p0_r51_HPLOUT_Output_Level_Control_t p0_r51;
  p0_r65_HPROUT_Output_Level_Control_t p0_r65;

  // Get settings from codec
  e_status = tlv320aic33_read_data(43, &p0_r43.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(44, &p0_r44.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(51, &p0_r51.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(65, &p0_r65.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Configure registers
  if(i_mute_flag == 0)
  {
    // Mute disable
    p0_r43.s.LeftDACDigitalMute = 0;
    p0_r44.s.RightDACDigitalMute = 0;
    p0_r51.s.HPLOUTMute = 1;    // It is strange, but in datasheet it is this way
    p0_r65.s.HPROUTMute = 1;
    p0_r51.s.HPLOUTFullyPoweredUp = 1;
    p0_r65.s.HPROUTFullyPoweredUp = 1;
  }
  else
  {
    // Mute enable
    p0_r43.s.LeftDACDigitalMute = 1;
    p0_r44.s.RightDACDigitalMute = 1;
    p0_r51.s.HPLOUTMute = 0;
    p0_r65.s.HPROUTMute = 0;
    p0_r51.s.HPLOUTFullyPoweredUp = 1;
    p0_r65.s.HPROUTFullyPoweredUp = 1;
  }

  // Set registers
  e_status = tlv320aic33_write_data(43, p0_r43.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_write_data(44, p0_r44.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_write_data(51, p0_r51.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  return tlv320aic33_write_data(65, p0_r65.i_reg);
}


/**
 * \brief Set DAC volume. Volume is in dB
 *
 * Because codec have 0.5 dB step, function convergence input parameter to\n
 *  closest result.
 * @param f_volume Volume in dB. Range is from 0 to -63.5 dB. Step is 0.5 dB.
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE tlv320aic33_set_DAC_volume_dB(float f_volume)
{
  // Check boundaries
  if(f_volume >0)       // Can not be higher than 0
    return GD_INCORRECT_PARAMETER;
  /* Check if smaller than -63.5 -> if yes -> set mute flag and set lowest
   * possible volume
   */
  if(f_volume < -63.5)
  {
    // Set mute flag
    tlv320aic33_set_DAC_mute(1);
    // And value to lowest possible
    f_volume = -63.5;
  }

  // Float to raw data and call function
  return tlv320aic33_set_DAC_volume( (uint8_t)( (f_volume*(-2))-0.5 )  );
}



/**
 * \brief Set CLKDIV_IN source
 * @param e_source Options are defined in CLKDIV_INSourceSelection_e
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_CLKDIV_IN_source(
    CLKDIV_INSourceSelection_e e_source)
{
  // For result codes
  GD_RES_CODE e_status;

  // Page 0, register 102
  p0_r102_Clock_Generation_Control_t p0_r102;

  // Get setting
  e_status = tlv320aic33_read_data(102, &p0_r102.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Change setting
  p0_r102.s.CLKDIV_INSourceSelection = e_source;
  // Set register
  return tlv320aic33_write_data(102, p0_r102.i_reg);
}



/**
 * \brief Set PLLCLK_IN source
 * @param e_source Options are defined in PLLCLK_INSourceSelection_e
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_PLLCLK_IN_source(
    PLLCLK_INSourceSelection_e e_source)
{
  // For result codes
  GD_RES_CODE e_status;

  // Page 0, register 102
  p0_r102_Clock_Generation_Control_t p0_r102;

  // Get setting
  e_status = tlv320aic33_read_data(102, &p0_r102.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Change setting
  p0_r102.s.PLLCLK_INSourceSelection = e_source;
  // Set register
  return tlv320aic33_write_data(102, p0_r102.i_reg);
}



/**
 * \brief Set headphones volume. Volume is in dB
 *
 * Because codec approximately 0.5 dB step, function convergence input\n
 * parameter to closest result.
 * @param f_volume Volume in dB. Range is from 0 to -78.3 dB. Step is\n
 * approximately 0.5 dB.
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE tlv320aic33_set_headphones_volume_dB(float f_volume)
{
  // For result codes
  GD_RES_CODE e_status;

  // Volume raw value (for codec register)
  uint8_t i_vol_raw;

  // Function that look into table and find correct raw value
  e_status = tlv320aic33_find_raw_volume_value(f_volume, &i_vol_raw);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Change setting
  return tlv320aic33_set_headphones_volume(i_vol_raw);
}
//===========================| Mid level functions |===========================
/**
 * \brief Set page to 0 and restart codec
 *
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE tlv320aic33_reset(void)
{
  // For store status code
  GD_RES_CODE e_status;

#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
  // Set virtual register image to default values
  // Default TLV volume
  s_virtual_reg_img.i_headphones_volume_db = 0;
  /* We do not know yet. By default is this value set to 2, so user get thru
   * generic driver "strange" value.
   */
  s_virtual_reg_img.i_dac_play_stereo = 2;
  // Set default TLV value
  s_virtual_reg_img.i_digital_interface_master_slave = 0;
  // Set default TLV value
  s_virtual_reg_img.i_digital_inteface_data_mode = 0;
  // Set default word length (defined in TLV)
  s_virtual_reg_img.i_word_length = 16;
  // In default headphone output is differential
  s_virtual_reg_img.i_headphones_output_single_ended = 0;
#endif

  // Reset codec. 1) Set page to 0  2) Set self cleaning software reset
  p0_r0_Page_Select_t p0_r0_Page_Select;
  p0_r0_Page_Select.s.PageSelect = 0;   // Set page to 0

  // Save status and check it
  e_status = tlv320aic33_write_data(0, p0_r0_Page_Select.i_reg);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  p0_r1_Software_Reset_t p0_r1_SW_res;
  p0_r1_SW_res.s.SoftwareReset = 1;     // Set reset to 1

  // Save status and return it
  e_status = tlv320aic33_write_data(1, p0_r1_SW_res.i_reg);
  return e_status;
}



/**
 * \brief Set output driver power on delay
 * @param e_delay Options are defined in OutputDriverPowerOnDelayControl_e
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_output_driver_power_on_delay(
    OutputDriverPowerOnDelayControl_e e_delay)
{
  // For store status code
  GD_RES_CODE e_status;

  // Page 0, register 42
  p0_r42_Output_Driver_Pop_Reduction_t p0_r42;

  // Get setting from codec
  e_status = tlv320aic33_read_data(42, &p0_r42.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Configure
  p0_r42.s.OutputDriverPowerOnDelayControl = e_delay;
  // Set register
  return tlv320aic33_write_data(42, p0_r42.i_reg);
}



/**
 * \brief Set driver ramp-up step time
 * @param e_time Options are defined in DriverRampUpStepTimingControl_e
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_driver_ramp_up_step_time(
    DriverRampUpStepTimingControl_e e_time)
{
  // For store status code
  GD_RES_CODE e_status;

  // Page 0, register 42
  p0_r42_Output_Driver_Pop_Reduction_t p0_r42;

  // Get setting from codec
  e_status = tlv320aic33_read_data(42, &p0_r42.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Configure
  p0_r42.s.DriverRampUpStepTimingControl = e_time;
  // Set register
  return tlv320aic33_write_data(42, p0_r42.i_reg);
}



/**
 * \brief Set DAC volume in raw format
 *
 * Volume is in binary representation in codec register, so it is up to user\n
 * to recalculate it to dB.
 *
 * @param i_volume Volume value (7 bit number) in binary representation for\n
 *  register
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_DAC_volume(uint8_t i_volume)
{
  // For store status code
  GD_RES_CODE e_status;

  // Check input variable
  if(i_volume >= 128)
    return GD_INCORRECT_PARAMETER;

  // Page 0, register 43 and 44
  p0_r43_Left_DAC_Digital_Volume_Control_t p0_r43;
  p0_r44_Right_DAC_Digital_Volume_Control_t p0_r44;

  // Get settings from codec
  e_status = tlv320aic33_read_data(43, &p0_r43.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(44, &p0_r44.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Configure registers
  p0_r43.s.LeftDACDigitalVolumeControlSetting = i_volume;
  p0_r44.s.RightDACDigitalVolumeControlSetting = i_volume;

  // Set registers
  e_status = tlv320aic33_write_data(43, p0_r43.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  return tlv320aic33_write_data(44, p0_r44.i_reg);
}



/**
 * \brief Set headphone volume in raw format
 *
 * Volume is in binary representation in codec register, so it is up to user\n
 * to recalculate it to dB.
 *
 * @param i_volume Volume value (7 bit number) in binary representation for\n
 *  register
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_headphones_volume(uint8_t i_volume)
{
  // Check input value
  if(i_volume >= 128)
    return GD_INCORRECT_PARAMETER;

  // For result codes
  GD_RES_CODE e_status;

  // Page 0, register 47 and 64
  p0_r47_DAC_L1_To_HPLOUT_Volume_Control_t p0_r47;
  p0_r64_DAC_R1_To_HPROUT_Volume_Control_t p0_r64;

  // Get settings
  e_status = tlv320aic33_read_data(47, &p0_r47.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(64, &p0_r64.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Change setting
  p0_r47.s.DACL1RouteToHPLOUTVolume = i_volume;
  p0_r64.s.DACR1RouteToHPROUTVolume = i_volume;
  // Set registers
  e_status = tlv320aic33_write_data(47, p0_r47.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_write_data(64, p0_r64.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Only if generic driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
  // Calculate actual volume in dB and store it in virtual register
  return tlv320aic33_find_float_volume_value(
      i_volume,
      &s_virtual_reg_img.i_headphones_volume_db);
#else
  return e_status;
#endif
}
//===========================| Low level functions |===========================
/**
 * \brief Write data on TWI (I2C) bus
 *
 * MCU is in master mode. Send TLV320AIC33 address, then register number and\n
 * data. This function also check if TWI module is available (when RTOS\n
 * support is enabled).
 *
 * @param i_register_number Register number
 *
 * @param i_value Register value
 *
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE tlv320aic33_write_data(
    uint8_t i_register_number,
    uint8_t i_value)
{
  // If RTOS support enable, then "lock" TWI module
  TLV320AIC33_LOCK_TWI_MODULE_IF_RTOS

  // Write data (2B) to TLV thru HAL. Also check result status
  if(tlv320aic33_HAL_write_data(i_register_number, i_value) != TLV320AIC33_OK)
  {
    // If not OK -> unlock device (if needed) and return FAIL
    TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS
    return GD_FAIL;
  }

  // "Unlock" device if needed
  TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS
  return GD_SUCCESS;
}

/**
 * \brief Read data on TWI (I2C) bus
 *
 * MCU is in master mode. Send TLV320AIC33 address and register value. Stop\n
 * condition. Send address and then receive data (1B). This function also\n
 * check if TWI module is available (when RTOS support is enabled).
 *
 * @param p_data Data are saved thru this pointer
 *
 * @param i_register_number Register number
 *
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE tlv320aic33_read_data(
    uint8_t i_register_number,
    uint8_t *p_data)
{
  // If RTOS support enable, then "lock" TWI module
  TLV320AIC33_LOCK_TWI_MODULE_IF_RTOS

  // Read data from TLV thru HAL. Also check result status
  if(tlv320aic33_HAL_read_data(i_register_number, p_data) != TLV320AIC33_OK)
  {
    // If not OK -> unlock device (if needed) and return FAIL
    TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS
    print_dbg("TLV Read failed\n");
    return GD_FAIL;
  }

  // "Unlock" device if needed
  TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS
  return GD_SUCCESS;
}

//=========================| Functions not for user |==========================

/**
 * \brief Find raw volume value to input float value
 * @param f_value Input float value
 * @param p_raw_volume Pointer to raw variable. Address where result will be\n
 *  written.
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_find_raw_volume_value(
    float f_value,
    uint8_t *p_raw_volume)
{
  // Check input value
  if(f_value > 0)
    return GD_INCORRECT_PARAMETER;

  // Index counter
  uint8_t i;

  // Modify float value (to make sure that converge "right way")
  f_value = f_value +0.25;

  for(i=0 ;     // Initialize value
      i<(uint8_t)(sizeof(TLV320AIC33_volume_table_float)/        // Condition
                  sizeof(TLV320AIC33_volume_table_float[0])) ;
      i++)      // Action
  {
    // Try to find out closest value
    if(tlv320aic33_read_byte_ro_mem(&TLV320AIC33_volume_table_float[i])
        < f_value)
    {
      *p_raw_volume = i;
      return GD_SUCCESS;
    }
  }

  // If not found -> just mute
  *p_raw_volume = 127;
  return GD_SUCCESS;
}



/**
 * \brief Find input float value raw volume value
 * @param i_raw_volume Input raw volume value
 * @param p_float Pointer to float value. To this address will be written\n
 *  result.
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE tlv320aic33_find_float_volume_value(
    uint8_t i_raw_volume,
    float *p_float)
{
  // Check input value
  if(i_raw_volume >= 128)
    return GD_INCORRECT_PARAMETER;

  // Test if there is mute value
  if(i_raw_volume >= 118)
  {
    // Set float value
    *p_float = -79;     /* This value must correspond with generic driver
                         * minimum value.
                         */
    return GD_SUCCESS;
  }
  else
  {
    /* Else there is some value. Let's find correct float value
     * This will be easy, because index of array is basicly raw volume value
     */
    *p_float = tlv320aic33_read_dword_ro_mem(
        &TLV320AIC33_volume_table_float[i_raw_volume]);
    return GD_SUCCESS;
  }
}
