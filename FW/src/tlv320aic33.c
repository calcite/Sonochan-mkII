/**
 * \file
 *
 * \brief Driver for codec TLV320AIC33
 *
 * Created:  02.04.2014\n
 * Modified: 21.08.2014
 *
 * \version 0.2a
 * \author Martin Stejskal, Tomas Bajus
 */

#include "tlv320aic33.h"

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
        (GD_DATA_VALUE*)&s_virtual_reg_img.f_headphones_volume_db,
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
      },
      {
        7,
        "DAC power",
        "Options: 0 - disable power ; 1 -enable power",
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        (GD_DATA_VALUE*)&s_virtual_reg_img.i_dac_power,
        tlv320aic33_set_DAC_power
      },
      {
        8,
        "DAC mute",
        "Options: 0 - disable mute ; 1 - enable mute",
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        (GD_DATA_VALUE*)&s_virtual_reg_img.i_dac_mute,
        tlv320aic33_set_DAC_mute
      },

//tomas
      {
        9,
        "DAC volume",
        "Range: 0 to -63.5 dB (when -64 dB -> mute)",
        float_type,
        {.data_float = -64},
        {.data_float = 0},
        float_type,
        {.data_float = -64},
        {.data_float = 0},
        (GD_DATA_VALUE*)&s_virtual_reg_img.f_dac_volume_db,
        tlv320aic33_set_DAC_volume_dB
      },
      {
        10,
        "ADC power",
        "Options: 0 - disable power ; 1 -enable power",
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        (GD_DATA_VALUE*)&s_virtual_reg_img.i_adc_power,
        tlv320aic33_set_ADC_power
      },
      {
        11,
        "ADC mute",
        "Options: 0 - disable mute ; 1 - enable mute",
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 1},
        (GD_DATA_VALUE*)&s_virtual_reg_img.i_dac_mute,
        tlv320aic33_set_ADC_mute
      },

      {
        12,
        "ADC gain",
        "Range <0:59,5> dB, step 0,5dB",
        float_type,
        {.data_uint8 = 0},
        {.data_uint8 = 59.5},
        float_type,
        {.data_uint8 = 0},
        {.data_uint8 = 59.5},
        (GD_DATA_VALUE*)&s_virtual_reg_img.f_adc_gain_db,
        tlv320aic33_set_ADC_gain_dB
      },
      {
        13,
        "ADCL from LINE2L",
        "LINE2L to ADCL, gain: 0=0dB, 1=-1.5dB, 2=-3dB..8=-12dB, 15=not routed",
        uint8_type,
        {.data_uint8= 0},
        {.data_uint8 = 15},
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 15},
        (GD_DATA_VALUE*)&s_virtual_reg_img.e_line2l_to_adc_left_gain,
        tlv320aic33_set_ADC_L_from_LINE2_L
      },
      {
        14,
        "ADCR from LINE2R",
        "LINE2R to ADCR, gain: 0=0dB, 1=-1.5dB, 2=-3dB..8=-12dB, 15=not routed",
        uint8_type,
        {.data_uint8= 0},
        {.data_uint8 = 15},
        uint8_type,
        {.data_uint8 = 0},
        {.data_uint8 = 15},
        (GD_DATA_VALUE*)&s_virtual_reg_img.e_line2r_to_adc_right_gain,
        tlv320aic33_set_ADC_R_from_LINE2_R
      }
  };
/// \brief Maximum command ID (is defined by last command)
#define TLV320AIC33_MAX_CMD_ID          8


const gd_metadata TLV320AIC33_metadata =
{
  TLV320AIC33_MAX_CMD_ID,              // Max CMD ID
  "Audio codec TLV320AIC33 v0.2a",     // Description
  (gd_config_struct*)&TLV320AIC33_config_table[0],      // Pointer to table
  0x12    // Serial number (0~255)
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
  // Page 0, register 82 structure
  p0_r82_DAC_L1_To_LEFT_LOPM_Volume_Control_t p0_r82;
  // Page 0, register 92 structure
  p0_r92_DAC_R1_To_RIGHT_LOPM_Volume_Control_t p0_r92;




  e_status = tlv320aic33_read_data(41, &p0_r41.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(47, &p0_r47.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(64, &p0_r64.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(82, &p0_r82.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(92, &p0_r92.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Configure
  p0_r41.s.LeftDACOutputSwitchingControl = left_DAC_output_DAC_L1_path;
  p0_r41.s.RightDACOutputSwitchingControl = right_DAC_output_DAC_R1_path;

  p0_r47.s.DACL1RouteToHPLOUTEnable = 1;
  p0_r64.s.DACR1RouteToHPROUTEnable = 1;

  p0_r82.s.DAC_L1OutputRoutingControl = DAC_L1_routed_to_LEFT_LOPM;
  p0_r92.s.DAC_R1OutputRoutingControl = DAC_R1_routed_to_RIGHT_LOPM;


  // Set registers
  e_status = tlv320aic33_write_data(41, p0_r41.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_write_data(47, p0_r47.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_write_data(64, p0_r64.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_write_data(82, p0_r82.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_write_data(92, p0_r92.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;


#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
  // If all OK, jut write to virtual register
  if(i_play_stereo == 0)
  {
    s_virtual_reg_img.i_dac_play_stereo = 0;
  }
  else
  {
    s_virtual_reg_img.i_dac_play_stereo = 1;
  }
#endif
  // Else just return status
  return e_status;
}


/**
 * \brief set ADC Left from LINE1L
 * sets routing and gain of LINE1L input to left channel of ADC
 * @param e_gain Options: gain {0; -1.5; -3.. -12} or disconnect from ADC
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_L_from_LINE1_L(e_ADCInputGain e_gain)
{
	//For result codes
  GD_RES_CODE e_status;

  //Page 0, register 19 structure
  p0_r19_LINE1L_To_Left_ADC_Control_t p0_r19;

	//Get settings from codec
  e_status = tlv320aic33_read_data(19, &p0_r19.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  //Set configuration
  p0_r19.s.LINE1LInputLevelControlForLeftADCPGAMix = e_gain;

  //Write settings to codec
  e_status = tlv320aic33_write_data(19, p0_r19.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    s_virtual_reg_img.e_line1l_to_adc_left_gain = e_gain;
#endif
    return e_status;
}



/**
 * \brief set ADC Left from LINE1R
 * sets routing and gain of LINE1R input to left channel of ADC
 * @param e_gain Options: gain {0; -1.5; -3.. -12} or disconnect from ADC
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_L_from_LINE1_R(e_ADCInputGain e_gain)
{
	//For result codes
  GD_RES_CODE e_status;

  //Page 0, register 21 structure
  p0_r21_LINE1R_To_Left_ADC_Control_t p0_r21;

	//Get settings from codec
  e_status = tlv320aic33_read_data(21, &p0_r21.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  //Set configuration
  p0_r21.s.LINE1RInputLevelControlForLeftADCPGAMix = e_gain;

  //Write settings to codec
  e_status = tlv320aic33_write_data(21, p0_r21.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    s_virtual_reg_img.e_line1r_to_adc_left_gain = e_gain;
#endif
    return e_status;
}



/**
 * \brief set ADC Left from LINE2L
 * sets routing and gain of LINE2L input to left channel of ADC
 * @param e_gain Options: gain {0; -1.5; -3.. -12} or disconnect from ADC
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_L_from_LINE2_L(e_ADCInputGain e_gain)
{
	//For result codes
  GD_RES_CODE e_status;

  //Page 0, register 20 structure
  p0_r20_LINE2L_To_Left_ADC_Control_t p0_r20;

	//Get settings from codec
  e_status = tlv320aic33_read_data(20, &p0_r20.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  //Set configuration
  p0_r20.s.LINE2LInputLevelControlForADCPGAMix = e_gain;

  //Write settings to codec
  e_status = tlv320aic33_write_data(20, p0_r20.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    s_virtual_reg_img.e_line2l_to_adc_left_gain = e_gain;
#endif
    return e_status;
}



/**
 * \brief set ADC Left from MIC3L
 * sets routing and gain of MIC3L input to left channel of ADC
 * @param e_gain Options: gain {0; -1.5; -3.. -12} or disconnect from ADC
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_L_from_MIC3_L(e_ADCInputGain e_gain)
{
	//For result codes
  GD_RES_CODE e_status;

  //Page 0, register 17 structure
  p0_r17_MIC3L_R_To_Left_ADC_Control_t p0_r17;

	//Get settings from codec
  e_status = tlv320aic33_read_data(17, &p0_r17.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  //Set configuration
  p0_r17.s.MIC3LInputLevelControlForLeftADCPGAMix = e_gain;

  //Write settings to codec
  e_status = tlv320aic33_write_data(17, p0_r17.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    s_virtual_reg_img.e_mic3l_to_adc_left_gain = e_gain;
#endif
    return e_status;
}



/**
 * \brief set ADC Left from MIC3R
 * sets routing and gain of MIC3R input to left channel of ADC
 * @param e_gain Options: gain {0; -1.5; -3.. -12} or disconnect from ADC
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_L_from_MIC3_R(e_ADCInputGain e_gain)
{
	//For result codes
  GD_RES_CODE e_status;

  //Page 0, register 17 structure
  p0_r17_MIC3L_R_To_Left_ADC_Control_t p0_r17;

	//Get settings from codec
  e_status = tlv320aic33_read_data(17, &p0_r17.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  //Set configuration
  p0_r17.s.MIC3RInputLevelControlForLeftADCPGAMix = e_gain;

  //Write settings to codec
  e_status = tlv320aic33_write_data(17, p0_r17.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    s_virtual_reg_img.e_mic3r_to_adc_left_gain = e_gain;
#endif
    return e_status;
}



/**
 * \brief set ADC Right from LINE1L
 * sets routing and gain of LINE1L input to right channel of ADC
 * @param e_gain Options: gain {0; -1.5; -3.. -12} or disconnect from ADC
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_R_from_LINE1_L(e_ADCInputGain e_gain)
{
	//For result codes
  GD_RES_CODE e_status;

  //Page 0, register 24 structure
  p0_r24_LINE1L_To_Right_ADC_Control_t p0_r24;

	//Get settings from codec
  e_status = tlv320aic33_read_data(24, &p0_r24.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  //Set configuration
  p0_r24.s.LINE1LInputLevelControlForRightADCPGAMix = e_gain;

  //Write settings to codec
  e_status = tlv320aic33_write_data(24, p0_r24.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    s_virtual_reg_img.e_line1l_to_adc_right_gain = e_gain;
#endif
    return e_status;
}



/**
 * \brief set ADC Right from LINE1R
 * sets routing and gain of LINE1R input to right channel of ADC
 * @param e_gain Options: gain {0; -1.5; -3.. -12} or disconnect from ADC
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_R_from_LINE1_R(e_ADCInputGain e_gain)
{
	//For result codes
  GD_RES_CODE e_status;

  //Page 0, register 22 structure
  p0_r22_LINE1R_To_Right_ADC_Control_t p0_r22;

	//Get settings from codec
  e_status = tlv320aic33_read_data(22, &p0_r22.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  //Set configuration
  p0_r22.s.LINE1RInputLevelControlForRightADCPGAMix = e_gain;

  //Write settings to codec
  e_status = tlv320aic33_write_data(22, p0_r22.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    s_virtual_reg_img.e_line1r_to_adc_right_gain = e_gain;
#endif
    return e_status;
}



/**
 * \brief set ADC Right from LINE2R
 * sets routing and gain of LINE2R input to right channel of ADC
 * @param e_gain Options: gain {0; -1.5; -3.. -12} or disconnect from ADC
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_R_from_LINE2_R(e_ADCInputGain e_gain)
{
	//For result codes
  GD_RES_CODE e_status;

  //Page 0, register 23 structure
  p0_r23_LINE2R_To_Right_ADC_Control_t p0_r23;

	//Get settings from codec
  e_status = tlv320aic33_read_data(23, &p0_r23.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  //Set configuration
  p0_r23.s.LINE2RInputLevelControlForRightADCPGAMix = e_gain;

  //Write settings to codec
  e_status = tlv320aic33_write_data(23, p0_r23.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    s_virtual_reg_img.e_line2r_to_adc_right_gain = e_gain;
#endif
    return e_status;
}



/**
 * \brief set ADC Right from MIC3L
 * sets routing and gain of MIC3L input to right channel of ADC
 * @param e_gain Options: gain {0; -1.5; -3.. -12} or disconnect from ADC
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_R_from_MIC3_L(e_ADCInputGain e_gain)
{
	//For result codes
  GD_RES_CODE e_status;

  //Page 0, register 18 structure
  p0_r18_MIC3L_R_To_Right_ADC_Control_t p0_r18;

	//Get settings from codec
  e_status = tlv320aic33_read_data(18, &p0_r18.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  //Set configuration
  p0_r18.s.MIC3LInputLevelControlForRightADCPGAMix = e_gain;

  //Write settings to codec
  e_status = tlv320aic33_write_data(18, p0_r18.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    s_virtual_reg_img.e_mic3l_to_adc_right_gain = e_gain;
#endif
    return e_status;
}



/**
 * \brief set ADC Right from MIC3R
 * sets routing and gain of MIC3R input to right channel of ADC
 * @param e_gain Options: gain {0; -1.5; -3.. -12} or disconnect from ADC
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_R_from_MIC3_R(e_ADCInputGain e_gain)
{
	//For result codes
  GD_RES_CODE e_status;

  //Page 0, register 18 structure
  p0_r18_MIC3L_R_To_Right_ADC_Control_t p0_r18;

	//Get settings from codec
  e_status = tlv320aic33_read_data(18, &p0_r18.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  //Set configuration
  p0_r18.s.MIC3RInputLevelControlForRightADCPGAMix = e_gain;

  //Write settings to codec
  e_status = tlv320aic33_write_data(18, p0_r18.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    s_virtual_reg_img.e_mic3r_to_adc_right_gain = e_gain;
#endif
    return e_status;
}



/**
 * \brief set ADC power
 * enable or disable powering of both ADCs - left an right
 * @param i_enable_ADCs Options: 0 - not powered; 1 - powered
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_power(uint8_t i_enable_ADCs)
{
	//For result codes
  GD_RES_CODE e_status;

  //Page 0, register 19, 22 structure
  p0_r19_LINE1L_To_Left_ADC_Control_t p0_r19; //tomas - toto som premenoval cez refactor, nazov struktury bol bez _
  p0_r22_LINE1R_To_Right_ADC_Control_t p0_r22;

	//Get settings from codec
  e_status = tlv320aic33_read_data(19, &p0_r19.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  e_status = tlv320aic33_read_data(22, &p0_r22.i_reg);
    if(e_status != GD_SUCCESS)
      return e_status;

  //Set configuration
  if(i_enable_ADCs == 0)
  {
    p0_r19.s.LeftADCChannelPowerControl = 0;
    p0_r22.s.RightADCChannelPowerControl = 0;
  }
  else
  {
    p0_r19.s.LeftADCChannelPowerControl = 1;
    p0_r22.s.RightADCChannelPowerControl = 1;
  }

  //Write settings to codec
  e_status = tlv320aic33_write_data(19, p0_r19.i_reg);
  if(e_status != GD_SUCCESS)
        return e_status;
  e_status = tlv320aic33_write_data(22, p0_r22.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    // Check input value once more
    if(i_enable_ADCs == 0)
      s_virtual_reg_img.i_adc_power = 0;
    else
      s_virtual_reg_img.i_adc_power = 1;
#endif
    return e_status;
}


/**
 * \brief Set ADC mute flag on or off
 * @param i_mute_flag Options: 0 - mute off; 1 - mute on
 * @return GD_SUCCESS (0) if all OKs
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_mute(uint8_t i_mute_flag)
{
  //For result codes
  GD_RES_CODE e_status;

  //Page 0, register 15, 16 structure
  p0_r15_Left_ADC_PGA_Control_t p0_r15;
  p0_r16_Right_ADC_PGA_Control_t p0_r16;

  //Get settings from codec
  e_status = tlv320aic33_read_data(15, &p0_r15.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  e_status = tlv320aic33_read_data(16, &p0_r16.i_reg);
    if(e_status != GD_SUCCESS)
      return e_status;

  //Set configuration
  if(i_mute_flag == 0)
  {
    p0_r15.s.LeftADCPGAMute = 0;
    p0_r16.s.RigthADCPGAMute = 0;
  }
  else
  {
    p0_r15.s.LeftADCPGAMute = 1;
    p0_r16.s.RigthADCPGAMute = 1;
  }

  //Write settings to codec
  e_status = tlv320aic33_write_data(15, p0_r15.i_reg);
  if(e_status != GD_SUCCESS)
        return e_status;
  e_status = tlv320aic33_write_data(16, p0_r16.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
    if(e_status != GD_SUCCESS)
      return e_status;

    // Check input value once more
    if(i_mute_flag == 0)
      s_virtual_reg_img.i_adc_mute = 0;
    else
      s_virtual_reg_img.i_adc_mute = 1;
#endif
    return e_status;
}


/**
 * \brief set ADC gain
 * sets ADC gain in range <0:59,5> dB
 * @param f_gain - float value of ADC gain in dB (max 59,5, min 0)
 * @return GD_SUCCESS (0) if all OKs
 *
 */
//tomas
GD_RES_CODE tlv320aic33_set_ADC_gain_dB(float f_gain)
{
  //For result code
  GD_RES_CODE e_status;

  //Parameter range test
  #define ADC_MAX_GAIN 59.5
  if((f_gain > ADC_MAX_GAIN)||(f_gain < 0))
    return GD_INCORRECT_PARAMETER;

  //Page 0, register 15 structure
  p0_r15_Left_ADC_PGA_Control_t p0_r15;
  p0_r16_Right_ADC_PGA_Control_t p0_r16;

  //Get settings from codec
  e_status = tlv320aic33_read_data(15, &p0_r15.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  e_status = tlv320aic33_read_data(16, &p0_r16.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  //Set configuration
  p0_r15.s.LeftADCPGAGainSetting = (uint8_t)(2*f_gain);
  p0_r16.s.RightADCPGAGainSetting = (uint8_t)(2*f_gain);

  //Write settings to codec
  e_status = tlv320aic33_write_data(15, p0_r15.i_reg);
  if(e_status != GD_SUCCESS)
  			return e_status;
  e_status = tlv320aic33_write_data(16, p0_r16.i_reg);

  // Only if gneric driver support enabled
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
  if(e_status != GD_SUCCESS)
    return e_status;
  s_virtual_reg_img.f_adc_gain_db = f_gain;
#endif
	  return e_status;
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
 * @param i_mode Options are defined in e_AudioSerialDataInterfaceTransferMode
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_data_interface_mode(
    e_AudioSerialDataInterfaceTransferMode e_mode)
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
  e_AudioSerialDataWordLengthControl e_word_length;

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
  e_status = tlv320aic33_write_data(37, p0_r37.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;

  // Only if generic driver support is enabled...
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
  // Save info
  if(e_status != GD_SUCCESS)
    return e_status;
  if(enable_DACs == 0)
    s_virtual_reg_img.i_dac_power = 0;
  else
    s_virtual_reg_img.i_dac_power = 1;
#endif
  return e_status;
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

  // Page 0, register 86 (LEFT_LOPM)
  p0_r86_LEFT_LOPM_Output_Level_Control_Register p0_r86;
  // Page 0, register 93 (RIGHT_LOPM)
  p0_r93_RIGHT_LOPM_Output_Level_Control_Register p0_r93;

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
  e_status = tlv320aic33_read_data(86, &p0_r86.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_read_data(93, &p0_r93.i_reg);
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

    // Again. Do not know who develop this chip....
    p0_r86.s.LEFT_LOPMMute = 1;
    p0_r93.s.RIGHT_LOPMMute = 1;

    p0_r86.s.LEFT_LOPMPowerControl  =  LEFT_LOPM_fully_powered_up;
    p0_r93.s.RIGHT_LOPMPowerControl = RIGHT_LOPM_fully_powered_up;
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

    p0_r86.s.LEFT_LOPMMute = 0;
    p0_r93.s.RIGHT_LOPMMute = 0;

    p0_r86.s.LEFT_LOPMPowerControl  =  LEFT_LOPM_fully_powered_up;
    p0_r93.s.RIGHT_LOPMPowerControl = RIGHT_LOPM_fully_powered_up;
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
  e_status = tlv320aic33_write_data(65, p0_r65.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_write_data(86, p0_r86.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;
  e_status = tlv320aic33_write_data(93, p0_r93.i_reg);
  if(e_status != GD_SUCCESS)
    return e_status;


  // Save mute value
  s_virtual_reg_img.i_dac_mute = i_mute_flag & 0x01;

  return e_status;
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
 * @param e_source Options are defined in e_CLKDIV_INSourceSelection
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_CLKDIV_IN_source(
    e_CLKDIV_INSourceSelection e_source)
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
 * @param e_source Options are defined in e_PLLCLK_INSourceSelection
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_PLLCLK_IN_source(
    e_PLLCLK_INSourceSelection e_source)
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

/**
 * \brief Get real headphones volume
 * @param p_f_volume Pointer to address where result will be written
 * @return GD_SUCCESS (0) if all right
 */
GD_RES_CODE tlv320aic33_get_headphones_volume_db(float *p_f_volume)
{
  *p_f_volume = s_virtual_reg_img.f_headphones_volume_db;
  return GD_SUCCESS;
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
  s_virtual_reg_img.f_headphones_volume_db = 0;
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
  p0_r0_Page_Select.i_reg = 0;  // Reset all values
  p0_r0_Page_Select.s.PageSelect = 0;   // Set page to 0

  // Save status and check it
  e_status = tlv320aic33_write_data(0,
 
   p0_r0_Page_Select.i_reg);
  if(e_status != GD_SUCCESS)
  {
    return e_status;
  }

  p0_r1_Software_Reset_t p0_r1_SW_res;
  p0_r1_SW_res.i_reg = 0;  // Reset all bits
  p0_r1_SW_res.s.SoftwareReset = 1;     // Set reset to 1

  // Save status and return it
  e_status = tlv320aic33_write_data(1, p0_r1_SW_res.i_reg);
  return e_status;
}



/**
 * \brief Set output driver power on delay
 * @param e_delay Options are defined in e_OutputDriverPowerOnDelayControl
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_output_driver_power_on_delay(
    e_OutputDriverPowerOnDelayControl e_delay)
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
 * @param e_time Options are defined in e_DriverRampUpStepTimingControl
 * @return GD_SUCCESS (0) if all OK
 */
GD_RES_CODE tlv320aic33_set_driver_ramp_up_step_time(
    e_DriverRampUpStepTimingControl e_time)
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
      &s_virtual_reg_img.f_headphones_volume_db);
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
    const uint8_t i_register_number,
    const uint8_t i_value)
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
    if(tlv320aic33_read_float_ro_mem(&TLV320AIC33_volume_table_float[i])
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
    *p_float = tlv320aic33_read_float_ro_mem(
        &TLV320AIC33_volume_table_float[i_raw_volume]);
    return GD_SUCCESS;
  }
}
