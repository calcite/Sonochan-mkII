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
 * \brief Driver for codec TLV320AIC33
 *
 * Created:  2014/04/02\n
 * Modified: 2017/09/15
 *
 * \version 0.4
 * \author Martin Stejskal, Tomas Bajus
 */

#ifndef _TLV320AIC33_H_
#define _TLV320AIC33_H_

//===========================| Included libraries |============================
/* HAL for PLL !!! Please include correct driver for used architecture!
 * Also in included driver should be defined enumeration TLV320AIC33_status_t
 */
#include "tlv320aic33_HAL_AVR32_UC3A3_HW_interface.h"

// Include only if architecture is AVR8
#ifdef __AVR_ARCH__
#include <avr/pgmspace.h>
#endif
//=================================| Options |=================================

/**
 * \name Basic TLV320AIC33 settings
 *
 * When options are 1 (true) and 0 (false) it is highly recommended use\n
 * 1 or 0 instead of true and false
 *
 * @{
 */

/**
 * \brief Support for generic driver support
 *
 * Options: 1 (support enabled) or 0 (support disabled)
 *
 */
#define TLV320AIC33_SUPPORT_GENERIC_DRIVER   1

/**
 * \brief Allow enable FreeRTOS features
 *
 * Options: 0 (disabled) or 1 (enabled). When enabled, then FreeRTOS features\n
 * are enabled.
 */
#define TLV320AIC33_SUPPORT_RTOS             1

/**
 * \brief Allow create mutex mutexI2C when TLV320AIC33_init is called.
 *
 * Mutexes is needed when in RTOS two functions share same hardware. This\n
 * option allow create mutex mutexI2C, so any other functions just can use\n
 * xSemaphoreTake and xSemaphoreGive functions.\n
 * Options: 0 (disabled) or 1 (enabled).\n
 *
 * \note This option is valid only if RTOS support is enabled.
 */
#define TLV320AIC33_RTOS_CREATE_MUTEX        0

/// @}

/// @}


/**
 * \name Advanced TLV320AIC33 settings
 *
 * @{
 */



//===========================| Additional includes |===========================
#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
// If used generic driver, get actual structures
#include "generic_driver.h"
// Also say compiler, that there exist settings on flash
extern const gd_config_struct TLV320AIC33_config_table[];
// And metadata
extern const gd_metadata TLV320AIC33_metadata;

#else
/**
 * \brief Error codes common with generic driver
 *
 * Please note, that this enum MUST be same with as enum in generic driver!
 */
#ifndef _GENERIC_DRIVER_H_
#ifndef GD_RES_CODE_DEFINED

#define GD_RES_CODE_DEFINED
typedef enum{
        GD_SUCCESS =                   0,//!< GD_SUCCESS
        GD_FAIL =                      1,//!< GD_FAIL
        GD_INCORRECT_PARAMETER =       2,//!< GD_INCORRECT_PARAMETER
        GD_INCORRECT_CMD_ID =          3,//!< GD_INCORRECT_CMD_ID
        GD_CMD_ID_NOT_EQUAL_IN_FLASH = 4,//!< GD_CMD_ID_NOT_EQUAL_IN_FLASH
        GD_INCORRECT_DEVICE_ID =       5 //!< GD_INCORRECT_DEVICE_ID
} GD_RES_CODE;
#endif
#endif

#endif

#if TLV320AIC33_SUPPORT_RTOS != 0
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern xSemaphoreHandle mutexI2C;

#endif
//===============================| Structures |================================

/**
 * \name TLV320AIC33_reg_struct Structures for TLV320AIC33 registers
 *
 * Union is compound from uint8_t value and structure. In uint8_t value are\n
 * raw data. Structure allow easily set bit by bit without need apply some\n
 * masks and bit shifts. This make code more clear and also it can save RAM.\n
 * So structure is for settings bits and uint8_t is for write and read\n
 * functions.\n
 * Note that structure can contain more structures or enums.
 *
 * \note Format: p{x}_r{y}_{name}\n
 * x - page (0, 1)\n
 * y - register number\n
 * name - register name
 *
 * @{
 */

// Note, that bit field is endian (and compiler) dependent. So at least try...
#define TLV320AIC33_BIG_ENDIAN_CONDITION                                      \
  ((defined(__BYTE_ORDER__) &&                                                \
   (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)) ||                               \
  defined(__AVR32__))

typedef enum{
	fsref_divided_by_1   = 0,
	fsref_divided_by_1_5 = 1,
	fsref_divided_by_2   = 2,
	fsref_divided_by_2_5 = 3,
	fsref_divided_by_3   = 4,
	fsref_divided_by_3_5 = 5,
	fsref_divided_by_4   = 6,
	fsref_divided_by_4_5 = 7,
	fsref_divided_by_5   = 8,
	fsref_divided_by_5_5 = 9,
	fsref_divided_by_6   = 10,
}e_ADCandDACSampleRate;

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t                       :7;
    uint8_t PageSelect            :1;
  }s;
  uint8_t i_reg;
}p0_r0_Page_Select_t;


typedef union{
  struct
  {
    uint8_t SoftwareReset         :1;
    uint8_t                       :7;
  }s;
  uint8_t i_reg;
}p0_r1_Software_Reset_t;

typedef union{
  struct{
    uint8_t ADCSampleRateSelect   :4;
    uint8_t DACSampleRateSelect   :4;
  }s;
  uint8_t i_reg;
}p0_r2_Codec_Sample_Rate_Select_t;

typedef union{
  struct{
    uint8_t Control               :1;
    uint8_t QValue                :4;
    uint8_t PValue                :3;
  }s;
  uint8_t i_reg;
}p0_r3_PLL_Programming_A_t;

typedef union{
  struct{
    uint8_t JValue                :6;
    uint8_t reserved              :2;
  }s;
  uint8_t i_reg;
}p0_r4_PLL_Programming_B_t;

typedef union{
  struct{
    uint8_t DValueMSB             :8;
  }s;
  uint8_t i_reg;
}p0_r5_PLL_Programming_C_t;

typedef union{
  struct{
    uint8_t DValueLSB             :6;
    uint8_t reserved              :2;
  }s;
  uint8_t i_reg;
}p0_r6_PLL_Programming_D_t;

#else
typedef union{
  struct{
    uint8_t PageSelect            :1;
    uint8_t                       :7;
  }s;
  uint8_t i_reg;
}p0_r0_Page_Select_t;


typedef union{
  struct
  {
    uint8_t                       :7;
    uint8_t SoftwareReset         :1;
  }s;
  uint8_t i_reg;
}p0_r1_Software_Reset_t;

typedef union{
  struct{
    uint8_t DACSampleRateSelect   :4;
    uint8_t ADCSampleRateSelect   :4;
  }s;
  uint8_t i_reg;
}p0_r2_Codec_Sample_Rate_Select_t;

typedef union{
  struct{
    uint8_t PValue                :3;
    uint8_t QValue                :4;
    uint8_t Control               :1;
  }s;
  uint8_t i_reg;
}p0_r3_PLL_Programming_A_t;

typedef union{
  struct{
    uint8_t reserved              :2;
    uint8_t JValue                :6;
  }s;
  uint8_t i_reg;
}p0_r4_PLL_Programming_B_t;

typedef union{
  struct{
    uint8_t DValueMSB             :8;
  }s;
  uint8_t i_reg;
}p0_r5_PLL_Programming_C_t;

typedef union{
  struct{
    uint8_t reserved              :2;
    uint8_t DValueLSB             :6;
  }s;
  uint8_t i_reg;
}p0_r6_PLL_Programming_D_t;
#endif





typedef enum{
  fsref_48khz = 0,
  fsref_44_1khz = 1
}e_FsrefSetting;
typedef enum{
  adc_dual_rate_mode_is_disabled = 0,
  adc_dual_rate_mode_is_enabled = 1
}e_ADCDualRateControl;
typedef enum{
  dac_dual_rate_mode_is_disabled = 0,
  dac_dual_rate_mode_is_enabled = 1
}e_DACDualRateControl;
typedef enum{
  left_dac_datapath_is_off = 0,
  left_dac_datapath_plays_left_channel_input_data = 1,
  left_dac_datapath_plays_right_channel_input_data = 2,
  left_dac_datapath_plays_mono_mix_left_and_right_channel_input_data = 3
}e_LeftDACDatapathControl;
typedef enum
{
  right_dac_datapath_is_off = 0,
  right_dac_datapath_plays_right_channel_input_data = 1,
  right_dac_datapath_plays_left_channel_input_data = 2,
  right_dac_datapath_plays_mono_mix_left_and_right_channel_input_data = 3
}e_RightDACDatapathControl;


#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    e_FsrefSetting FsrefSetting                         :1;
    e_ADCDualRateControl ADCDualRateControl             :1;
    e_DACDualRateControl DACDualRateControl             :1;
    e_LeftDACDatapathControl LeftDACDatapathControl     :2;
    e_RightDACDatapathControl RightDACDatapathControl   :2;
    uint8_t                                             :1;
  }s;
  uint8_t i_reg;
}p0_r7_Codec_Datapath_Setup_t;
#else
typedef union{
  struct{
    uint8_t                                             :1;
    e_RightDACDatapathControl RightDACDatapathControl   :2;
    e_LeftDACDatapathControl LeftDACDatapathControl     :2;
    e_DACDualRateControl DACDualRateControl             :1;
    e_ADCDualRateControl ADCDualRateControl             :1;
    e_FsrefSetting FsrefSetting                         :1;
  }s;
  uint8_t i_reg;
}p0_r7_Codec_Datapath_Setup_t;
#endif




typedef enum{
  bit_clock_is_an_input = 0,
  bit_clock_is_an_output = 1
}e_BitClockDirectionalControl;
typedef enum{
  word_clock_is_an_input = 0,
  word_clock_is_an_output = 1
}e_WordClockDirectionalControl;
typedef enum{
  do_not_3state_DOUT_when_valid_data_is_not_being_send = 0,
  three_state_DOUT_when_valid_data_is_not_being_send = 1
}e_SerialOutputDataDriver3StateControl;
typedef enum{
  bit_clk_and_word_clk_not_transmitt_in_master_mode_if_power_dwn = 0,
  bit_clk_and_word_clk_transmitt_in_master_mode_if_power_dwn = 1
}e_BitWordClockDriveControl;
typedef enum{
  disable_3D_digital_effect = 0,
  enable_3D_digital_effect = 1
}e_3DEffectControl;
typedef enum{
  digital_mic_support_disabled = 0,
  digital_mic_support_enabled_oversampling_128 = 1,
  digital_mic_support_enabled_oversampling_64 = 2,
  digital_mic_support_enabled_oversampling_32 = 3
}e_DigitalMicrophoneFunctionalityControl;


#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    e_BitClockDirectionalControl BitClockDirectionalControl     :1;
    e_WordClockDirectionalControl WordClockDirectionalControl   :1;
    e_SerialOutputDataDriver3StateControl
      SerialOutputDataDriver3StateControl                       :1;
    e_BitWordClockDriveControl BitWordClockDriveControl         :1;
    uint8_t                                                     :1;
    e_3DEffectControl _3DEffectControl                          :1;
    e_DigitalMicrophoneFunctionalityControl
      DigitalMicrophoneFunctionalityControl                     :2;
  }s;
  uint8_t i_reg;
}p0_r8_Audio_Serial_Interface_Control_A_t;
#else
typedef union{
  struct{
    e_DigitalMicrophoneFunctionalityControl
      DigitalMicrophoneFunctionalityControl                     :2;
    e_3DEffectControl _3DEffectControl                          :1;
    uint8_t                                                     :1;
    e_BitWordClockDriveControl BitWordClockDriveControl         :1;
    e_SerialOutputDataDriver3StateControl
      SerialOutputDataDriver3StateControl                       :1;
    e_WordClockDirectionalControl WordClockDirectionalControl   :1;
    e_BitClockDirectionalControl BitClockDirectionalControl     :1;
  }s;
  uint8_t i_reg;
}p0_r8_Audio_Serial_Interface_Control_A_t;
#endif



/**
 * \name Page0_reg9 Page 0, register 9 structures and enums
 *
 * @{
 */
typedef enum{
  serial_data_bus_uses_I2S_mode = 0,            //!< I2S mode
  serial_data_bus_uses_DSP_mode = 1,            //!< DSP mode
  serial_data_bus_uses_right_justified_mode = 2,//!< Right justified mode
  serial_data_bus_uses_left_justified_mode = 3  //!< Left justified mode
}e_AudioSerialDataInterface;
typedef enum{
  audio_data_word_length_16bit = 0,
  audio_data_word_length_20bit = 1,
  audio_data_word_length_24bit = 2,
  audio_data_word_length_32bit = 3
}e_AudioSerialDataWordLengthControl;
typedef enum{
  continuous_transfer_mode = 0,
  clock_256_transfer_mode = 1
}e_BitClockRateControl;
typedef enum{
  re_sync_without_soft_muting = 0,
  re_sync_with_soft_muting = 1
}e_ReSyncMuteBehavior;
#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    e_AudioSerialDataInterface
      AudioSerialDataInterfaceTransferMode                      :2;
    e_AudioSerialDataWordLengthControl
      AudioSerialDataWordLengthControl                          :2;
    e_BitClockRateControl BitClockRateControl                   :1;
    uint8_t DACReSync                                           :1;
    uint8_t ADCReSync                                           :1;
    e_ReSyncMuteBehavior ReSyncMuteBehavior                     :1;
  }s;
  uint8_t i_reg;
}p0_r9_Audio_Serial_Interface_Control_B_t;
#else
typedef union{
  struct{
    e_ReSyncMuteBehavior ReSyncMuteBehavior                     :1;
    uint8_t ADCReSync                                           :1;
    uint8_t DACReSync                                           :1;
    e_BitClockRateControl BitClockRateControl                   :1;
    e_AudioSerialDataWordLengthControl
      AudioSerialDataWordLengthControl                          :2;
    e_AudioSerialDataInterface
      AudioSerialDataInterfaceTransferMode                      :2;
  }s;
  uint8_t i_reg;
}p0_r9_Audio_Serial_Interface_Control_B_t;

#endif
///@}


typedef union{
  struct{
    uint8_t AudioSerialDataWordOffsetControl      :8;
  }s;
  uint8_t i_reg;
}p0_r10_Audio_Serial_Interface_Control_C_t;

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LeftADCOverflowFlag                   :1;
    uint8_t RightADCOverflowFlag                  :1;
    uint8_t LeftDACOverflowFlag                   :1;
    uint8_t RightDACOverflowFlag                  :1;
    uint8_t PLLRValue                             :4;
  }s;
  uint8_t i_reg;
}p0_r11_Audio_Codec_Overflow_Flag_t;
#else
typedef union{
  struct{
    uint8_t PLLRValue                             :4;
    uint8_t RightDACOverflowFlag                  :1;
    uint8_t LeftDACOverflowFlag                   :1;
    uint8_t RightADCOverflowFlag                  :1;
    uint8_t LeftADCOverflowFlag                   :1;
  }s;
  uint8_t i_reg;
}p0_r11_Audio_Codec_Overflow_Flag_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LeftADCHighpassFilterControl          :2;
    uint8_t RightADCHighpassFilterControl         :2;
    uint8_t LeftDACDigitalEffectsFilterControl    :1;
    uint8_t LeftDACDeEmphasisFilterControl        :1;
    uint8_t RightDACDigitalEffectsFilterControl   :1;
    uint8_t RightDACDeEmphasisFilterControl       :1;
  }s;
  uint8_t i_reg;
}p0_r12_Audio_Codec_Digital_Filter_Control_t;
#else
typedef union{
  struct{
    uint8_t RightDACDeEmphasisFilterControl       :1;
    uint8_t RightDACDigitalEffectsFilterControl   :1;
    uint8_t LeftDACDeEmphasisFilterControl        :1;
    uint8_t LeftDACDigitalEffectsFilterControl    :1;
    uint8_t RightADCHighpassFilterControl         :2;
    uint8_t LeftADCHighpassFilterControl          :2;
  }s;
  uint8_t i_reg;
}p0_r12_Audio_Codec_Digital_Filter_Control_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t HeadsetDetectionControl                                       :1;
    uint8_t HeadsetTypeDetectionResults                                   :2;
    uint8_t HeadsetGlitchSuppressionDebounceControlForJackDetection       :3;
    uint8_t HeadsetGlitchSuppressionDebounceControlForButtonPress         :2;
  }s;
  uint8_t i_reg;
}p0_r13_Headset_Button_Press_Detection_A_t;
#else
typedef union{
  struct{
    uint8_t HeadsetGlitchSuppressionDebounceControlForButtonPress         :2;
    uint8_t HeadsetGlitchSuppressionDebounceControlForJackDetection       :3;
    uint8_t HeadsetTypeDetectionResults                                   :2;
    uint8_t HeadsetDetectionControl                                       :1;
  }s;
  uint8_t i_reg;
}p0_r13_Headset_Button_Press_Detection_A_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t DriverCapacitiveCoupling              :1;
    uint8_t StereoOutputDriverConfigurationA      :1;
    uint8_t ButtonPressDetectionFlag              :1;
    uint8_t HeadsetDetectionFlag                  :1;
    uint8_t StereoOutputDriverConfigurationB      :1;
    uint8_t                                       :3;
  }s;
  uint8_t i_reg;
}p0_r14_Headset_Button_Press_Detection_B_t;
#else
typedef union{
  struct{
    uint8_t                                       :3;
    uint8_t StereoOutputDriverConfigurationB      :1;
    uint8_t HeadsetDetectionFlag                  :1;
    uint8_t ButtonPressDetectionFlag              :1;
    uint8_t StereoOutputDriverConfigurationA      :1;
    uint8_t DriverCapacitiveCoupling              :1;
  }s;
  uint8_t i_reg;
}p0_r14_Headset_Button_Press_Detection_B_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LeftADCPGAMute                        :1;
    uint8_t LeftADCPGAGainSetting                 :7;
  }s;
  uint8_t i_reg;
}p0_r15_Left_ADC_PGA_Control_t;
#else
typedef union{
  struct{
    uint8_t LeftADCPGAGainSetting                 :7;
    uint8_t LeftADCPGAMute                        :1;
  }s;
  uint8_t i_reg;
}p0_r15_Left_ADC_PGA_Control_t;

#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t RigthADCPGAMute                        :1;
    uint8_t RightADCPGAGainSetting                 :7;
  }s;
  uint8_t i_reg;
}p0_r16_Right_ADC_PGA_Control_t;
#else
typedef union{
  struct{
    uint8_t RightADCPGAGainSetting                 :7;
    uint8_t RigthADCPGAMute                        :1;
  }s;
  uint8_t i_reg;
}p0_r16_Right_ADC_PGA_Control_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t MIC3LInputLevelControlForLeftADCPGAMix        :4;
    uint8_t MIC3RInputLevelControlForLeftADCPGAMix        :4;
  }s;
  uint8_t i_reg;
}p0_r17_MIC3L_R_To_Left_ADC_Control_t;
#else
typedef union{
  struct{
    uint8_t MIC3RInputLevelControlForLeftADCPGAMix        :4;
    uint8_t MIC3LInputLevelControlForLeftADCPGAMix        :4;
  }s;
  uint8_t i_reg;
}p0_r17_MIC3L_R_To_Left_ADC_Control_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t MIC3LInputLevelControlForRightADCPGAMix       :4;
    uint8_t MIC3RInputLevelControlForRightADCPGAMix       :4;
  }s;
  uint8_t i_reg;
}p0_r18_MIC3L_R_To_Right_ADC_Control_t;
#else
typedef union{
  struct{
    uint8_t MIC3RInputLevelControlForRightADCPGAMix       :4;
    uint8_t MIC3LInputLevelControlForRightADCPGAMix       :4;
  }s;
  uint8_t i_reg;
}p0_r18_MIC3L_R_To_Right_ADC_Control_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LINE1LSingleEndedVsFullyDifferentialControl   :1;
    uint8_t LINE1LInputLevelControlForLeftADCPGAMix       :4;
    uint8_t LeftADCChannelPowerControl                    :1;
    uint8_t LeftADCPGASoftSteppingControl                 :2;
  }s;
  uint8_t i_reg;
}p0_r19_LINE1L_To_Left_ADC_Control_t;
#else
typedef union{
  struct{
    uint8_t LeftADCPGASoftSteppingControl                 :2;
    uint8_t LeftADCChannelPowerControl                    :1;
    uint8_t LINE1LInputLevelControlForLeftADCPGAMix       :4;
    uint8_t LINE1LSingleEndedVsFullyDifferentialControl   :1;
  }s;
  uint8_t i_reg;
}p0_r19_LINE1L_To_Left_ADC_Control_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LINE2LSingleEndedVsFullyDifferentialControl   :1;
    uint8_t LINE2LInputLevelControlForADCPGAMix           :4;
    uint8_t LeftADCChannelWeakCommonModeBiasControl       :1;
    uint8_t                                               :2;
  }s;
  uint8_t i_reg;
}p0_r20_LINE2L_To_Left_ADC_Control_t;
#else
typedef union{
  struct{
    uint8_t                                               :2;
    uint8_t LeftADCChannelWeakCommonModeBiasControl       :1;
    uint8_t LINE2LInputLevelControlForADCPGAMix           :4;
    uint8_t LINE2LSingleEndedVsFullyDifferentialControl   :1;
  }s;
  uint8_t i_reg;
}p0_r20_LINE2L_To_Left_ADC_Control_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LINE1RSingleEndedVsFullyDifferentialControl   :1;
    uint8_t LINE1RInputLevelControlForLeftADCPGAMix       :4;
    uint8_t                                               :3;
  }s;
  uint8_t i_reg;
}p0_r21_LINE1R_To_Left_ADC_Control_t;
#else
typedef union{
  struct{
    uint8_t                                               :3;
    uint8_t LINE1RInputLevelControlForLeftADCPGAMix       :4;
    uint8_t LINE1RSingleEndedVsFullyDifferentialControl   :1;
  }s;
  uint8_t i_reg;
}p0_r21_LINE1R_To_Left_ADC_Control_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LINE1RSingleEndedVsFullyDifferentialControl   :1;
    uint8_t LINE1RInputLevelControlForRightADCPGAMix      :4;
    uint8_t RightADCChannelPowerControl                   :1;
    uint8_t RightADCPGASoftSteppingControl                :2;
  }s;
  uint8_t i_reg;
}p0_r22_LINE1R_To_Right_ADC_Control_t;
#else
typedef union{
  struct{
    uint8_t RightADCPGASoftSteppingControl                :2;
    uint8_t RightADCChannelPowerControl                   :1;
    uint8_t LINE1RInputLevelControlForRightADCPGAMix      :4;
    uint8_t LINE1RSingleEndedVsFullyDifferentialControl   :1;
  }s;
  uint8_t i_reg;
}p0_r22_LINE1R_To_Right_ADC_Control_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LINE2RSingleEndedVsFullyDifferentialControl   :1;
    uint8_t LINE2RInputLevelControlForRightADCPGAMix      :4;
    uint8_t RightADCChannelWeakCommonModeBiasControl      :1;
    uint8_t                                               :2;
  }s;
  uint8_t i_reg;
}p0_r23_LINE2R_To_Right_ADC_Control_t;
#else
typedef union{
  struct{
    uint8_t                                               :2;
    uint8_t RightADCChannelWeakCommonModeBiasControl      :1;
    uint8_t LINE2RInputLevelControlForRightADCPGAMix      :4;
    uint8_t LINE2RSingleEndedVsFullyDifferentialControl   :1;
  }s;
  uint8_t i_reg;
}p0_r23_LINE2R_To_Right_ADC_Control_t;

#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LINE1LSingleEndedVsFullyDifferential          :1;
    uint8_t LINE1LInputLevelControlForRightADCPGAMix      :4;
    uint8_t                                               :3;
  }s;
  uint8_t i_reg;
}p0_r24_LINE1L_To_Right_ADC_Control_t;
#else
typedef union{
  struct{
    uint8_t                                               :3;
    uint8_t LINE1LInputLevelControlForRightADCPGAMix      :4;
    uint8_t LINE1LSingleEndedVsFullyDifferential          :1;
  }s;
  uint8_t i_reg;
}p0_r24_LINE1L_To_Right_ADC_Control_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t MICBIASLevelControl                           :2;
    uint8_t                                               :3;
    uint8_t                                               :3;
  }s;
  uint8_t i_reg;
}p0_r25_MICBIAS_Control_t;
#else
typedef union{
  struct{
    uint8_t                                               :3;
    uint8_t                                               :3;
    uint8_t MICBIASLevelControl                           :2;
  }s;
  uint8_t i_reg;
}p0_r25_MICBIAS_Control_t;
#endif



//registers 26 - 35    automatic gain control
//


typedef enum{
	applied_and_programmed_gain_are_not_equal = 0,
	applied_gain_is_equal_to_programmed_gain = 1
}e_ADCPGAStatus;

typedef enum{
	signal_power_is_greather_than_noise_treshold = 0,
	signal_power_is_less_than_noise_treshold = 0,
}e_AGCSignalDetectionStatus;

typedef enum{
	AGC_is_not_saturated = 0,
	AGC_gain_applied = 1
}e_AGCSaturationFlag;

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    e_ADCPGAStatus LeftADCPGAStatus              	        :1;
    uint8_t LeftADCPowerStatus                            :1;
    e_AGCSignalDetectionStatus
      LeftAGCSignalDetectionStatus                        :1;
    e_AGCSaturationFlag LeftAGCSaturationFlag             :1;
    e_ADCPGAStatus RightADCPGAStatus              				:1;
    uint8_t RightADCPowerStatus                           :1;
    e_AGCSignalDetectionStatus
      RightAGCSignalDetectionStatus                 			:1;
    e_AGCSaturationFlag RightAGCSaturationFlag            :1;
  }s;
  uint8_t i_reg;
}p0_r36_ADC_Flag_Register_t;
#else
typedef union{
  struct{
  	e_AGCSaturationFlag RightAGCSaturationFlag            :1;
  	e_AGCSignalDetectionStatus
  		RightAGCSignalDetectionStatus                 			:1;
  	uint8_t RightADCPowerStatus                           :1;
  	e_ADCPGAStatus RightADCPGAStatus              				:1;
  	e_AGCSaturationFlag LeftAGCSaturationFlag             :1;
  	e_AGCSignalDetectionStatus
  	  LeftAGCSignalDetectionStatus                        :1;
  	uint8_t LeftADCPowerStatus                            :1;
  	e_ADCPGAStatus LeftADCPGAStatus              	        :1;
  }s;
  uint8_t i_reg;
}p0_r36_ADC_Flag_Register_t;
#endif





typedef enum{
  HPLCOM_is_differential_of_HPLOUT = 0,
  HPLCOM_is_constant_VCM = 1,
  HPLCOM_is_single_ended = 2
}e_HPLCOMOutputDriverConfigurationControl;

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LeftDACPower                                  :1;
    uint8_t RightDACPower                                 :1;
    e_HPLCOMOutputDriverConfigurationControl
      HPLCOMOutputDriverConfigurationControl              :2;
    uint8_t                                               :4;
  }s;
  uint8_t i_reg;
}p0_r37_DAC_Power_And_Output_Driver_Control_t;
#else
typedef union{
  struct{
    uint8_t                                               :4;
    e_HPLCOMOutputDriverConfigurationControl
      HPLCOMOutputDriverConfigurationControl              :2;
    uint8_t RightDACPower                                 :1;
    uint8_t LeftDACPower                                  :1;
  }s;
  uint8_t i_reg;
}p0_r37_DAC_Power_And_Output_Driver_Control_t;
#endif

typedef enum{
  HPRCOM_is_differential_of_HPROUT = 0,
  HPRCOM_is_constant_VCM = 1,
  HPRCOM_is_single_ended = 2,
  HPRCOM_is_differential_of_HPLCOM = 3,
  HPRCOM_is_external_feedback_with_HPLCOM_as_VCM_output = 4
}e_HPRCOMOutputDriverConfigurationControl;
typedef enum{
  if_short_protect_en_limit_max_current = 0,
  if_short_protect_en_power_down_when_short = 1
}e_ShortCorcuitProtectionModeControl;
#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t                                               :2;
    e_HPRCOMOutputDriverConfigurationControl
      HPRCOMOutputDriverConfigurationControl              :3;
    uint8_t ShortCircuitProtection                        :1;
    e_ShortCorcuitProtectionModeControl
      ShortCorcuitProtectionModeControl                   :1;
    uint8_t                                               :1;
  }s;
  uint8_t i_reg;
}p0_r38_High_Power_Output_Driver_Control_t;
#else
typedef union{
  struct{
    uint8_t                                               :1;
    e_ShortCorcuitProtectionModeControl
      ShortCorcuitProtectionModeControl                   :1;
    uint8_t ShortCircuitProtection                        :1;
    e_HPRCOMOutputDriverConfigurationControl
      HPRCOMOutputDriverConfigurationControl              :3;
    uint8_t                                               :2;
  }s;
  uint8_t i_reg;
}p0_r38_High_Power_Output_Driver_Control_t;
#endif

typedef union{
  struct{
    uint8_t                                               :8;
  }s;
  uint8_t i_reg;
}p0_r39_Reserved_t;


#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t OutputCommonModeVoltageControl                :2;
    uint8_t LINE2LBypassPathControl                       :2;
    uint8_t LINE2RBypassPathControl                       :2;
    uint8_t OutputVolumeControlSoftStepping               :2;
  }s;
  uint8_t i_reg;
}p0_r40_High_Power_Output_Stage_Control_t;
#else
typedef union{
  struct{
    uint8_t OutputVolumeControlSoftStepping               :2;
    uint8_t LINE2RBypassPathControl                       :2;
    uint8_t LINE2LBypassPathControl                       :2;
    uint8_t OutputCommonModeVoltageControl                :2;
  }s;
  uint8_t i_reg;
}p0_r40_High_Power_Output_Stage_Control_t;
#endif



typedef enum{
  left_DAC_output_DAC_L1_path = 0,
  left_DAC_output_DAC_L3_path = 1,
  left_DAC_output_DAC_L2_path = 2
}e_LeftDACOutputSwitchingControl;
typedef enum{
  right_DAC_output_DAC_R1_path = 0,
  right_DAC_output_DAC_R3_path = 1,
  right_DAC_output_DAC_R2_path = 2
}e_RightDACOutputSwitchingControl;
typedef enum{
  left_and_right_DAC_channels_independent_volume = 0,
  left_DAC_channel_follows_right_channel = 1,
  right_DAC_channel_follows_left_channel = 2
}e_DACDigitalVolumeControlFunctionality;
#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    e_LeftDACOutputSwitchingControl
      LeftDACOutputSwitchingControl                       :2;
    e_RightDACOutputSwitchingControl
      RightDACOutputSwitchingControl                      :2;
    uint8_t                                               :2;
    e_DACDigitalVolumeControlFunctionality
      DACDigitalVolumeControlFunctionality                :2;
  }s;
  uint8_t i_reg;
}p0_r41_DAC_Output_Switching_Control_t;
#else
typedef union{
  struct{
    e_DACDigitalVolumeControlFunctionality
      DACDigitalVolumeControlFunctionality                :2;
    uint8_t                                               :2;
    e_RightDACOutputSwitchingControl
      RightDACOutputSwitchingControl                      :2;
    e_LeftDACOutputSwitchingControl
      LeftDACOutputSwitchingControl                       :2;
  }s;
  uint8_t i_reg;
}p0_r41_DAC_Output_Switching_Control_t;
#endif





typedef enum{
  driver_power_on_time_0us = 0,
  driver_power_on_time_10us = 1,
  driver_power_on_time_100us = 2,
  driver_power_on_time_1ms = 3,
  driver_power_on_time_10ms = 4,
  driver_power_on_time_50ms = 5,
  driver_power_on_time_100ms = 6,
  driver_power_on_time_200ms = 7,
  driver_power_on_time_400ms = 8,
  driver_power_on_time_800ms = 9,
  driver_power_on_time_2s = 10,
  driver_power_on_time_4s = 11
}e_OutputDriverPowerOnDelayControl;
typedef enum{
  driver_ramp_up_step_time_0ms = 0,
  driver_ramp_up_step_time_1ms = 1,
  driver_ramp_up_step_time_2ms = 2,
  driver_ramp_up_step_time_4ms = 3
}e_DriverRampUpStepTimingControl;
typedef enum{
  common_output_voltage_generated_from_resistor_divider = 0,
  common_output_voltage_generated_from_bandgap_reference = 1
}e_WeakOutputCommonModeVoltageControl;

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union
{
  struct{
    e_OutputDriverPowerOnDelayControl
      OutputDriverPowerOnDelayControl                     :4;
    e_DriverRampUpStepTimingControl
      DriverRampUpStepTimingControl                       :2;
    e_WeakOutputCommonModeVoltageControl
      WeakOutputCommonModeVoltageControl                  :1;
    uint8_t                                               :1;
  }s;
  uint8_t i_reg;
}p0_r42_Output_Driver_Pop_Reduction_t;
#else
typedef union
{
  struct{
    uint8_t                                               :1;
    e_WeakOutputCommonModeVoltageControl
      WeakOutputCommonModeVoltageControl                  :1;
    e_DriverRampUpStepTimingControl
        DriverRampUpStepTimingControl                       :2;
    e_OutputDriverPowerOnDelayControl
      OutputDriverPowerOnDelayControl                     :4;
  }s;
  uint8_t i_reg;
}p0_r42_Output_Driver_Pop_Reduction_t;

#endif


#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LeftDACDigitalMute                            :1;
    uint8_t LeftDACDigitalVolumeControlSetting            :7;
  }s;
  uint8_t i_reg;
}p0_r43_Left_DAC_Digital_Volume_Control_t;
#else
typedef union{
  struct{
    uint8_t LeftDACDigitalVolumeControlSetting            :7;
    uint8_t LeftDACDigitalMute                            :1;
  }s;
  uint8_t i_reg;
}p0_r43_Left_DAC_Digital_Volume_Control_t;
#endif

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t RightDACDigitalMute                           :1;
    uint8_t RightDACDigitalVolumeControlSetting           :7;
  }s;
  uint8_t i_reg;
}p0_r44_Right_DAC_Digital_Volume_Control_t;
#else
typedef union{
  struct{
    uint8_t RightDACDigitalVolumeControlSetting           :7;
    uint8_t RightDACDigitalMute                           :1;
  }s;
  uint8_t i_reg;
}p0_r44_Right_DAC_Digital_Volume_Control_t;
#endif



#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LINE2LRouteToHPLOUTEnable                    :1;
    uint8_t LINE2LRouteToHPLOUTVolume                    :7;
  }s;
  uint8_t i_reg;
}p0_r45_LINE2L_To_HPLOUT_Volume_Control_t;
#else
typedef union{
  struct{
    uint8_t LINE2LRouteToHPLOUTVolume                    :7;
    uint8_t LINE2LRouteToHPLOUTEnable                    :1;
  }s;
  uint8_t i_reg;
}p0_r45_LINE2L_To_HPLOUT_Volume_Control_t;
#endif



#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t PGALRouteToHPLOUTEnable                    :1;
    uint8_t PGALRouteToHPLOUTVolume                    :7;
  }s;
  uint8_t i_reg;
}p0_r46_PGA_L_To_HPLOUT_Volume_Control_t;
#else
typedef union{
  struct{
    uint8_t PGALRouteToHPLOUTVolume                    :7;
    uint8_t PGALRouteToHPLOUTEnable                    :1;
  }s;
  uint8_t i_reg;
}p0_r46_PGA_L_To_HPLOUT_Volume_Control_t;
#endif




#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t DACL1RouteToHPLOUTEnable                    :1;
    uint8_t DACL1RouteToHPLOUTVolume                    :7;
  }s;
  uint8_t i_reg;
}p0_r47_DAC_L1_To_HPLOUT_Volume_Control_t;
#else
typedef union{
  struct{
    uint8_t DACL1RouteToHPLOUTVolume                    :7;
    uint8_t DACL1RouteToHPLOUTEnable                    :1;
  }s;
  uint8_t i_reg;
}p0_r47_DAC_L1_To_HPLOUT_Volume_Control_t;
#endif







typedef enum{
  HPLOUT_is_weakly_driven_to_common_mode_when_pwr_dwn = 0,
  HPLOUT_is_3_stated_with_pwr_dwn = 1
}e_HPLOUTPowerDownDriveControl;
typedef enum{
  all_programmed_gains_to_HPLOUT_have_been_applied_yet = 0,
  not_all_programmed_gains_to_HPLOUT_have_been_applied_yet = 1
}e_HPLOUTVolumeControlStatus;
#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t HPLOUTOutputLevelControl                    :4;
    uint8_t HPLOUTMute                                  :1;
    e_HPLOUTPowerDownDriveControl
      HPLOUTPowerDownDriveControl                       :1;
    e_HPLOUTVolumeControlStatus
      HPLOUTVolumeControlStatus                         :1;
    uint8_t HPLOUTFullyPoweredUp                        :1;
  }s;
  uint8_t i_reg;
}p0_r51_HPLOUT_Output_Level_Control_t;
#else
typedef union{
  struct{
    uint8_t HPLOUTFullyPoweredUp                        :1;
    e_HPLOUTVolumeControlStatus
      HPLOUTVolumeControlStatus                         :1;
    e_HPLOUTPowerDownDriveControl
      HPLOUTPowerDownDriveControl                       :1;
    uint8_t HPLOUTMute                                  :1;
    uint8_t HPLOUTOutputLevelControl                    :4;
  }s;
  uint8_t i_reg;
}p0_r51_HPLOUT_Output_Level_Control_t;
#endif


#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t LINE2RRouteToHPROUTEnable                    :1;
    uint8_t LINE2RRouteToHPROUTVolume                    :7;
  }s;
  uint8_t i_reg;
}p0_r62_LINE2R_To_HPROUT_Volume_Control_t;
#else
typedef union{
  struct{
    uint8_t LINE2RRouteToHPROUTVolume                    :7;
    uint8_t LINE2RRouteToHPROUTEnable                    :1;
  }s;
  uint8_t i_reg;
}p0_r62_LINE2R_To_HPROUT_Volume_Control_t;
#endif


#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t DACR1RouteToHPROUTEnable                    :1;
    uint8_t DACR1RouteToHPROUTVolume                    :7;
  }s;
  uint8_t i_reg;
}p0_r64_DAC_R1_To_HPROUT_Volume_Control_t;
#else
typedef union{
  struct{
    uint8_t DACR1RouteToHPROUTVolume                    :7;
    uint8_t DACR1RouteToHPROUTEnable                    :1;
  }s;
  uint8_t i_reg;
}p0_r64_DAC_R1_To_HPROUT_Volume_Control_t;

#endif









typedef enum{
  HPROUT_is_weakly_driven_to_common_mode_when_pwr_dwn = 0,
  HPROUT_is_3_stated_with_pwr_dwn = 1
}e_HPROUTPowerDownDriveControl;
typedef enum{
  all_programmed_gains_to_HPROUT_have_been_applied_yet = 0,
  not_all_programmed_gains_to_HPROUT_have_been_applied_yet = 1
}e_HPROUTVolumeControlStatus;

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t HPROUTOutputLevelControl                    :4;
    uint8_t HPROUTMute                                  :1;
    e_HPROUTPowerDownDriveControl
      HPROUTPowerDownDriveControl                       :1;
    e_HPROUTVolumeControlStatus
      HPROUTVolumeControlStatus                         :1;
    uint8_t HPROUTFullyPoweredUp                        :1;
  }s;
  uint8_t i_reg;
}p0_r65_HPROUT_Output_Level_Control_t;
#else
typedef union{
  struct{
    uint8_t HPROUTFullyPoweredUp                        :1;
    e_HPROUTVolumeControlStatus
      HPROUTVolumeControlStatus                         :1;
    e_HPROUTPowerDownDriveControl
      HPROUTPowerDownDriveControl                       :1;
    uint8_t HPROUTMute                                  :1;
    uint8_t HPROUTOutputLevelControl                    :4;
  }s;
  uint8_t i_reg;
}p0_r65_HPROUT_Output_Level_Control_t;
#endif




typedef enum{
  DAC_L1_not_routed_to_LEFT_LOPM = 0,
  DAC_L1_routed_to_LEFT_LOPM = 1
}e_DAC_L1OutputRoutingControl;
#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    e_DAC_L1OutputRoutingControl
      DAC_L1OutputRoutingControl                        :1;
    uint8_t DAC_L1ToLEFT_LOPMAnalogVolumeControl        :7;
  }s;
  uint8_t i_reg;
}p0_r82_DAC_L1_To_LEFT_LOPM_Volume_Control_t;
#else
typedef union{
  struct{
    uint8_t DAC_L1ToLEFT_LOPMAnalogVolumeControl        :7;
    e_DAC_L1OutputRoutingControl
      DAC_L1OutputRoutingControl                        :1;
  }s;
  uint8_t i_reg;
}p0_r82_DAC_L1_To_LEFT_LOPM_Volume_Control_t;
#endif


typedef enum{
  LEFT_LOPM_Output_level_0dB = 0,
  LEFT_LOPM_Output_level_1dB = 1,
  LEFT_LOPM_Output_level_2dB = 2,
  LEFT_LOPM_Output_level_3dB = 3,
  LEFT_LOPM_Output_level_4dB = 4,
  LEFT_LOPM_Output_level_5dB = 5,
  LEFT_LOPM_Output_level_6dB = 6,
  LEFT_LOPM_Output_level_7dB = 7,
  LEFT_LOPM_Output_level_8dB = 8,
  LEFT_LOPM_Output_level_9dB = 9
}e_LEFT_LOPMOutputLevel;
typedef enum{
  LEFT_LOPM_all_prog_gains_to_LEFT_LOPM_applied = 0,
  LEFT_LOPM_not_all_prog_gains_to_LEFT_LOPM_applied_yet = 1
}e_LEFT_LOPMVolumeControlStatus;
typedef enum{
  LEFT_LOPM_not_fully_powered_up = 0,
  LEFT_LOPM_fully_powered_up = 1,
}e_LEFT_LOPMPowerControl;
#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    e_LEFT_LOPMOutputLevel
      LEFT_LOPMOutputLevel                              :4;
    uint8_t LEFT_LOPMMute                               :1;
    uint8_t                                             :1;
    e_LEFT_LOPMVolumeControlStatus
      LEFT_LOPMVolumeControlStatus                      :1;
    e_LEFT_LOPMPowerControl
      LEFT_LOPMPowerControl                             :1;
  }s;
  uint8_t i_reg;
}p0_r86_LEFT_LOPM_Output_Level_Control_Register;
#else
typedef union{
  struct{
    e_LEFT_LOPMPowerControl
      LEFT_LOPMPowerControl                             :1;
    e_LEFT_LOPMVolumeControlStatus
      LEFT_LOPMVolumeControlStatus                      :1;
    uint8_t                                             :1;
    uint8_t LEFT_LOPMMute                               :1;
    e_LEFT_LOPMOutputLevel
      LEFT_LOPMOutputLevel                              :4;
  }s;
  uint8_t i_reg;
}p0_r86_LEFT_LOPM_Output_Level_Control_Register;
#endif





typedef enum{
  DAC_R1_not_routed_to_RIGHT_LOPM = 0,
  DAC_R1_routed_to_RIGHT_LOPM = 1
}e_DAC_R1OutputRoutingControl;
#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    e_DAC_R1OutputRoutingControl
      DAC_R1OutputRoutingControl                        :1;
    uint8_t DAC_R1ToRIGHT_LOPMAnalogVolumeControl       :7;
  }s;
  uint8_t i_reg;
}p0_r92_DAC_R1_To_RIGHT_LOPM_Volume_Control_t;
#else
typedef union{
  struct{
    uint8_t DAC_R1ToRIGHT_LOPMAnalogVolumeControl       :7;
    e_DAC_R1OutputRoutingControl
      DAC_R1OutputRoutingControl                        :1;
  }s;
  uint8_t i_reg;
}p0_r92_DAC_R1_To_RIGHT_LOPM_Volume_Control_t;
#endif










typedef enum{
  RIGHT_LOPM_Output_level_0dB = 0,
  RIGHT_LOPM_Output_level_1dB = 1,
  RIGHT_LOPM_Output_level_2dB = 2,
  RIGHT_LOPM_Output_level_3dB = 3,
  RIGHT_LOPM_Output_level_4dB = 4,
  RIGHT_LOPM_Output_level_5dB = 5,
  RIGHT_LOPM_Output_level_6dB = 6,
  RIGHT_LOPM_Output_level_7dB = 7,
  RIGHT_LOPM_Output_level_8dB = 8,
  RIGHT_LOPM_Output_level_9dB = 9
}e_RIGHT_LOPMOutputLevel;
typedef enum{
  RIGHT_LOPM_all_prog_gains_to_RIGHT_LOPM_applied = 0,
  RIGHT_LOPM_not_all_prog_gains_to_RIGHT_LOPM_applied_yet = 1
}e_RIGHT_LOPMVolumeControlStatus;
typedef enum{
  RIGHT_LOPM_not_fully_powered_up = 0,
  RIGHT_LOPM_fully_powered_up = 1,
}e_RIGHT_LOPMPowerControl;
#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    e_RIGHT_LOPMOutputLevel
      RIGHT_LOPMOutputLevel                             :4;
    uint8_t RIGHT_LOPMMute                              :1;
    uint8_t                                             :1;
    e_RIGHT_LOPMVolumeControlStatus
      RIGHT_LOPMVolumeControlStatus                     :1;
    e_RIGHT_LOPMPowerControl
      RIGHT_LOPMPowerControl                            :1;
  }s;
  uint8_t i_reg;
}p0_r93_RIGHT_LOPM_Output_Level_Control_Register;
#else
typedef union{
  struct{
    e_RIGHT_LOPMPowerControl
      RIGHT_LOPMPowerControl                            :1;
    e_RIGHT_LOPMVolumeControlStatus
      RIGHT_LOPMVolumeControlStatus                     :1;
    uint8_t                                             :1;
    uint8_t RIGHT_LOPMMute                              :1;
    e_RIGHT_LOPMOutputLevel
      RIGHT_LOPMOutputLevel                             :4;
  }s;
  uint8_t i_reg;
}p0_r93_RIGHT_LOPM_Output_Level_Control_Register;
#endif


typedef enum{
	CODEC_CLKIN_uses_PLLDIV_OUT = 0,
	CODEC_CLKIN_uses_CLKDIV_OUT = 1,
}e_CODEC_CLKINSourceSel;

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    uint8_t i_MFPControl                                  			:6;
    e_CODEC_CLKINSourceSel CODEC_CLKINSourceSelection     :2;
  }s;
  uint8_t i_reg;
}p0_r101_Additional_GPIO_Control_t;
#else
typedef union{
  struct{
    e_CODEC_CLKINSourceSel CODEC_CLKINSourceSelection     :2;
    uint8_t i_MFPControl                                  			:6;
  }s;
  uint8_t i_reg;
}p0_r101_Additional_GPIO_Control_t;
#endif


typedef enum{
  CLKDIV_IN_uses_MCLK = 0,
  CLKDIV_IN_uses_GPIO2 = 1,
  CLKDIV_IN_uses_BCLK = 2
}e_CLKDIV_INSourceSel;
typedef enum{
  PLLCLK_IN_uses_MCLK = 0,
  PLLCLK_IN_uses_GPIO2 = 1,
  PLLCLK_IN_uses_BLCK = 2
}e_PLLCLK_INSourceSel;

#if TLV320AIC33_BIG_ENDIAN_CONDITION
typedef union{
  struct{
    e_CLKDIV_INSourceSel
      CLKDIV_INSourceSelection                                  :2;
    e_PLLCLK_INSourceSel PLLCLK_INSourceSelection         :2;
    // 0b0000 -> 16 ; 0b0001 -> 17 ; 0b0010 -> 2 ; 0b0011 -> 3 ; 0b1111 -> 15
    uint8_t PLLClockDividerNValue                               :4;
  }s;
  uint8_t i_reg;
}p0_r102_Clock_Generation_Control_t;
#else
typedef union{
  struct{
    // 0b0000 -> 16 ; 0b0001 -> 17 ; 0b0010 -> 2 ; 0b0011 -> 3 ; 0b1111 -> 15
    uint8_t PLLClockDividerNValue                               :4;
    e_PLLCLK_INSourceSel PLLCLK_INSourceSelection         :2;
    e_CLKDIV_INSourceSel
      CLKDIV_INSourceSelection                                  :2;
  }s;
  uint8_t i_reg;
}p0_r102_Clock_Generation_Control_t;

#endif
///@}

typedef enum{
  loss_0_0db = 0,
  loss_1_5db = 1,
  loss_3_0db = 2,
  loss_4_5db = 3,
  loss_6_0db = 4,
  loss_7_5db = 5,
  loss_9_0db = 6,
  loss_10_5db = 7,
  loss_12_0db = 8,
  disconnected = 15
}e_ADCInputGain;


#if TLV320AIC33_SUPPORT_GENERIC_DRIVER != 0
// This structure have meaning only if generic driver is enabled
typedef struct{
  // Headphones volume level in dB // actually DAC1 to HP to routing
  float f_headphones_volume_db;

  // LINE2 L routing to HPLOUT
  float f_line2l_to_hplout_db;

  // LINE2 R routing to HPROUT
  float f_line2r_to_hprout_db;

  // DAC volume level in dB
  float f_dac_volume_db;

  // Info if DAC play stereo or mono
  uint8_t i_dac_play_stereo;

  // Digital interface master(BLCK,WCLK out)/slave(BLCK,WCLK in) mode
  uint8_t i_digital_interface_master_slave;

  // Digital interface data mode 0-I2S 1-DSP 2-right justified 3-left justified
  uint8_t i_digital_inteface_data_mode;

  // Word length (16, 20, 24, 32)
  uint8_t i_word_length;

  // BCLK rate control
  uint8_t i_bclk_rate_control;

  // Audio serial data offset
  uint8_t i_audio_serial_data_offset;

  // CODEC_CLKINSourceSelection
  e_CODEC_CLKINSourceSel e_codec_clkin_source;

  // CLKDIV_INSourceSelection
  e_CLKDIV_INSourceSel e_clkdiv_in_source;

  // PLLCLK_INSourceSelection
  e_PLLCLK_INSourceSel e_pllclk_in_source;

  // PLL divider value
  uint8_t i_pll_clock_divider_N_value;

  // PLL control bit (enable pll)
  uint8_t i_pll_enable;

  // PLL Q value
  uint8_t i_pll_q_value;

  // PLL P value
  uint8_t i_pll_p_value;

  // PLL J value
  uint8_t i_pll_j_value;

  // PLL D value
  uint16_t i_pll_d_value;

  // PLL R value
  uint8_t i_pll_r_value;

  // Fsref setting
  e_FsrefSetting e_fsref;

  // ADC sample rate select
  e_ADCandDACSampleRate e_adc_sample_rate;

  // DAC sample rate select
  e_ADCandDACSampleRate e_dac_sample_rate;

  // ADC and DAC dual rate control
  e_ADCDualRateControl e_adc_dac_dual_rate;

  // Headphone driver output
  uint8_t i_headphones_output_single_ended;

  // DAC mute
  uint8_t i_dac_mute;

  // ADC mute
  uint8_t i_adc_mute;

  // DAC power
  uint8_t i_dac_power;

  // ADC power
  uint8_t i_adc_power;

  // HP_OUT power
  uint8_t i_hp_out_power;

  // HP_OUT mute
  uint8_t i_hp_out_mute;

  // ADC gain dB
  float f_adc_gain_db;

  // left DAC output switching control
  uint8_t i_left_DAC_output;

  // right DAC output switching control
  uint8_t i_right_DAC_output;

  // MIC3R to ADC Right gain
  e_ADCInputGain e_mic3r_to_adc_right_gain;

  // MIC3L to ADC Right gain
  e_ADCInputGain e_mic3l_to_adc_right_gain;

  // LINE2R to ADC Right gain
  e_ADCInputGain e_line2r_to_adc_right_gain;

  // LINE1R to ADC Right gain
  e_ADCInputGain e_line1r_to_adc_right_gain;

  // LINE1L to ADC Right gain
  e_ADCInputGain e_line1l_to_adc_right_gain;

  // MIC3R to ADC Left gain
  e_ADCInputGain e_mic3r_to_adc_left_gain;

  // MIC3L to ADC Left gain
  e_ADCInputGain e_mic3l_to_adc_left_gain;

  // LINE2L to ADC Left gain
  e_ADCInputGain e_line2l_to_adc_left_gain;

  // LINE1R to ADC Left gain
  e_ADCInputGain e_line1r_to_adc_left_gain;

  // LINE1L to ADC Left gain
  e_ADCInputGain e_line1l_to_adc_left_gain;

  // LINE2L bypass
  uint8_t i_line2l_bypass_enable;

  // LINE2R bypass
  uint8_t i_line2r_bypass_enable;

  // module power status register
  uint8_t i_module_power_status_reg;

  // ADC flag register
  uint8_t i_adc_flag_reg;

}tlv320aic33_virtual_reg_img_t;

#endif

//===============================| Definitions |===============================
//=================================| Macros |==================================
// If AVR8 architecture
#if defined(__AVR_ARCH__) || defined(__AVR__)
#define tlv320aic33_read_byte_ro_mem(p_address)     pgm_read_byte(p_address)
#define tlv320aic33_read_word_ro_mem(p_address)     pgm_read_word(p_address)
#define tlv320aic33_read_dword_ro_mem(p_address)    pgm_read_dword(p_address)
#define tlv320aic33_read_float_ro_mem(p_address)    pgm_read_float(p_address)
#else   // Else different architecture
#define tlv320aic33_read_byte_ro_mem(p_address)     *(p_address)
#define tlv320aic33_read_word_ro_mem(p_address)     *(p_address)
#define tlv320aic33_read_dword_ro_mem(p_address)    *(p_address)
#define tlv320aic33_read_float_ro_mem(p_address)    *(p_address)
#endif

/**
 * \brief Define lock function when support for RTOS is enabled
 *
 * If TLV320AIC33_SUPPORT_RTOS is enabled, then there is defined function, that\n
 * "lock" TWI device, so any other device can not use it.\n
 * If TLV320AIC33_SUPPORT_RTOS is disabled, there is defined just void macro.
 */
#if TLV320AIC33_SUPPORT_RTOS != 0
  #define TLV320AIC33_LOCK_TWI_MODULE_IF_RTOS    \
    xSemaphoreTake( mutexI2C, portMAX_DELAY );
#else
  #define TLV320AIC33_LOCK_TWI_MODULE_IF_RTOS
#endif


#if TLV320AIC33_SUPPORT_RTOS != 0
  #define TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS        \
    xSemaphoreGive( mutexI2C );
#else
  #define TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS
#endif



//==========================| High level functions |===========================

GD_RES_CODE tlv320aic33_init(void);

//  clock settings

GD_RES_CODE tlv320aic33_set_PLL_enable(uint8_t i_enable);
GD_RES_CODE tlv320aic33_set_PLL_Q_value(uint8_t i_q_val);
GD_RES_CODE tlv320aic33_set_PLL_P_value(uint8_t i_p_val);
GD_RES_CODE tlv320aic33_set_PLL_J_value(uint8_t i_j_val);
GD_RES_CODE tlv320aic33_set_PLL_D_value(uint16_t i_d_val);
GD_RES_CODE tlv320aic33_set_Fsref(e_FsrefSetting e_setting);
GD_RES_CODE tlv320aic33_set_PLL_R_value(uint8_t i_r_val);
GD_RES_CODE tlv320aic33_set_CODEC_CLKIN_source(e_CODEC_CLKINSourceSel e_source);
GD_RES_CODE tlv320aic33_set_CLKDIV_IN_source(e_CLKDIV_INSourceSel e_source);
GD_RES_CODE tlv320aic33_set_PLLCLK_IN_source(e_PLLCLK_INSourceSel e_source);
GD_RES_CODE tlv320aic33_set_PLL_clock_divider_N_value(uint8_t i_pll_divider);

//  digital interface settings

GD_RES_CODE tlv320aic33_set_digital_interface_mode(uint8_t i_master);
GD_RES_CODE tlv320aic33_set_data_interface(e_AudioSerialDataInterface e_mode);
GD_RES_CODE tlv320aic33_get_data_interface(
                                         e_AudioSerialDataInterface *p_e_mode);
GD_RES_CODE tlv320aic33_set_word_length(uint8_t i_word_length);
GD_RES_CODE tlv320aic33_set_BCLK_rate(e_BitClockRateControl e_bclk_rate);
GD_RES_CODE tlv320aic33_get_BCLK_rate(e_BitClockRateControl *p_e_bclk_rate);
GD_RES_CODE tlv320aic33_set_data_offset(uint8_t i_offset);

//  ADC and DAC settings

GD_RES_CODE tlv320aic33_set_ADC_power(uint8_t enable_ADCs);
GD_RES_CODE tlv320aic33_set_DAC_power(uint8_t enable_DACs);
GD_RES_CODE tlv320aic33_set_ADC_mute(uint8_t i_mute_flag);
GD_RES_CODE tlv320aic33_set_DAC_mute(uint8_t i_mute_flag);
GD_RES_CODE tlv320aic33_set_ADC_sample_rate(e_ADCandDACSampleRate e_samplerate);
GD_RES_CODE tlv320aic33_set_DAC_sample_rate(e_ADCandDACSampleRate e_samplerate);
GD_RES_CODE tlv320aic33_set_ADC_and_DAC_dual_rate(uint8_t i_enable);
GD_RES_CODE tlv320aic33_set_DAC_play_input_data(uint8_t i_play_stereo);//todo move to higher layer
GD_RES_CODE tlv320aic33_set_DAC_left_output_routing(uint8_t i_dac_output);
GD_RES_CODE tlv320aic33_set_DAC_right_output_routing(uint8_t i_dac_output);


//  signal routing

GD_RES_CODE tlv320aic33_set_ADC_L_from_LINE2_L(e_ADCInputGain e_gain);
GD_RES_CODE tlv320aic33_set_ADC_R_from_LINE2_R(e_ADCInputGain e_gain);
GD_RES_CODE tlv320aic33_set_ADC_L_from_LINE1_L(e_ADCInputGain e_gain);
GD_RES_CODE tlv320aic33_set_ADC_L_from_LINE1_R(e_ADCInputGain e_gain);
GD_RES_CODE tlv320aic33_set_ADC_L_from_MIC3_L(e_ADCInputGain e_gain);
GD_RES_CODE tlv320aic33_set_ADC_L_from_MIC3_R(e_ADCInputGain e_gain);
GD_RES_CODE tlv320aic33_set_ADC_R_from_LINE1_L(e_ADCInputGain e_gain);
GD_RES_CODE tlv320aic33_set_ADC_R_from_LINE1_R(e_ADCInputGain e_gain);
GD_RES_CODE tlv320aic33_set_ADC_R_from_MIC3_L(e_ADCInputGain e_gain);
GD_RES_CODE tlv320aic33_set_ADC_R_from_MIC3_R(e_ADCInputGain e_gain);
//ADC R from LINE2 L and ADC L from LINE2 R not supported
GD_RES_CODE tlv320aic33_set_LINE2L_bypass(uint8_t i_enable);
GD_RES_CODE tlv320aic33_set_LINE2R_bypass(uint8_t i_enable);
GD_RES_CODE tlv320aic33_set_LINE2L_to_HPLOUT(float f_volume_dB);
GD_RES_CODE tlv320aic33_set_LINE2R_to_HPROUT(float f_volume_dB);

// line out and high power output control
GD_RES_CODE tlv320aic33_set_HP_out_power(uint8_t i_power);
GD_RES_CODE tlv320aic33_set_HP_out_mute(uint8_t i_mute);


//  gain and volume settings

// this is actually routing DACL1 to HPLOUT and DACR1 to HPROUT //todo rename
GD_RES_CODE tlv320aic33_set_headphones_volume_dB(float f_volume);

GD_RES_CODE tlv320aic33_get_headphones_volume_db(float *p_f_volume); //not completed
GD_RES_CODE tlv320aic33_set_DAC_volume_dB(float f_volume);
GD_RES_CODE tlv320aic33_set_ADC_gain_dB(float f_gain);

//  another high level functions

GD_RES_CODE tlv320aic33_get_ADC_flag_register(void);
GD_RES_CODE tlv320aic33_get_module_power_status(void);

GD_RES_CODE tlv320aic33_set_headphones_single_ended(uint8_t i_single_ended);
//this function is meaningless because headphones can by connected only as
//single ended and that is default setting of audio codec

//===========================| Mid level functions |===========================
GD_RES_CODE tlv320aic33_reset(void);
GD_RES_CODE tlv320aic33_set_output_driver_power_on_delay(
    e_OutputDriverPowerOnDelayControl e_delay);
GD_RES_CODE tlv320aic33_set_driver_ramp_up_step_time(
    e_DriverRampUpStepTimingControl e_time);
GD_RES_CODE tlv320aic33_set_DAC_volume(uint8_t i_volume);
GD_RES_CODE tlv320aic33_set_headphones_volume(uint8_t i_volume);

//===========================| Low level functions |===========================
GD_RES_CODE tlv320aic33_write_data(
    const uint8_t i_register_number,
    const uint8_t i_value);
GD_RES_CODE tlv320aic33_read_data(
    uint8_t i_register_number,
    uint8_t *p_data);

#endif
