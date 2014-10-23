/**
 * \file
 *
 * \brief Generic driver
 *
 * \b Important \b note for \b AVR8 architecture: functions, that read from\n
 * flash memory presume that constants are stored in low 64 kB of flash\n
 * memory. If not, please take a look at\n
 * \a http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=93874
 *
 * Created  26.08.2013\n
 * Modified 21.10.2014
 *
 * \version 1.3
 * \author Martin Stejskal, Tomas Bajus
 */


#ifndef _GENERIC_DRIVER_H_
#define _GENERIC_DRIVER_H_

//usart debug messages enable //not implemented in sonochan
//#define usart_debug_msg
//#include "usart.h"

//================================| Includes |=================================
#include <inttypes.h>
#include <stdio.h>      // Some architectures have there defined "uint" type

// Include only if architecture is AVR8
#ifdef __AVR_ARCH__
#include <avr/pgmspace.h>
#endif


//===============================| Definitions |===============================
/**
 * \brief Maximal size of text array for descriptors
 *
 * Due to SRAM limitations it is recommended set this value to 60. However on\n
 * more powerful AVR it could be more.
 */
#define GD_MAX_STRING_SIZE    80

//===============================| Structures |================================
#ifndef GD_RES_CODE_DEFINED

#define GD_RES_CODE_DEFINED
/**
 * \brief Error codes returned by functions
 */
typedef enum{
  GD_SUCCESS =                   0,//!< GD_SUCCESS
  GD_FAIL =                      1,//!< GD_FAIL
  GD_INCORRECT_PARAMETER =       2,//!< GD_INCORRECT_PARAMETER
  GD_INCORRECT_CMD_ID =          3,//!< GD_INCORRECT_CMD_ID
  GD_CMD_ID_NOT_EQUAL_IN_FLASH = 4,//!< GD_CMD_ID_NOT_EQUAL_IN_FLASH
  GD_INCORRECT_DEVICE_ID =       5 //!< GD_INCORRECT_DEVICE_ID
} GD_RES_CODE;
#endif

/**
 * \brief Enum of supported data types as values
 *
 * \note Every type should have defined number. It is not allowed to use\n
 * number 255 or higher. Higher layers use number 255 (0xFF) as special\n
 * signal that some exception was detected
 */
typedef enum{
  void_type = 0,   //!< void_type No data

  char_type = 1,  //!< char_type One character

  int_type = 2,   //!< int_type Signed integer
  int8_type = 3,  //!< int8_type 8 bit signed integer
  int16_type = 4, //!< int16_type 16 bit signed integer
  int32_type = 5, //!< int32_type 32 bit signed integer

  uint_type = 6,  //!< uint_type Unsigned integer
  uint8_type = 7, //!< uint8_type 8 bit unsigned integer
  uint16_type = 8,//!< uint16_type 16 bit unsigned integer
  uint32_type = 9,//!< uint32_type 32 bit unsigned integer

  float_type = 10, //!< float_type Float

  group_type = 11, //!< group_type Dummy command. Just metadata for group

} GD_DATA_TYPE;


/**
 * \brief For different data types is used union
 *
 * Because simply protocol do not know what data will receive, there is\n
 * 4 bytes "space" to save value. Depend on programmer which data type will\n
 * use.
 *
 * \note Big endian issue is corrected "on the fly".
 */
typedef union{
  char          data_char;

  int           data_int;       // Process as 32 bit. If not it is Your problem
  int8_t        data_int8;
  int16_t       data_int16;
  int32_t       data_int32;

#ifndef uint  // For case, that uint is not defined
#define uint unsigned int
#endif
  uint          data_uint;      // Process as 32 bit. If not it is Your problem
  uint8_t       data_uint8;
  uint16_t      data_uint16;
  uint32_t      data_uint32;

  float         data_float;
} GD_DATA_VALUE;

/**
 * \brief Structure for configuration table
 *
 * \b Important \b note: For "Command ID"=0 \b must be e_data_type set \n
 * to int8_type -> also u_min_value u_max_value and u_value\n
 * \b must be type uint8_t !
 */
typedef struct{
  uint16_t      i_cmd_id;        // Command ID
  char          c_name[GD_MAX_STRING_SIZE];  // Function name
  char          c_descriptor[GD_MAX_STRING_SIZE];  // Descriptor
  GD_DATA_TYPE  e_in_data_type;    // Select input data type
  GD_DATA_VALUE u_in_min_value;    // Minimal input value
  GD_DATA_VALUE u_in_max_value;    // Maximal input value

  GD_DATA_TYPE  e_out_data_type;  // Select output data type
  GD_DATA_VALUE u_out_min_value;  // Minimal output value
  GD_DATA_VALUE u_out_max_value;  // Maximal output value

  #ifdef __AVR_ARCH__								// not implemented in sonochan
  GD_DATA_VALUE def_value;				// Default value
	#endif

  GD_DATA_VALUE *p_out_value;    // Pointer to actual (output) value
  GD_RES_CODE   (*p_funtion)(void);  // Pointer to function
}gd_config_struct;


/**
 * \brief Structure for metadata of specific hardware
 *
 * Example of definition:\n
 * gd_metadata TLV320AIC23_metadata =\n
 * {\n
 *   TLV_MAX_CMD_ID,\n
 *   "Audio Codec TLV320AIC23B",\n
 *   (gd_config_struct*)&TLV320AIC23_config_table_flash,\n
 *   0x00\n
 * };\n
 */
typedef struct{
  // Maximal command ID - never should exceed 0xFFFE !!!
  uint16_t           i_max_cmd_ID;

  // Description of device
  char               c_description[GD_MAX_STRING_SIZE];

  gd_config_struct*  p_config_table;  // Pointer to configuration table
  uint8_t            i_serial;    // Serial number
}gd_metadata;

//=================================| Macros |==================================
// If AVR8 architecture
#ifdef __AVR_ARCH__
#define gd_read_byte_ro_mem(p_address)     pgm_read_byte(p_address)
#define gd_read_word_ro_mem(p_address)     pgm_read_word(p_address)
#define gd_read_dword_ro_mem(p_address)    pgm_read_dword(p_address)
#else   // Else different architecture
#define gd_read_byte_ro_mem(p_address)     *(p_address)
#define gd_read_word_ro_mem(p_address)     *(p_address)
#define gd_read_dword_ro_mem(p_address)    *(p_address)
#endif

// Create pointer according to pointer size (architecture)
#define GD_POINTER_TO_FUNCTION                                  \
switch(sizeof(p_size))                                          \
{                                                               \
case 1:                                                         \
  p_func = (void*) gd_read_byte_ro_mem(&p_flash->p_funtion);    \
  break;                                                        \
case 2:                                                         \
  p_func = (void*) gd_read_word_ro_mem(&p_flash->p_funtion);    \
  break;                                                        \
case 4:                                                         \
  p_func = (void*) gd_read_dword_ro_mem(&p_flash->p_funtion);   \
  break;                                                        \
default:                                                        \
  while(1);                                                     \
}

//============================| Global variables |=============================
extern volatile GD_DATA_VALUE gd_void_value;

//===========================| Function prototypes |===========================
GD_RES_CODE gd_get_setting(const gd_metadata *p_metadata,
              uint16_t i_cmd_ID,
              gd_config_struct *p_config_table);

GD_RES_CODE gd_set_setting(const gd_metadata  *p_metadata,
              uint16_t       i_cmd_ID,
              GD_DATA_VALUE    i_data);

GD_RES_CODE gd_void_function(void);
#endif
