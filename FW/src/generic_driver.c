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

#include "generic_driver.h"

//============================| Global variables |=============================
/**
 * \brief Empty value.
 *
 * Useful when called function has no parameter -> link to this "empty value"\n
 * -> no compilation problems, no reading problems
 */
volatile GD_DATA_VALUE gd_void_value = {.data_uint32 = 0};

//====================| Function prototypes not for user |=====================
/**
 * \brief If big endian is detected, then input big endian data converted to\n
 *  little endian.
 *
 * Because higher layers expect little endian order, it is good idea to\n
 * convert bytes to little endian. Conversion is done only if needed.
 *
 * @param i_little_endian Detected endian. 0 - big endian ; 1 - little endian
 * @param i_32b_value Input data (32 bit)
 * @param e_data_type Define which data type will be presented
 * @return corrected 32 bit value.
 */
GD_DATA_VALUE gd_if_big_endian_convert_big_to_little_endian(
    uint8_t i_little_endian,
    GD_DATA_VALUE i_32b_value,
    GD_DATA_TYPE e_data_type);


/**
 * \brief If big endian is detected, then input little endian data are\n
 * converted to big endian
 *
 * Because higher layers works with little endian logic, it is good idean to\n
 * conver bytes to big endian. Conversion is done only if needed.
 *
 * @param i_little_endian Detected endian. 0 - big endian ; 1 - little endian
 * @param i_32b_value Input data (32 bit)
 * @param e_data_type Define which data type will be presented
 * @return corrected 32 bit value.
 */
GD_DATA_VALUE gd_if_big_endian_convert_little_to_big_endian(
    uint8_t i_little_endian,
    GD_DATA_VALUE i_32b_value,
    GD_DATA_TYPE e_data_type);
//================================| Functions |================================

/**
 * \brief Void function. Do nothing
 *
 * Sometimes is need to point to function, that does not do anything, so\n
 * there is
 * @return Always GD_SUCCESS
 */
GD_RES_CODE gd_void_function(void)
{
  //usart message
#ifdef usart_debug_msg
	usart_tx_text("gd_void_function\n");
#endif
	return GD_SUCCESS;
}



/**
 * \brief Get settings data from driver
 * @param p_metadata Pointer to metadata of driver (max CMD ID and so on)
 * @param i_cmd_ID Command ID. Thru commands user can control target device
 * @param p_config_table Pointer to image of configure table - 1 structure.\n
 * Settings will be saved to this structure
 * @return GD_SUCCESS if all OK. Else please refer GD_RES_CODE
 */
GD_RES_CODE gd_get_setting(const gd_metadata *p_metadata,
              uint16_t i_cmd_ID,
              gd_config_struct *p_config_table)
{
  // First check i_cmd_ID
  // Check if ID is not higher than max ID
  // Note: "(*p_metadata).i_max_cmd_ID" is same
  if((i_cmd_ID > p_metadata->i_max_cmd_ID) ||
    (i_cmd_ID > 0xFFFE)
  )
  {
    return GD_INCORRECT_CMD_ID;
  }

  // Set pointer to "right" configure table on flash (device configure table)
  gd_config_struct *p_flash;

  // And add offset for cmd - remember: It is array of structures
  p_flash = (p_metadata->p_config_table) + i_cmd_ID;


  // Read command ID (and check if is same)
  if( gd_read_byte_ro_mem( &(p_flash->i_cmd_id) ) != i_cmd_ID)
  {
    return GD_CMD_ID_NOT_EQUAL_IN_FLASH;
  }

  // Keep detected endian value
  uint8_t i_little_endian;
  // Test endian. With little endian is all right.
  const GD_DATA_VALUE tmp_data_val_32 = {.data_uint32 = 1};
  if(tmp_data_val_32.data_uint8 == 1)
  {
    i_little_endian = 1;
  }
  else
  {
    i_little_endian = 0;
  }

  // OK, seems to be all right, lets fill configuration table


  // Write ID
  p_config_table->i_cmd_id = i_cmd_ID;



  /* Create pointer to read only (usually flash) memory
   */
  uint8_t *p_add_flash;

  // Create pointer to SRAM (point to characters in descriptor)
  uint8_t *p_add_SRAM;



  // Write name

  // Set flash pointer to begin of string descriptor
  p_add_flash =(uint8_t *)
      &(p_flash->c_name[0]);
  // Set SRAM pointer to begin of string descriptor
  p_add_SRAM = (uint8_t *) &(p_config_table->c_name[0]);

  #ifdef __AVR_ARCH__
  strlcpy_P(p_add_SRAM,p_add_flash,GD_MAX_STRING_SIZE);
  #else   // Else different architecture
  strlcpy(p_add_SRAM,p_add_flash,GD_MAX_STRING_SIZE);
  #endif


  // Write descriptor

  // Set flash pointer to begin of string descriptor
  p_add_flash =(uint8_t *)
      &(p_flash->c_descriptor[0]);
  // Set SRAM pointer to begin of string descriptor
  p_add_SRAM = (uint8_t *) &(p_config_table->c_descriptor[0]);


  #ifdef __AVR_ARCH__
  strlcpy_P(p_add_SRAM,p_add_flash,GD_MAX_STRING_SIZE);
  #else   // Else different architecture
  strlcpy(p_add_SRAM,p_add_flash,GD_MAX_STRING_SIZE);
  #endif


  // Now get input data type
  p_config_table->e_in_data_type =(GD_DATA_TYPE)
      gd_read_byte_ro_mem(&p_flash->e_in_data_type);


  // Copy 32 bits of input min value
  p_config_table->u_in_min_value.data_uint32 =
      gd_read_dword_ro_mem(&(p_flash->u_in_min_value.data_uint32));
  // Check if system is Big endian (evil)
  p_config_table->u_in_min_value = gd_if_big_endian_convert_big_to_little_endian(
      i_little_endian,
      p_config_table->u_in_min_value,
      p_config_table->e_in_data_type);




  // Copy 32 bits of input max value
  p_config_table->u_in_max_value.data_uint32 =
      gd_read_dword_ro_mem(&(p_flash->u_in_max_value.data_uint32));
  // Check if system is Big endian (evil)
  p_config_table->u_in_max_value = gd_if_big_endian_convert_big_to_little_endian(
      i_little_endian,
      p_config_table->u_in_max_value,
      p_config_table->e_in_data_type);




  // Now get output data type
  p_config_table->e_out_data_type =(GD_DATA_TYPE)
      gd_read_byte_ro_mem(&p_flash->e_out_data_type);




  // Copy 32 bits of output min value
  p_config_table->u_out_min_value.data_uint32 =
      gd_read_dword_ro_mem(&(p_flash->u_out_min_value.data_uint32));
  // Check if system is Big endian (evil)
  p_config_table->u_out_min_value = gd_if_big_endian_convert_big_to_little_endian(
      i_little_endian,
      p_config_table->u_out_min_value,
      p_config_table->e_out_data_type);




  // Copy 32 bits of output max value
  p_config_table->u_out_max_value.data_uint32 =
      gd_read_dword_ro_mem(&(p_flash->u_out_max_value.data_uint32));
  // Check if system is Big endian (evil)
  p_config_table->u_out_max_value = gd_if_big_endian_convert_big_to_little_endian(
      i_little_endian,
      p_config_table->u_out_max_value,
      p_config_table->e_out_data_type);


#ifdef __AVR_ARCH__								// not implemented in sonochan
  // Copy 32 bits of default value
  p_config_table->def_value.data_uint32 =
      gd_read_dword_ro_mem(&(p_flash->def_value.data_uint32));
  // Check if system is Big endian (evil)
  p_config_table->def_value = gd_if_big_endian_convert_big_to_little_endian(
      i_little_endian,
      p_config_table->def_value,
      p_config_table->e_out_data_type);
#endif




  /* Copy pointer to output value and function.
   * Because on different architectures (8b/16b/32b/64b) can be different
   * pointer size, we must select right one
   */
  const void *p_size;
  switch(sizeof(p_size))
  {
  case 1:
    // 8 bit pointer
    p_config_table->p_out_value = (GD_DATA_VALUE*)
        gd_read_byte_ro_mem(&
        (p_flash->p_out_value));
    // Copy pointer to function
    p_config_table->p_funtion =
        (void*)gd_read_byte_ro_mem(&p_flash->p_funtion);
    break;
  case 2:
    // 16 bit pointer
    p_config_table->p_out_value = (GD_DATA_VALUE*)
        gd_read_word_ro_mem(&
        (p_flash->p_out_value));
    // Copy pointer to function
    p_config_table->p_funtion =
        (void*)gd_read_word_ro_mem(&p_flash->p_funtion);
    break;
  case 4:
    // 32 bit pointer
    p_config_table->p_out_value = (GD_DATA_VALUE*)
        gd_read_dword_ro_mem(&
        (p_flash->p_out_value));
    // Copy pointer to function
    p_config_table->p_funtion =
        (void*)gd_read_dword_ro_mem(&p_flash->p_funtion);
    break;
  default:
    // Unsupported architecture
    while(1);
  }


  // All done, success
  return GD_SUCCESS;
}


/**
 * \brief Set settings data to driver
 * @param p_metadata Pointer to metadata of driver (max CMD ID and so on)
 * @param i_cmd_ID Command ID. Thru commands user can control target device.
 * @param i_data Data in union structure GD_DATA_VALUE
 * @return GD_SUCCESS if all OK. Else please refer GD_RES_CODE
 */
GD_RES_CODE gd_set_setting(  const gd_metadata  *p_metadata,
                uint16_t       i_cmd_ID,
                GD_DATA_VALUE    i_data)
{
	// First check i_cmd_ID
  // Check if ID is not higher than max ID
  // Note: "(*p_metadata).i_max_cmd_ID" is same
  if(  (i_cmd_ID > p_metadata->i_max_cmd_ID) || (i_cmd_ID > 0xFFFE)  )
  {
    //usart message - CMD ID
			#ifdef usart_debug_msg
  		usart_tx_text("set_setting\n");
			#endif
  	return GD_INCORRECT_CMD_ID;
  }

  // Set pointer to "right" configure table on flash (device configure table)
  gd_config_struct *p_flash;

  // And add offset for cmd - remember: It is array of structures
  p_flash = (p_metadata->p_config_table) + i_cmd_ID;


  // Read command ID (and check if is same)
  if( (gd_read_byte_ro_mem( &(p_flash->i_cmd_id))) != i_cmd_ID)
  {
    return GD_CMD_ID_NOT_EQUAL_IN_FLASH;
  }


  // Keep detected endian value
  uint8_t i_little_endian;
  // Test endian. With little endian is all right.
  const GD_DATA_VALUE tmp_data_val_32 = {.data_uint32 = 1};
  if(tmp_data_val_32.data_uint8 == 1)
  {
    i_little_endian = 1;
  }
  else
  {
    i_little_endian = 0;
  }
  // OK, seems to be all right. Let's call some functions

  /* Create pointer to function depend on input data type ->
   * -> first get data_type and if needed convert data
   */
  GD_DATA_TYPE i_in_data_type =
      (GD_DATA_TYPE) gd_read_byte_ro_mem(&p_flash->e_in_data_type);

  // Convert input data if needed
  i_data = gd_if_big_endian_convert_little_to_big_endian(
      i_little_endian,
      i_data,
      i_in_data_type);


  /* Set pointer address.
   * Because on different architectures (8b/16b/32b/64b) can be different
   * pointer size, we must select right one. So for this is there macro
   * GD_POINTER_TO_FUNCTION. However we must define some pointer so compiler
   * can get sizeof(pointer).
   */
  const void *p_size;

  // NOTE: p_func: pointer to function

  // TEST if data type is VOID
  if(i_in_data_type == void_type)
  {
    GD_RES_CODE (*p_func)(void);
    GD_POINTER_TO_FUNCTION              // Set pointer
    return p_func();                    // Call function
  }
  // TEST if data type is CHAR
  if(i_in_data_type == char_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(char);
    GD_POINTER_TO_FUNCTION              // Set pointer
    return p_func(i_data.data_char);    // Call function
  }
  // TEST if data type is INT
  if(i_in_data_type == int_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(int);
    GD_POINTER_TO_FUNCTION              // Set pointer
    return p_func(i_data.data_int);     // Call function
  }
  // TEST if data type is INT8
  if(i_in_data_type == int8_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(int8_t);
    GD_POINTER_TO_FUNCTION              // Set pointer
    return p_func(i_data.data_int8);    // Call function
  }
  // TEST if data type is INT16
  if(i_in_data_type == int16_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(int16_t);
    GD_POINTER_TO_FUNCTION              // Set pointer
    return p_func(i_data.data_int16);   // Call function
  }
  // TEST if data type is INT32
  if(i_in_data_type == int32_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(int32_t);
    GD_POINTER_TO_FUNCTION              // Set pointer
    return p_func(i_data.data_int32);   // Call function
  }
  // TEST if data type is UINT
  if(i_in_data_type == uint_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(uint);
    GD_POINTER_TO_FUNCTION              // Set pointer
    return p_func(i_data.data_uint);    // Call function
  }
  // TEST if data type is UINT8
  if(i_in_data_type == uint8_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(uint8_t);
    GD_POINTER_TO_FUNCTION              // Set pointer
    return p_func(i_data.data_uint8);   // Call function
  }
  // TEST if data type is UINT16
  if(i_in_data_type == uint16_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(uint16_t);
    GD_POINTER_TO_FUNCTION              // Set pointer
    return p_func(i_data.data_uint16);  // Call function
  }
  // TEST if data type is UINT32
  if(i_in_data_type == uint32_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(uint32_t);
    GD_POINTER_TO_FUNCTION              // Set pointer
    return p_func(i_data.data_uint32);  // Call function
  }
  // TEST if data type is FLOAT
  if(i_in_data_type == float_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(float);
    GD_POINTER_TO_FUNCTION              // Set pointer
    return p_func(i_data.data_float);   // Call function
  }
  /* TEST if data type is GROUP -> just metadata function. Should not be
   * called. Just return GD_INCORRECT_CMD_ID (well, it is not valid command,
   * so ...)
   */
  if(i_in_data_type == group_type)
  {
    return GD_INCORRECT_CMD_ID;
  }

  // Else data type is not defined here -> fail
  return GD_FAIL;
}

//====================| Function prototypes not for user |=====================

inline GD_DATA_VALUE gd_if_big_endian_convert_big_to_little_endian(
    uint8_t i_little_endian,
    GD_DATA_VALUE i_32b_value,
    GD_DATA_TYPE e_data_type)
{
  // If little endian, everything have logic and we can not process data
  if(i_little_endian == 1)
    return i_32b_value;
  // Else we must do some annoying processing. Oh, I hate Big endian systems...

  /* According to data type switch bits. It is not so easy, because higher
   * layers use shifting, so 32 bit value are magical right, but for 16 bit
   * and 8 bit not :/
   */
  // Test for 32 bit value
  if((e_data_type == int32_type) ||
     (e_data_type == uint32_type) ||
     (e_data_type == int_type) ||
     (e_data_type == uint_type) ||
     (e_data_type == float_type))
    return i_32b_value;     // Nothing to do

  // Test for 16 bit values
  if((e_data_type == int16_type) ||
     (e_data_type == uint16_type))
  {
    // Load data from uint16_type to 32 bit and swap them
    i_32b_value.data_uint32 = i_32b_value.data_uint16;
    return i_32b_value;
  }

  // Test for 8 bit values
  if((e_data_type == char_type) ||
     (e_data_type == int8_type) ||
     (e_data_type == uint8_type))
  {
    // Load data from uint8_type to 32 bit and swap them
    i_32b_value.data_uint32 = i_32b_value.data_uint8;
    return i_32b_value;
  }

  /* Else never should happen. Anyway, everything is possible, so let's set
   * return value to some strange value
   */
  i_32b_value.data_uint32 = 0xEEEEEEEE;
  return i_32b_value;
}




inline GD_DATA_VALUE gd_if_big_endian_convert_little_to_big_endian(
    uint8_t i_little_endian,
    GD_DATA_VALUE i_32b_value,
    GD_DATA_TYPE e_data_type)
{
  // If little endian, everything have logic and we can not process data
  if(i_little_endian == 1)
    return i_32b_value;
  // Else we must do some annoying processing. Oh, I hate Big endian systems...

  /* According to data type switch bits. It is not so easy, because higher
   * layers use shifting, so 32 bit value are magical right, but for 16 bit
   * and 8 bit not :/
   */
  // Test for 32 bit value
  if((e_data_type == int32_type) ||
     (e_data_type == uint32_type) ||
     (e_data_type == int_type) ||
     (e_data_type == uint_type) ||
     (e_data_type == float_type))
    return i_32b_value;     // Nothing to do

  // Test for 16 bit values
  if((e_data_type == int16_type) ||
     (e_data_type == uint16_type))
  {
    // Load data from uint16_type to 32 bit and swap them
    i_32b_value.data_uint32 = 0xFFFF0000 & i_32b_value.data_uint32<<16;
    return i_32b_value;
  }

  // Test for 8 bit values
  if((e_data_type == char_type) ||
     (e_data_type == int8_type) ||
     (e_data_type == uint8_type))
  {
    // Load data from uint8_type to 32 bit and swap them
    i_32b_value.data_uint32 = 0xFF000000 & i_32b_value.data_uint32<<24;
    return i_32b_value;
  }
  /* Else never should happen. Anyway, everything is possible, so let's set
   * return value to some strange value
   */
  i_32b_value.data_uint32 = 0xEEEEEEEE;
  return i_32b_value;
}



