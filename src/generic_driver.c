/**
 * \file
 *
 * \brief Generic driver
 *
 * Created  26.08.2013
 * Modified 02.04.2014
 *
 * \b Important \b note for \b AVR8 architecture: functions, that read from\n
 * flash memory presume that constants are stored in low 64 kB of flash\n
 * memory. If not, please take a look at\n
 * \a http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=93874
 *
 * \version 1.4
 * \author Martin Stejskal
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
  if( read_byte_ro_mem( &(p_flash->i_cmd_id) ) != i_cmd_ID)
  {
    return GD_CMD_ID_NOT_EQUAL_IN_FLASH;
  }

  // OK, seems to be all right, lets fill configuration table




  // Write ID
  p_config_table->i_cmd_id = i_cmd_ID;




  /* Create pointer to read only (usually flash) memory
   */
  uint8_t *p_address_to_flash;

  // Create pointer to SRAM (point to characters in descriptor)
  uint8_t *p_address_to_SRAM;



  // Write name
  // Temporary value
  uint8_t c_tmp;

  // Set flash pointer to begin of string descriptor
  p_address_to_flash =(uint8_t *)
      &(p_flash->c_name[0]);
  // Set SRAM pointer to begin of string descriptor
  p_address_to_SRAM = (uint8_t *) &(p_config_table->c_name[0]);

  /* Temporary counter for case, that descriptor on flash will be longer
   * than GD_MAX_STRING_SIZE - in that case just stop reading from
   * flash and as last character will be written to SRAM as NULL
   */
  uint8_t i_tmp_cnt = 1;
  while(1)
  {
    // Copy string data until NULL character found

    // Get character
    c_tmp = read_byte_ro_mem(p_address_to_flash++);
    // Copy it
    *(p_address_to_SRAM++) = c_tmp;

    // And check for NULL character
    if( c_tmp == 0x00 )
    {
      break;
    }


    // Increase value of temporary counter
    i_tmp_cnt++;
    // Test, if we are on the end of string array in SRAM
    if(i_tmp_cnt == GD_MAX_STRING_SIZE)
    {
      // If we have only last B free, write NULL character
      *(p_address_to_SRAM) = 0x00;
      // And jump out of the cycle
      break;
    }
  }


  // Write descriptor

  // Set flash pointer to begin of string descriptor
  p_address_to_flash =(uint8_t *)
      &(p_flash->c_descriptor[0]);
  // Set SRAM pointer to begin of string descriptor
  p_address_to_SRAM = (uint8_t *) &(p_config_table->c_descriptor[0]);

  /* Temporary counter for case, that descriptor on flash will be longer
   * than GD_MAX_DESCRIPTOR_SIZE - in that case just stop reading from
   * flash and as last character will be written to SRAM NULL
   */
  i_tmp_cnt = 1;
  while(1)
  {
    // Copy string data until NULL character found

    // Get character
    c_tmp = read_byte_ro_mem(p_address_to_flash++);
    // Copy it
    *(p_address_to_SRAM++) = c_tmp;

    // And check for NULL character
    if( c_tmp == 0xFF )
    {
      break;
    }


    // Increase value of temporary counter
    i_tmp_cnt++;
    // Test, if we are on the end of string array in SRAM
    if(i_tmp_cnt == GD_MAX_STRING_SIZE)
    {
      // If we have only last B free, write NULL character
      *(p_address_to_SRAM) = 0x00;
      // And jump out of the cycle
      break;
    }
  }




  // Now get input data type
  p_config_table->e_in_data_type =(GD_DATA_TYPE)
      read_byte_ro_mem(&p_flash->e_in_data_type);




  // Copy 32 bits of input min value
  p_config_table->u_in_min_value.data_uint32 =
      read_dword_ro_mem(&(p_flash->u_in_min_value.data_uint32));




  // Copy 32 bits of input max value
  p_config_table->u_in_max_value.data_uint32 =
      read_dword_ro_mem(&(p_flash->u_in_max_value.data_uint32));




  // Now get output data type
  p_config_table->e_out_data_type =(GD_DATA_TYPE)
      read_byte_ro_mem(&p_flash->e_out_data_type);




  // Copy 32 bits of output min value
  p_config_table->u_out_min_value.data_uint32 =
      read_dword_ro_mem(&(p_flash->u_out_min_value.data_uint32));




  // Copy 32 bits of output max value
  p_config_table->u_out_max_value.data_uint32 =
      read_dword_ro_mem(&(p_flash->u_out_max_value.data_uint32));




  // Copy pointer to output value
  p_config_table->p_out_value = (GD_DATA_VALUE*)
      read_word_ro_mem(&
      (p_flash->p_out_value));



  // Copy pointer to function
  p_config_table->p_funtion = (void*)read_word_ro_mem(&p_flash->p_funtion);




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
  if( (read_byte_ro_mem( &(p_flash->i_cmd_id))) != i_cmd_ID)
  {
    return GD_CMD_ID_NOT_EQUAL_IN_FLASH;
  }

  // OK, seems to be all right. Let's call some functions




  /* Create pointer to function depend on input data type ->
   * -> first get data_type
   */
  GD_DATA_TYPE i_in_data_type =
      (GD_DATA_TYPE) read_byte_ro_mem(&p_flash->e_in_data_type);

  // NOTE: p_func: pointer to function

  // TEST if data type is VOID
  if(i_in_data_type == void_type)
  {
    GD_RES_CODE (*p_func)(void);
    p_func = (void*) read_word_ro_mem(&p_flash->p_funtion);  // Set pointer
    return p_func();                  // Call function
  }
  // TEST if data type is CHAR
  if(i_in_data_type == char_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(char);
    p_func = (void*) read_word_ro_mem(&p_flash->p_funtion);  // Set pointer
    return p_func(i_data.data_char);        // Call function
  }
  // TEST if data type is INT
  if(i_in_data_type == int_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(int);
    p_func = (void*) read_word_ro_mem(&p_flash->p_funtion);  // Set pointer
    return p_func(i_data.data_int);          // Call function
  }
  // TEST if data type is INT8
  if(i_in_data_type == int8_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(int8_t);
    p_func = (void*) read_word_ro_mem(&p_flash->p_funtion);  // Set pointer
    return p_func(i_data.data_int8);        // Call function
  }
  // TEST if data type is INT16
  if(i_in_data_type == int16_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(int16_t);
    p_func = (void*) read_word_ro_mem(&p_flash->p_funtion);  // Set pointer
    return p_func(i_data.data_int16);        // Call function
  }
  // TEST if data type is INT32
  if(i_in_data_type == int32_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(int32_t);
    p_func = (void*) read_word_ro_mem(&p_flash->p_funtion);  // Set pointer
    return p_func(i_data.data_int32);        // Call function
  }
  // TEST if data type is UINT
  if(i_in_data_type == uint_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(uint);
    p_func = (void*) read_word_ro_mem(&p_flash->p_funtion);  // Set pointer
    return p_func(i_data.data_uint);        // Call function
  }
  // TEST if data type is UINT8
  if(i_in_data_type == uint8_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(uint8_t);
    p_func = (void*) read_word_ro_mem(&p_flash->p_funtion);  // Set pointer
    return p_func(i_data.data_uint8);        // Call function
  }
  // TEST if data type is UINT16
  if(i_in_data_type == uint16_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(uint16_t);
    p_func = (void*) read_word_ro_mem(&p_flash->p_funtion);  // Set pointer
    return p_func(i_data.data_uint16);        // Call function
  }
  // TEST if data type is UINT32
  if(i_in_data_type == uint32_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(uint32_t);
    p_func = (void*) read_word_ro_mem(&p_flash->p_funtion);  // Set pointer
    return p_func(i_data.data_uint32);        // Call function
  }
  // TEST if data type is FLOAT
  if(i_in_data_type == float_type)
  {
    // Get pointer to function
    GD_RES_CODE (*p_func)(float);
    p_func = (void*) read_word_ro_mem(&p_flash->p_funtion);  // Set pointer
    return p_func(i_data.data_float);        // Call function
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











