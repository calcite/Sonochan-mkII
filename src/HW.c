/**
 * \file
 *
 * \brief HW controller for generic driver
 *
 * \b Important \b note for \b AVR8 architecture: functions, that read from\n
 * flash memory presume that constants are stored in low 64 kB of flash\n
 * memory. If not, please take a look at\n
 * \a http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=93874
 *
 * Created  28.08.2013\n
 * Modified 23.04.2014
 * 
 * \version 1.2.2
 * \author Martin Stejskal
 */

#include "HW.h"

//============================| Global variables |=============================
/**
 * \brief Array of pointers to metadata of drivers
 *
 * Pointer value is filled by preprocessor (macro)
 */
const gd_metadata *p_HW_metadata[HW_MAX_DEVICE_ID+1] ={
    HW_POINTER_ARRAY_TO_METADATA };


/**
 * \brief Get settings from defined device
 * @param i_device_ID Device ID. Defined in .h file
 * @param i_cmd_ID Command ID for device
 * @param p_config_table Pointer to image of configure table - 1 structure.\n
 * Settings will be saved to this structure
 * @return GD_SUCCESS if all OK. Else please refer GD_RES_CODE
 */
GD_RES_CODE hw_get_setting(uint8_t i_device_ID,
              uint16_t i_cmd_ID,
              gd_config_struct *p_config_table)
{
  // When get device ID, check if it is in boundary condition
  if(i_device_ID > HW_MAX_DEVICE_ID)
  {
    return GD_INCORRECT_DEVICE_ID;
  }

  /* OK, device should exist, so now call get_settings from generic_driver.c
   * and get actual data
   */
  return gd_get_setting(  p_HW_metadata[i_device_ID],
              i_cmd_ID,
              p_config_table);
}



/**
 * \brief Set settings at defined device
 * @param i_device_ID Device ID. Defined in .h file
 * @param i_cmd_ID Command ID for device
 * @param i_data Data. Must be type GD_DATA_VALUE (4B)
 * @return GD_SUCCESS if all OK. Else please refer GD_RES_CODE
 */
GD_RES_CODE hw_set_setting(uint8_t i_device_ID,
              uint16_t i_cmd_ID,
              GD_DATA_VALUE i_data)
{
  // When get device ID, check if it is in boundary condition
  if(i_device_ID > HW_MAX_DEVICE_ID)
  {
    return GD_INCORRECT_DEVICE_ID;
  }

  /* OK, device should exist, so now call set_settings from generic_driver.c
   * and set actual data
   */
  return gd_set_setting(  p_HW_metadata[i_device_ID],
              i_cmd_ID,
              i_data);
}


/**
 * \brief Get pointer to device metadata
 *
 * Example of usage:\n
 * \code
 * GD_RES_CODE status;
 * gd_metadata** p_p_metadata; // Pointer to pointer to metadata
 * gd_metadata*  p_metadata;   // Pointer to metadata
 * p_p_metadata = &p_metadata; // Assign pointer to pointer pointer address
 * status = hw_get_device_metadata(3, p_p_metadata); // Get pointer to metadata
 * // Now we can use p_metadata as pointer to structure with metadata
 * char *p_txt; // Pointer to character
 * p_txt = &p_metadata->c_description[0]; // Set to metadata description
 * LCD_write_string(p_txt); // Write device description to LCD
 * \endcode
 *
 * @param i_device_ID Device ID. Defined in .h file
 * @param p_device_metadata Pointer to user image of metadata. MUST BE POINTER
 * @return GD_SUCCESS if all OK. Else please refer GD_RES_CODE
 */
inline GD_RES_CODE hw_get_device_metadata(  uint8_t i_device_ID,
                      gd_metadata **p_device_metadata)
{
  // When get device ID, check if it is in boundary condition
  if(i_device_ID > HW_MAX_DEVICE_ID)
  {
    return GD_INCORRECT_DEVICE_ID;
  }

  *p_device_metadata = (gd_metadata*)p_HW_metadata[i_device_ID];

  return GD_SUCCESS;
}


/**
 * \brief Return last device index. Index start at 0
 *
 * @return Last device index. Index start at 0
 */
inline uint8_t hw_get_max_device_index(void)
{
  return HW_MAX_DEVICE_ID;
}
