/**
 * \file
 *
 * \brief Driver for codec TLV320AIC33
 *
 * Created:  02.04.2014\n
 * Modified: 02.04.2014
 *
 * \version 0.1
 * \author Martin Stejskal
 */

#include "tlv320aic33.h"

//============================| Global variables |=============================
//=========================| Generic driver support |==========================
//=============================| FreeRTOS stuff |==============================
// If RTOS support is enabled, create this
#if TLV320AIC33_SUPPORT_RTOS != 0
portBASE_TYPE xStatus;
xSemaphoreHandle mutexI2C;
#endif

//==========================| High level functions |===========================

/**
 * \brief Initialize TLV320AIC33
 *
 * Must be called \b before any \b other \b function from this library!
 *
 * @return GD_SUCCESS if all right
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

  return GD_SUCCESS;
}


/**
 * \brief Write data on TWI (I2C) bus
 *
 * MCU is in master mode. Send TLV320AIC33 address and then data (argument). This\n
 * function also check if TWI module is available (when RTOS support is\n
 * enabled).
 *
 * @param p_data Pointer to data array which will be send to PLL thru TWI (I2C)
 *
 * @param i_number_of_bytes Number of data bytes, which will be send
 *
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE tlv320aic33_write_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes)
{
  // If RTOS support enable, then "lock" TWI module
  TLV320AIC33_LOCK_TWI_MODULE_IF_RTOS

  // Write data to PLL thru HAL. Also check result status
  if(tlv320aic33_HAL_write_data(p_data, i_number_of_bytes) != TLV320AIC33_OK)
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
 * MCU is in master mode. Send TLV320AIC33 address and then receive data. This\n
 * function also check if TWI module is available (when RTOS support is\n
 * enabled).
 *
 * @param p_data Data are saved thru this pointer
 *
 * @param i_number_of_bytes Number of data bytes, which will be received
 *
 * @return GD_SUCCESS (0) if all OK
 */
inline GD_RES_CODE tlv320aic33_read_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes)
{
  // If RTOS support enable, then "lock" TWI module
  TLV320AIC33_LOCK_TWI_MODULE_IF_RTOS

  // Read data from PLL thru HAL. Also check result status
  if(tlv320aic33_HAL_read_data(p_data, i_number_of_bytes) != TLV320AIC33_OK)
  {
    // If not OK -> unlock device (if needed) and return FAIL
    TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS
    return GD_FAIL;
  }

  // "Unlock" device if needed
  TLV320AIC33_UNLOCK_TWI_MODULE_IF_RTOS
  return GD_SUCCESS;
}


