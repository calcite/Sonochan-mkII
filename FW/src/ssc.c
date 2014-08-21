/**
 * \file
 *
 * \brief Driver for SSC module on UC3A3256
 *
 * Created:  20.08.2014\n
 * Modified: 20.08.2014
 *
 * \version 0.1
 * \author Martin Stejskal
 */

#include "ssc.h"

//============================| Global variables |=============================

//================================| Functions |================================

//==========================| High level functions |===========================
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



//===========================| Low level functions |===========================
SSC_RES_CODE ssc_FSYNC_role()
{
  return SSC_SUCCESS;
}


