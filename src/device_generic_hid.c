/* -*- mode: c++; tab-width: 4; c-basic-offset: 4 -*- */
/* This source file is part of the ATMEL AVR32-SoftwareFramework-AT32UC3-1.5.0 Release */

/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief Management of the USB device mouse HID task.
 *
 * This file manages the USB device mouse HID task.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a USB module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ***************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 */

//_____  I N C L U D E S ___________________________________________________

#include "conf_usb.h"


#if USB_DEVICE_FEATURE == ENABLED

#include "board.h"
#ifdef FREERTOS_USED
#include "FreeRTOS.h"
#include "task.h"
#endif
#include "usb_drv.h"
#include "gpio.h"
#include "usb_descriptors.h"
#include "usb_standard_request.h"
#include "usb_specific_request.h"
#include "device_generic_hid.h"

//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N S ______________________________________________



//_____ D E C L A R A T I O N S ____________________________________________

static uint8_t ep_hid_rx;
static uint8_t ep_hid_tx;
//!
//! @brief This function initializes the hardware/software resources
//! required for device HID task.
//!
void device_generic_HID_init(uint8_t ep_rx, uint8_t ep_tx)
{
  ep_hid_rx = ep_rx;
  ep_hid_tx = ep_tx;
#ifndef FREERTOS_USED
#if USB_HOST_FEATURE == ENABLED
  // If both device and host features are enabled, check if device mode is engaged
  // (accessing the USB registers of a non-engaged mode, even with load operations,
  // may corrupt USB FIFO data).
  if (Is_usb_device())
#endif  // USB_HOST_FEATURE == ENABLED
    Usb_enable_sof_interrupt();
#endif  // FREERTOS_USED

#ifdef FREERTOS_USED
  xTaskCreate(device_gneric_hid_task,
        configTSK_USB_DHID_NAME,
        configTSK_USB_DHID_STACK_SIZE,
        NULL,
        configTSK_USB_DHID_PRIORITY,
        NULL);
#endif  // FREERTOS_USED
}



//!
//! @brief Entry point of the device gneric HID task management
//!
#ifdef FREERTOS_USED
void device_gneric_hid_task(void *pvParameters)
#else
void device_gneric_hid_task(void)
#endif
{
  uint8_t i_data_length;
  const uint8_t EP_HID_RX = ep_hid_rx;
  const uint8_t EP_HID_TX = ep_hid_tx;

  // Enable/disable transmitter
  uint8_t i_tx_enable = 0;

  // Small counter
  uint8_t i;


#ifdef FREERTOS_USED
  portTickType xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();
  while (TRUE)
  {
    vTaskDelayUntil(&xLastWakeTime, configTSK_USB_DHID_PERIOD);

    // First, check the device enumeration state
    if (!Is_device_enumerated()) continue;
#else
    // First, check the device enumeration state
    if (!Is_device_enumerated()) return;
#endif  // FREERTOS_USED

    // Check if RX data ready
    if( Is_usb_out_received(EP_HID_RX))
    {
      Usb_reset_endpoint_fifo_access(EP_HID_RX);
      // Get data length
      i_data_length = Usb_byte_count(EP_HID_RX);
      /* Test data length. We expect 8, so if data input is longer, than extra
       * bytes will be removed/unused
       */
      if(i_data_length > 8)
      {
        i_data_length = 8;
      }
      // Read data
      usb_read_ep_rxpacket(EP_HID_RX, &usb_report[0], i_data_length, NULL);
      Usb_ack_out_received_free(EP_HID_RX);

      // Call function that process data
#if DEVICE_GENERIC_HID_SUPPORT_UNIPROT
      Uniprot_rx_data(DEVICE_GENERIC_HID_UNIPROT_PIPE, &usb_report[0]);
#else
      // PUT YOUR RX FUNCTION HERE
#endif
    }

    // Check if TX should transmit
#if DEVICE_GENERIC_HID_SUPPORT_UNIPROT
    i_tx_enable = Uniprot_Is_TX_transmission_enable(
                                            DEVICE_GENERIC_HID_UNIPROT_PIPE);
#else
    // PUT YOUR TX ENABLE FUNCTION/CONDITION HERE
    i_tx_enable = 1;
#endif
    if(i_tx_enable != 0)
    {
      if(Is_usb_in_ready(EP_HID_TX))
      {
        Usb_reset_endpoint_fifo_access(EP_HID_TX);
#if DEVICE_GENERIC_HID_SUPPORT_UNIPROT
        Uniprot_tx_data(DEVICE_GENERIC_HID_UNIPROT_PIPE, &usb_report[0]);
#else
        // PUT YOUR TX FUNCTION HERE
#endif
        // Transmit byte per byte
        for(i=0 ; i<8 ; i++)
        {
          Usb_write_endpoint_data(EP_HID_TX, 8, usb_report[i]);
        }
        Usb_ack_in_ready_send(EP_HID_TX);
      }
    }

#ifdef FREERTOS_USED
  }
#endif
}
#endif  // USB_DEVICE_FEATURE == ENABLED
