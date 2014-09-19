/* -*- mode: c++; tab-width: 4; c-basic-offset: 4 -*- */
/* This source file is part of the ATMEL AVR32-SoftwareFramework-AT32UC3-1.5.0 Release */

/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief USB identifiers.
 *
 * This file contains the USB parameters that uniquely identify the USB
 * application through descriptor tables.
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
 *
 * Additions and Modifications to ATMEL AVR32-SoftwareFramework-AT32UC3 are:
 *
 * Copyright (C) Alex Lee
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

//_____ I N C L U D E S ____________________________________________________

#include "conf_usb.h"

#if USB_DEVICE_FEATURE == ENABLED

#include "usb_drv.h"
#include "usb_descriptors.h"
#include "usb_standard_request.h"
#include "usb_specific_request.h"
#include "usb_audio.h"


//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N S ______________________________________________


// image specific definitions moved to *_usb_descriptors.c

U16 configTSK_USB_DEV_PERIOD;

// usb_user_language_id
const S_usb_language_id usb_user_language_id =
{
  sizeof(S_usb_language_id),
  STRING_DESCRIPTOR,
  Usb_format_mcu_to_usb_data(16, LANGUAGE_ID)
};


// usb_user_manufacturer_string_descriptor
const S_usb_manufacturer_string_descriptor usb_user_manufacturer_string_descriptor =
{
  sizeof(S_usb_manufacturer_string_descriptor),
  STRING_DESCRIPTOR,
  USB_MANUFACTURER_NAME
};


// usb_user_product_string_descriptor
//[Martin] This section will be re written
#if defined (__GNUC__)
__attribute__((__section__(".userpage")))
#endif
const S_usb_product_string_descriptor usb_user_product_string_descriptor =
{
  /* USB will see (by default) only name (not number). When user change number,
   * after restart name will be changed.
   */
  (sizeof(S_usb_product_string_descriptor)),
  STRING_DESCRIPTOR,
  USB_PRODUCT_NAME
};


// usb_user_serial_number
const S_usb_serial_number usb_user_serial_number =
{
  sizeof(S_usb_serial_number),
  STRING_DESCRIPTOR,
  USB_SERIAL_NUMBER
};


//usb_user_wl
const S_usb_wl usb_user_wl =
{
  sizeof(S_usb_wl),
  STRING_DESCRIPTOR,
  USB_WL
};

//usb_user_ait
const S_usb_ait usb_user_ait =
{
  sizeof(S_usb_ait),
  STRING_DESCRIPTOR,
  USB_AIT
};

//usb_user_aot
const S_usb_aot usb_user_aot =
{
  sizeof(S_usb_aot),
  STRING_DESCRIPTOR,
  USB_AOT
};

//usb_user_ain
const S_usb_ain usb_user_ain =
{
  sizeof(S_usb_ain),
  STRING_DESCRIPTOR,
  USB_AIN
};

//usb_user_aia
const S_usb_aia usb_user_aia =
{
  sizeof(S_usb_aia),
  STRING_DESCRIPTOR,
  USB_AIA
};

// usb_hid_report_descriptor
const U8 usb_hid_report_descriptor[USB_HID_REPORT_DESC] =
{
      0x06, 0xA0, 0xFF,  // Usage page (vendor defined)
      0x09, 0x01,  // Usage ID (vendor defined)
      0xA1, 0x01,  // Collection (application)

    // The Input report
        0x09, 0x03,       // Usage ID - vendor defined
        0x15, 0x00,       // Logical Minimum (0)
        0x26, 0xFF, 0x00,   // Logical Maximum (255)
        0x75, 0x08,       // Report Size (8 bits)
        0x95, 0x08,       // Report Count (8 fields)
        0x81, 0x02,       // Input (Data, Variable, Absolute)

    // The Output report
        0x09, 0x04,       // Usage ID - vendor defined
        0x15, 0x00,       // Logical Minimum (0)
        0x26, 0xFF, 0x00,   // Logical Maximum (255)
        0x75, 0x08,       // Report Size (8 bits)
        0x95, 0x08,       // Report Count (8 fields)
        0x91, 0x02,        // Output (Data, Variable, Absolute)

    // The Feature report
        0x09, 0x05,       // Usage ID - vendor defined
        0x15, 0x00,       // Logical Minimum (0)
        0x26, 0xFF, 0x00,   // Logical Maximum (255)
        0x75, 0x08,      // Report Size (8 bits)
        0x95, 0x08,     // Report Count  (8 fields)
        0xB1, 0x02,       // Feature (Data, Variable, Absolute)

      0xC0  // end collection

};





//================================| Functions |================================

/**
 * @brief Add user defined number to product name
 * This function allow user simply set custom number to every single device.
 * @param i_number Options: 0-99 (else set 0)
 */
inline void usb_desc_set_number_to_product_name(uint8_t i_number)
{
  uint8_t i_10  =  0;
  uint8_t i_1   =   0;

  // Check input parameter
  if(i_number >99) i_number = 99;

  // Get tens
  i_10  = i_number/10;
  i_number = i_number - (10*i_10);

  // Get units
  i_1   = i_number;

  // To get correct character is enough just add ASCII value 0x30 :)
  flashc_memset16((void*)&usb_user_product_string_descriptor.wstring[USB_PN_LENGTH-2],
      Usb_format_mcu_to_usb_data(16, (U16)(i_10  + 0x30)),
      2,1);
  flashc_memset16((void*)&usb_user_product_string_descriptor.wstring[USB_PN_LENGTH-1],
      Usb_format_mcu_to_usb_data(16, (U16)(i_1   + 0x30)),
      2,1);
}

/**
 * @brief Get number behind product name
 * @return Number behind device name
 */
inline uint8_t usb_desc_get_number_from_product_name(void)
{
  uint8_t i_number = 0;

  // To access memory we must use pointer, else optimizer set all as constants
  // and reading actual value will be fake (just old one value)
  volatile uint16_t *p_mem;

  // Get from ASCII value number - minus 0x30 ('0')

  // 10x
  p_mem = (uint16_t*)
          &(usb_user_product_string_descriptor.wstring[USB_PN_LENGTH-2]);

  i_number = i_number + 10*(uint8_t)
       (Usb_format_usb_to_mcu_data(16, (U16)(*p_mem))    -'0' );

  // 1x
  p_mem = (uint16_t*)
          &(usb_user_product_string_descriptor.wstring[USB_PN_LENGTH-1]);

  i_number = i_number +(uint8_t)
      (Usb_format_usb_to_mcu_data(16, (U16)(*p_mem))     -'0');

  return i_number;
}


#endif  // USB_DEVICE_FEATURE == ENABLED
