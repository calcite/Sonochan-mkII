/* This header file is part of the ATMEL AVR32-SoftwareFramework-AT32UC3-1.5.0 Release */

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

#ifndef _DEVICE_GENERIC_HID_H_
#define _DEVICE_GENERIC_HID_H_


//================================| Includes |=================================
#include <inttypes.h>

#include "conf_usb.h"

#if USB_DEVICE_FEATURE == DISABLED
  #error "device_generic_hid.h is #included although USB_DEVICE_FEATURE\
 is disabled!"
#endif



//============================| Uniprot settings |=============================
/**
 * \brief Allow enable or disable uniprot support
 * Options: 0 (disabled), 1 (enabled)
 */
#define DEVICE_GENERIC_HID_SUPPORT_UNIPROT         1


// This settings have meaning only is support is enabled
#if DEVICE_GENERIC_HID_SUPPORT_UNIPROT
/**
 * Define pipe number for uniprot (Pipe order is defined in uniprot.h file)
 * by UNI_PIPEx_FRAME_SIZE, UNI_PIPEx_FUNC_INIT and UNI_PIPEx_FUNC_TASK
 * where "x" is pipe number which must be same here!
 */
#define DEVICE_GENERIC_HID_UNIPROT_PIPE    0

/// Define frame size (number of bytes per one transmission)
#define UNI_PIPE0_FRAME_SIZE    8
/// Initialize function which will be called in initialization process

#define UNI_PIPE0_FUNC_INIT     \
  device_generic_HID_init(UAC1_EP_HID_RX, UAC1_EP_HID_TX)

/// Driver task
// When RTOS is enabled, then task should not be called
#ifdef FREERTOS_USED
 #define UNI_PIPE0_FUNC_TASK
#else
 #define UNI_PIPE0_FUNC_TASK     \
   device_generic_hid_task()
#endif
// Universal protocol header file
#include "uniprot.h"
#endif

//===========================| Function prototypes |===========================

extern void device_generic_HID_init(U8 ep_hid_rx, U8 ep_hid_tx);
#ifdef FREERTOS_USED
extern void device_generic_hid_task(void *pvParameters);
#else
extern void device_generic_hid_task(void);
#endif


#endif  // _DEVICE_GENERIC_HID_H_
