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
 * \brief Virtual bridge, that connect HW library and universal protocol
 * library
 *
 *
 * Created  24.09.2013\n
 * Modified 28.04.2014
 *
 * \version 0.4.1
 * \author Martin Stejskal
 */

#ifndef _HW_BRIDGE_UNIPROT_H
#define _HW_BRIDGE_UNIPROT_H

//=================================| Options |=================================
/// \brief Define witch pipe bridge will use
#define HW_BRIDGE_UNIPROT_USED_PIPE             0

/**
 * \brief Allow enable or disable FreeRTOS support
 *
 * Options: 0 (FreeRTOS disabled) or 1 (FreeRTOS enabled)
 */
#define HW_BRIDGE_UNIPROT_SUPPORT_RTOS          1


/**
 * \brief Debug version - 1 (debug enabled) or 0 (debug disabled)
 *
 * When software is in develop stage there can be a lot unexpected behavior.\n
 * So in debug mode is there more checking, witch in fully working code have\n
 * no meaning. Anyway sometimes this redundant checking is good when debugging.
 */
#define HW_BRIDGE_UNIPROT_DEBUG                 1

//=========================| Some checks of settings |=========================

//================================| Includes |=================================
// HW library
#include "HW.h"

// Universal protocol library
#include "uniprot.h"

// FreeRTOS stuff
#if HW_BRIDGE_UNIPROT_SUPPORT_RTOS != 0
#include "FreeRTOS.h"
#include "task.h"
#endif

//===============================| Definitions |===============================
/**
 * \brief Size of RX buffer
 *
 * max - 1B Device ID, 1B Request ID, 2B for CMD ID and 4B for data value
 */
#define HW_BRIDGE_UNIPROT_RX_BUFFER_SIZE        8

/**
 * \brief Size of TX buffer
 *
 * 1B Device ID, 1B Return code, 2B CMD ID, 1B IN TYPE, 4B IN MIN, 4B IN MAX,\n
 * 1B OUT TYPE, 4B OUT MIN, 4B OUT MAX, 4B OUT VALUE + 2*MAX STRING SIZE
 */
#define HW_BRIDGE_UNIPROT_TX_BUFFER_SIZE        (26+2*GD_MAX_STRING_SIZE)
//===============================| Structures |================================
/**
 * \brief Possible states of bridge
 *
 * Also number of some states represent their "Command number". Especially:\n
 * \b HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_SETTINGS, \n
 * \b HW_BRIDGE_UNIPROT_STATE_REQUEST_SET_SETTINGS, \n
 * \b HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_METADATA, \n
 * \b HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_NUM_OF_DEV
 */
typedef enum{
	HW_BRIDGE_UNIPROT_STATE_WAITING_FOR_REQUEST = 0,      //!< HW_BRIDGE_UNIPROT_STATE_WAITING_FOR_REQUEST
	HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_SETTING = 1,     //!< HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_SETTINGS
	HW_BRIDGE_UNIPROT_STATE_REQUEST_SET_SETTINGS = 2,     //!< HW_BRIDGE_UNIPROT_STATE_REQUEST_SET_SETTINGS
	HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_METADATA = 3,     //!< HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_METADATA
	HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_NUM_OF_DEV = 4,   //!< HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_NUM_OF_DEV
	HW_BRIDGE_UNIPROT_STATE_SEND_ONLY_RETURN_CODE,        //!< HW_BRIDGE_UNIPROT_STATE_SEND_ONLY_RETURN_CODE
	HW_BRIDGE_UNIPROT_STATE_SEND_RETURN_CODE_AND_METADATA,//!< HW_BRIDGE_UNIPROT_STATE_SEND_RETURN_CODE_AND_METADATA
	HW_BRIDGE_UNIPROT_STATE_SEND_RETURN_CODE_AND_SETTING, //!< HW_BRIDGE_UNIPROT_STATE_SEND_RETURN_CODE_AND_SETTING


	/* This case never should happen, but may in debug
	 */
	HW_BRIDGE_UNIPROT_STATE_FAILED_TO_READ_METADATA       //!< HW_BRIDGE_UNIPROT_STATE_FAILED_TO_READ_METADATA
} BRIDGE_STATE;

//==========================| Functions prototypes |===========================
void bridge_init(void);


#if HW_BRIDGE_UNIPROT_SUPPORT_RTOS != 0
void bridge_task_FreeRTOS(void *pvParameters);
#endif

void bridge_task(void);



#endif
