/**
 * \file
 *
 * \brief Virtual bridge, that connect HW library and universal protocol
 * library
 *
 *
 * Created  24.09.2013
 * Modified 02.04.2014
 *
 * \version 0.3
 * \author Martin Stejskal
 */
#ifndef _HW_BRIDGE_UNIPROT_H
#define _HW_BRIDGE_UNIPROT_H

//=================================| Options |=================================
/// \brief Define witch pipe bridge will use
#define HW_BRIDGE_UNIPROT_USED_PIPE             0

/**
 * \brief Debug version - 1 (debug enabled) or 0 (debug disabled)
 *
 * When software is in develop stage there can be a lot unexpected behavior.\n
 * So in debug mode is there more checking, witch in fully working code have\n
 * no meaning. Anyway sometimes this redundant checking is good when debugging.
 */
#define bridge_version_debug                    1

//=========================| Some checks of settings |=========================

#if (bridge_version != 1) && (bridge_version != 0)
#error "Please select one of the option for version debug: 1 or 0"
#endif
//================================| Includes |=================================
// HW library
#include "HW.h"

// Universal protocol library
#include "uniprot.h"

// FreeRTOS stuff
#ifdef FREERTOS_USED
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


#ifdef FREERTOS_USED
void bridge_task_FreeRTOS(void *pvParameters);
#endif

void bridge_task(void);



#endif
