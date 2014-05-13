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

#ifndef _HW_H_
#define _HW_H_

//============================| Configure tables |=============================

/**
 * \name Device list for generic driver
 *
 * Here user define how is named variable with metadata of selected device.\n
 * Maximum supported devices (for now): 10.\n
 * Example of usage:\n
 * // Link to .h file\n
 * #include "some_driver.h"\n
 * // Name of structure with metadata\n
 * #define DEVICE0 some_driver_metadata\n
 * \n
 * #include "driver2.h"\n
 * #define DEVICE1 driver2_metadata\n
 * \n
 * #include "extra_driver.h"\n
 * #define DEVICE2 there_are_metadata_for_extra_driver
 *
 * @{
 */

/// \brief Board driver
#define DEVICE0         BRD_DRV_metadata
#include "brd_driver_hw_03.h"

/// \brief TLV320AIC33 codec
#include "tlv320aic33.h"
#define DEVICE1         TLV320AIC33_metadata

/// \brief Fractional PLL CS2200
#include "cs2200.h"
#define DEVICE2         CS2200_metadata

/// @}

//================================| Includes |=================================
#include <inttypes.h>

// Include also lower layer - generic driver
#include "generic_driver.h"


//=================================| Macros |==================================
#ifndef DEVICE0
#error "At least one device MUST be defined! Please set constant DEVICE0 at\
	.h file, where value is name of variable where are stored metadata.\
	Example: #define DEVICE0 TLV320AIC23_metadata"
// And Add default values, so no other compiler errors should shown
#define HW_MAX_DEVICE_ID	0
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0

#endif
#ifdef DEVICE10
#error "Sorry. Driver support only 10 devices. If you need more, then edit .h\
	file"
// And Add default values, so no other compiler errors should shown
#define HW_MAX_DEVICE_ID	0
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0
#endif


#ifdef DEVICE9
#define HW_MAX_DEVICE_ID	9
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0,\
	(gd_metadata*)&DEVICE1,\
	(gd_metadata*)&DEVICE2,\
	(gd_metadata*)&DEVICE3,\
	(gd_metadata*)&DEVICE4,\
	(gd_metadata*)&DEVICE5,\
	(gd_metadata*)&DEVICE6,\
	(gd_metadata*)&DEVICE7,\
	(gd_metadata*)&DEVICE8,\
	(gd_metadata*)&DEVICE9
#else
#ifdef DEVICE8
#define HW_MAX_DEVICE_ID	8
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0,\
	(gd_metadata*)&DEVICE1,\
	(gd_metadata*)&DEVICE2,\
	(gd_metadata*)&DEVICE3,\
	(gd_metadata*)&DEVICE4,\
	(gd_metadata*)&DEVICE5,\
	(gd_metadata*)&DEVICE6,\
	(gd_metadata*)&DEVICE7,\
	(gd_metadata*)&DEVICE8
#else
#ifdef DEVICE7
#define HW_MAX_DEVICE_ID	7
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0,\
	(gd_metadata*)&DEVICE1,\
	(gd_metadata*)&DEVICE2,\
	(gd_metadata*)&DEVICE3,\
	(gd_metadata*)&DEVICE4,\
	(gd_metadata*)&DEVICE5,\
	(gd_metadata*)&DEVICE6,\
	(gd_metadata*)&DEVICE7
#else
#ifdef DEVICE6
#define HW_MAX_DEVICE_ID	6
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0,\
	(gd_metadata*)&DEVICE1,\
	(gd_metadata*)&DEVICE2,\
	(gd_metadata*)&DEVICE3,\
	(gd_metadata*)&DEVICE4,\
	(gd_metadata*)&DEVICE5,\
	(gd_metadata*)&DEVICE6
#else
#ifdef DEVICE5
#define HW_MAX_DEVICE_ID	5
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0,\
	(gd_metadata*)&DEVICE1,\
	(gd_metadata*)&DEVICE2,\
	(gd_metadata*)&DEVICE3,\
	(gd_metadata*)&DEVICE4,\
	(gd_metadata*)&DEVICE5
#else
#ifdef DEVICE4
#define HW_MAX_DEVICE_ID	4
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0,\
	(gd_metadata*)&DEVICE1,\
	(gd_metadata*)&DEVICE2,\
	(gd_metadata*)&DEVICE3,\
	(gd_metadata*)&DEVICE4
#else
#ifdef DEVICE3
#define HW_MAX_DEVICE_ID	3
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0,\
	(gd_metadata*)&DEVICE1,\
	(gd_metadata*)&DEVICE2,\
	(gd_metadata*)&DEVICE3
#else
#ifdef DEVICE2
#define HW_MAX_DEVICE_ID	2
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0,\
	(gd_metadata*)&DEVICE1,\
	(gd_metadata*)&DEVICE2
#else
#ifdef DEVICE1
#define HW_MAX_DEVICE_ID	1
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0,\
	(gd_metadata*)&DEVICE1
#else
#ifdef DEVICE0
#define HW_MAX_DEVICE_ID	0
#define HW_POINTER_ARRAY_TO_METADATA	(gd_metadata*)&DEVICE0
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif


//===========================| Function prototypes |===========================

// Basic functions
GD_RES_CODE hw_get_setting(
    uint8_t i_device_ID,
    uint16_t i_cmd_ID,
    gd_config_struct *p_config_table);

GD_RES_CODE hw_set_setting(
    uint8_t i_device_ID,
    uint16_t i_cmd_ID,
    GD_DATA_VALUE i_data);

GD_RES_CODE hw_get_device_metadata(
    uint8_t i_device_ID,
    gd_metadata **p_device_metadata);

uint8_t hw_get_max_device_index(void);
#endif
