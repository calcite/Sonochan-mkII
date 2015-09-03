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
 * @file
 *
 * @brief Extra functions for tinyprintf library
 *
 * Because tinyprintf does not include support for float, this library\n
 * contain some basic functions, that can transform float value to string\n
 * with some precision. Not good as in original sprintf(), but definitely\n
 * faster and smaller.
 *
 * Created:  2015/08/06
 * Modified: 2015/08/06
 *
 * @version 0.1
 * @author Martin Stejskal
 */


#ifndef _TINY_PRINTF_EXTRA_H_
#define _TINY_PRINTF_EXTRA_H_
//================================| Includes |=================================
#include <inttypes.h>

//================================| Functions |================================
//==========================| High level functions |===========================
uint8_t floatToStr(float f_number, uint8_t num_after_point, char *p_string);

//===========================| Mid level functions |===========================
uint8_t i32ToStr(int32_t i_num, char *p_string);
uint8_t u32ToStr(uint32_t i_num, char *p_string);

//===========================| Low level functions |===========================
char digitToStr(uint8_t i_digit);

#endif

