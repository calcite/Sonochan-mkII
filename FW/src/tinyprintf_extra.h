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

