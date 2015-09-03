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

#include "tinyprintf_extra.h"

//==========================| High level functions |===========================
/**
 * @brief Transform float value to string
 * @param f_number
 * @param num_after_point
 * @param p_string
 * @return
 */
uint8_t floatToStr(float f_number, uint8_t num_after_point, char *p_string)
{
  // Count characters that were written to buffer
  uint8_t num_of_char = 0;

  /* Check input number. If it is higher than "0xFFFF FFFF" (32 bit), then
   * we are not able to recalculate it without overflow -> error
   * To be sure that constants will be more platform independent (32 bit and
   * 64 bit systems), so pure numbers are used. Now it is problem of compiler.
   */
  if(f_number > (float)(2147483647L) )
  {
    // Too high value. So far no character written
    return num_of_char;
  }
  if(f_number < (float)(int32_t)(-2147483647L) )
  {
    // Too low value. So far not character written
    return num_of_char;
  }
  // There is also integer limits.
  if(num_after_point > 9)
  {
    // Can not count this precision by following simple algorythm
    return num_of_char;
  }

  // Convert float to int. So we lost everything behind decal point
  int32_t num_bef_pnt = (int32_t)f_number;

  /* Add to string. Because float can be signed, we use function for signed
   * values
   */
  num_of_char = i32ToStr(num_bef_pnt, p_string);

  /* And now decimal point thing. Point is that we just grab original number
   * multiply it 10^(num_after_point) and minus "cut int"*10^(num_after_point).
   * Difference save as int and transform it to characters
   */
  if(num_after_point > 0)
  {
    // Add more precise numbers (rewrite last null)
    p_string = p_string + num_of_char -1;
    *(p_string++) = '.';
    num_of_char++;

    // Keep information about one digit value
    uint8_t i_digit;

    // Get rid of sign
    if(f_number < 0)
    {
      f_number = (-1) * f_number;
      num_bef_pnt = (-1) * num_bef_pnt;
    }

    // Get rid of decimal point
    f_number = f_number - (float)num_bef_pnt;

    uint8_t i;

    for(i = 0 ; i<num_after_point ; i++)
    {
      /* Every cycle "shift" (multiply by 10) difference, add one digit to
       * string and minus "new value"
       */
      f_number = f_number * 10;

      i_digit = (uint8_t)f_number;
      *(p_string++) = digitToStr(i_digit);
      num_of_char++;

      f_number = f_number - (float)i_digit;
    }

    // Add NULL (because we deleted old one)
    *(p_string) = 0x00;
    num_of_char++;
  }

  return num_of_char;
}
//===========================| Mid level functions |===========================
inline uint8_t i32ToStr(int32_t i_num, char *p_string)
{
  uint8_t num_of_char = 0;

  /* Check if number is negative -> add "-" and move pointer and change number
   * to unsigned -> simple calculations
   */
  if(i_num < 0)
  {
    *p_string++ = '-';
    num_of_char++;
    i_num = (-1) * i_num;
  }

  num_of_char = num_of_char + u32ToStr(i_num, p_string);

  return num_of_char;
}


inline uint8_t u32ToStr(uint32_t i_num, char *p_string)
{
  // Counts written characters
  uint8_t num_of_char = 0;

  // Counts decades. For 32 bit it is 10 (10 decades). For 8 bit it is 3
  int8_t i_decade = 10;
  // Base value - for 32 bit it is 1 000 000 000 (highest decade)
  uint32_t i_base = 1e9;
  // Multiply this number by base value
  int8_t i_multiplier;
  /* Because 2^32 can be something ove 4 000 000 000 -> start at 9 000 000 000
   * does not make sense and also side effect are errors (because of comparing
   * 32 bit and wider value). To avoid this mess, we just start multiplier
   * at 4
   */
  uint8_t i_start_multiplier = 4;

  /* Calculated base value * multiplier. Have to be for 1 decade higher than
   * maximum number value (binary and decimal decades are not "synchronized")
   */
  uint32_t i_base_multip;

  // Just status, that keep information if first non-zero number was found
  uint8_t i_fnd_non_zero = 0;

  // Check only one exception: 0
  if(i_num == 0)
  {
    *(p_string++) = '0';
    *(p_string++) = 0x00;
    num_of_char = 2;
    return num_of_char ;
  }

  // For every single decade
  for(i_decade = 10; i_decade > 0 ; i_decade--)
  {
    // In every decade find appropriate value
    for(i_multiplier = i_start_multiplier; i_multiplier >= 0 ; i_multiplier--)
    {
      i_base_multip = (i_base * i_multiplier);
      if(i_base_multip <= i_num)
      {
        /* We found number. So add it to string, increase num_of_char,
         * decrease number itself, and break this loop. Of course, there is
         * nice exception to get rid of leading zeros, which nobody likes
         */
        if((i_multiplier == 0) && (i_fnd_non_zero == 0))
        {
          /* In case we found zero and so far only zeros were found, just
           * break this loop
           */
          break;
        }
        *(p_string++) = digitToStr(i_multiplier);
        num_of_char++;
        i_num = i_num - i_base_multip;
        i_fnd_non_zero = 1;
        break;
      }
    }
    // Set multiplier for next decade
    i_start_multiplier = 9;

    // Because next "round" we move to lower decade
    i_base = i_base/10;
  }

  // And add NULL
  *(p_string++) = 0x00;
  num_of_char++;

  return num_of_char;
}


//===========================| Low level functions |===========================
inline char digitToStr(uint8_t i_digit)
{
  char c_digit;

  switch(i_digit)
  {
  case 0:       c_digit = '0';      break;
  case 1:       c_digit = '1';      break;
  case 2:       c_digit = '2';      break;
  case 3:       c_digit = '3';      break;
  case 4:       c_digit = '4';      break;
  case 5:       c_digit = '5';      break;
  case 6:       c_digit = '6';      break;
  case 7:       c_digit = '7';      break;
  case 8:       c_digit = '8';      break;
  case 9:       c_digit = '9';      break;
  case 10:      c_digit = 'A';      break;
  case 11:      c_digit = 'B';      break;
  case 12:      c_digit = 'C';      break;
  case 13:      c_digit = 'D';      break;
  case 14:      c_digit = 'E';      break;
  case 15:      c_digit = 'F';      break;
  // If not known -> error
  default:      c_digit = '?';      break;
  }

  return c_digit;
}

