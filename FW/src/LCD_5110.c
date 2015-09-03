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
 * \brief Generic driver for LCD used in nokia 5110
 *
 * Used for basic operations (init, clear, write text, ...) with LCD 5510.\n
 * It need HAL driver (which depend on used architecture) to work properly.\n
 *
 * Created:  04.07.2012\n
 * Modified: 11.09.2014
 *
 * \version 1.2
 * \author Martin Stejskal
 */

#include "LCD_5110.h"

//=========================| Font for LCD nokia 5110 |=========================
#ifndef _LCD_5110_FONT_NOKIA_5110_
#define _LCD_5110_FONT_NOKIA_5110_
//! Binary appearance all symbols in array
const unsigned char LCD_5110_font6x8[][6] =
{
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp	   32
{ 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !	   33
{ 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "	   34
{ 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #	   35
{ 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $	   36
{ 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %	   37
{ 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &	   38
{ 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '	   39
{ 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (	   40
{ 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )	   41
{ 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *	   42
{ 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +	   43
{ 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,	   44
{ 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -	   45
{ 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .	   46
{ 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /	   47
{ 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 0	   48
{ 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 1	   49
{ 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 2	   50
{ 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 3	   51
{ 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 4	   52
{ 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 5	   53
{ 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 6	   54
{ 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 7	   55
{ 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 8	   56
{ 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 9	   57
{ 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :	   58
{ 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;	   59
{ 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <	   60
{ 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =	   61
{ 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >	   62
{ 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?	   63
{ 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @	   64
{ 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A	   65
{ 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B	   66
{ 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C	   67
{ 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D	   68
{ 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E	   69
{ 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F	   70
{ 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G	   71
{ 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H	   72
{ 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I	   73
{ 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J	   74
{ 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K	   75
{ 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L	   76
{ 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M	   77
{ 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N	   78
{ 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O	   79
{ 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P	   80
{ 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q	   81
{ 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R	   82
{ 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S	   83
{ 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T	   84
{ 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U	   85
{ 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V	   86
{ 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W	   87
{ 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X	   88
{ 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y	   89
{ 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z	   90
{ 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [	   91
{ 0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55 },   // 55	   92
{ 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]	   93
{ 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^	   94
{ 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _	   95
{ 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '	   96
{ 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a	   97
{ 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b	   98
{ 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c	   99
{ 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d	   100
{ 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e	   101
{ 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f	   102
{ 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g	   103
{ 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h	   104
{ 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i	   105
{ 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j	   106
{ 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k	   107
{ 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l	   108
{ 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m	   109
{ 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n	   110
{ 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o	   111
{ 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p	   112
{ 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q	   113
{ 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r	   114
{ 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s	   115
{ 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t	   116
{ 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u	   117
{ 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v	   118
{ 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w	   119
{ 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x	   120
{ 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y	   121
{ 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z	   122
{ 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }    // horiz lines        123
};
#endif


//===============================| Structures |================================
/**
 * \brief Structure which easily allow split 32bit value into 4 8bit variables
 */
typedef union{
  struct{
    uint32_t i_32bit;
  };
  struct{
#if defined(__ORDER_BIG_ENDIAN__) || defined(__AVR32__)
    /// \note On AVR32 is in structure MSB first. On x86 it is opposite!
    uint8_t i_8bit_3;   /// MSB Byte
    uint8_t i_8bit_2;
    uint8_t i_8bit_1;
    uint8_t i_8bit_0;   /// LSB Byte
#else
    uint8_t i_8bit_0;	/// LSB Byte
    uint8_t i_8bit_1;
    uint8_t i_8bit_2;
    uint8_t i_8bit_3;	/// MSB Byte
#endif
  };
}lcd_5110_32bits;


//============================| Global variables |=============================

/**
 * \brief Global variables for counting LCD "lines" and determinate X position
 *
 * @{
 */

/**
 * Counts "lines" on LCD, so next word can be easy written to next line\n
 * (easy to calculate)
 */
static uint8_t i_LCD_line_counter = 0;

/**
 * Determine X position on LCD
 */
static uint8_t i_LCD_x_position = 0;

/**
 * If is set to 1, after writing text thru LCD_5110_write() is new message\n
 * written new line. If value is 0, then new message continue on actual line\n
 * is possible.
 */
static uint8_t i_add_LF_after_write = LCD_5110_DEFAULT_AUTO_LF;

/**
 * If is cleared, then after writing text to last line continue on line 0.\n
 * All old informations stay on display. However in some cases is good idea\n
 * to clear display before continue again on line 0. Something as book page.\n
 * So if you want this feature, set this to one.
 */
static uint8_t i_auto_clear_display_flag = LCD_5110_DEFAULT_AUTO_LCD_CLEAR;

/**
 * If bit is set, then always before writing (LCD_5110_write_*) clear line.\n
 * In most cases it can be useful, but not every time. So there is option.\n
 * This can be set also in runtime.
 */
static uint8_t i_auto_clear_line_flag = LCD_5110_DEFAULT_AUTO_LINE_CLEAR;
/// @}


//=============================| FreeRTOS stuff |==============================
// If RTOS support is enabled, create this
#if LCD_5110_SUPPORT_RTOS != 0
portBASE_TYPE xStatus;
xSemaphoreHandle mutexSPI;
#endif

//==================| Function prototypes not seen by user |===================
/**
 * \brief Print only one digit (0~F)
 * @param i_digit Number to show. Minimum value is 0, maximum is 0xF.\n
 * Anything else will not be displayed properly
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_show_digit(uint8_t i_digit);

/**
 * \brief Convert input hexadecimal number to string and add it to input\n
 *  string variable
 *
 * @param p_string Pointer to string. To this string will be written result
 * @param i_hex_number 8bit number. Assume binary format
 */
void LCD_5110_add_8bit_hex_to_sring( char *p_string, uint8_t i_hex_number);


/**
 * \brief From given number get tens and add tens to string
 *
 *
 * @param p_string Pointer to string array. To this string will be written
 * @param i_dec_number Number 0~99. If higher then behavior can be unexpected
 * @return Tens (10, 20, 30 and so on) depend what detect
 */
uint8_t LCD_5110_add_tens_to_string(char *p_string,
                                    uint8_t i_dec_number);

/**
 * \brief From given number get units and add it to string
 *
 * @param p_string Pointer to string array. To this string will be written
 * @param i_dec_number Number 0~9. If higher then behavior can be unexpected
 */
void LCD_5110_add_unit_to_string(char *p_string, uint8_t i_dec_number);

/**
 * \brief Convert 8bit unsigned binary number to text value
 * @param p_string Pointer to string array. To this string will be written
 * @param i_dec_number 8bit unsigned value: 0~255
 */
void LCD_5110_add_8bit_dec_unsigned_to_string(char *p_string,
                                              uint8_t i_dec_number);

/**
 * \brief Convert 16bit unsigned binary number to text value
 * @param p_string Pointer to string array. To this string will be written
 * @param i_dec_number 16bit unsigned value: 0~65535
 */
void LCD_5110_add_16bit_dec_unsigned_to_string(char *p_string,
                                               uint16_t i_dec16);

//================================| Functions |================================

inline e_lcd_5110_status LCD_5110_init(void){
  // Variable for status value - default value as error - just for case
  e_lcd_5110_status e_status = LCD_5110_ERROR;

  // If RTOS support enable and create flag is set then create mutex
#if (LCD_5110_SUPPORT_RTOS != 0) && (LCD_5110_RTOS_CREATE_MUTEX != 0)
  mutexSPI = xSemaphoreCreateMutex();
#endif

  // Lock SPI device if RTOS support enabled (because we use HAL functions)
  LCD_5110_LOCK_SPI_MODULE_IF_RTOS
  // Initialize hardware
  e_status = LCD_5110_HAL_init();

  // Unlock SPI device...
  LCD_5110_UNLOCK_SPI_MODULE_IF_RTOS

  // Check status - if not OK, then return status to higher layer
  if(e_status != LCD_5110_OK)
  {
    return e_status;
  }

  // Clear nokia 5510 display
  e_status = LCD_5110_clear();
  if(e_status != LCD_5110_OK)
  {
    return e_status;
  }


  // And disable chip select on nokia 5510 LCD - already done by LCD_debug_clear

  return LCD_5110_OK;
};


//=============================================================================


e_lcd_5110_status LCD_5110_clear(void)
{
  // Variable for status value - default value as error - just for case
  e_lcd_5110_status e_status = LCD_5110_ERROR;

  int i;

  // And clear N5110 LCD
  for (i=0 ; i<504 ; i++)	// (48x84)/8=504
  {
    // Write 0x00, which means "all showed bits to zero"
    e_status = LCD_5110_send_data_byte(0x00);
    if(e_status != LCD_5110_OK)
    {
      return e_status;
    }
  }

  // Set position coordinates on LCD to 0,0
  LCD_5110_set_X(0);
  LCD_5110_set_line(0);

  // Set inform coordinates (use in future)
  i_LCD_line_counter = 0;	// Line is zero ("first" line on real display)
  i_LCD_x_position = 0;	// X coordinate is set to begin -> zero

  return LCD_5110_OK;
};


//=============================================================================


e_lcd_5110_status LCD_5110_write(const char * p_txt_to_show)
{
  // Variable for status value - default value as error - just for case
  e_lcd_5110_status e_status = LCD_5110_ERROR;

  // for counting pixels, that are already written to LCD
  uint8_t i;

  // Symbol ASCII number is recalculated
  uint8_t symbol_ascii_number;

  // If user want clear display and if we are on first line
  if((i_auto_clear_display_flag != 0) && i_LCD_line_counter == 0)
  {
    // Clear display and check result
    e_status = LCD_5110_clear();
    if(e_status != LCD_5110_OK)
    {
      return e_status;
    }
  }
  else
  {// Check if user want at least clear actual line
    if(i_auto_clear_line_flag != 0)
    {
      // Clear actual line
      e_status = LCD_5110_clear_line(i_LCD_line_counter);
      if(e_status != LCD_5110_OK)
      {
        return e_status;
      }
    }
  }

  // Print symbols, until is found NULL character -> end of string
  while ( *p_txt_to_show != 0x00 )
  {
    /* Test if actual symbol is '\n' -> this means "jump to next line" -> must
     * change line number.
     */
    if ( (*(p_txt_to_show) == '\n' ))
    {
      // X position on zero
      i_LCD_x_position = 0;

      // And change number of actual line (plus one -> move to next)
      i_LCD_line_counter++;

      // Set X address in LCD to zero
      e_status = LCD_5110_set_X(0);

      if(e_status != LCD_5110_OK)
      {
        return e_status;
      }

      // Move to next line
      e_status = LCD_5110_set_line(i_LCD_line_counter);

      if(e_status != LCD_5110_OK)
      {
        return e_status;
      }

      /* Move pointer forward by one (to next character, we won't write line
       * feed on LCD)
       */
      p_txt_to_show++;
      // Anyway test if this newline was the last character -> break this cycle
      if(*p_txt_to_show == 0x00)
      {
        // Stop reading new bytes
        break;
      }

      // Check if user want clear line by default
      if(i_auto_clear_line_flag != 0)
      {
        // Clear actual line
        e_status = LCD_5110_clear_line(i_LCD_line_counter);
        if(e_status != LCD_5110_OK)
        {
          return e_status;
        }
      }
    }


    /* Test if next symbol fit on actual line or not. If not, then jump to next
     * line and set X coordinates to 0
     */
    if (i_LCD_x_position > (83-6+1) )
    /* 83 is maximum address, 6 is symbol length however, when to 0 is
     * added 6 -> new X position is 6, but it should be 5 (cause we started
     * from zero) -> so that is reason +1 offset
     */

    {
      // Set X address to zero
      i_LCD_x_position = 0;
      // Move to next line
      i_LCD_line_counter++;

      // Set X to zero
      e_status = LCD_5110_set_X(0);

      if(e_status != LCD_5110_OK)
      {
        return e_status;
      }

      // And move to next line
      e_status = LCD_5110_set_line(i_LCD_line_counter);

      if(e_status != LCD_5110_OK)
      {
        return e_status;
      }

      // If user want clear line before writing
      if(i_auto_clear_line_flag != 0)
      {
        // Clear actual line
        e_status = LCD_5110_clear_line(i_LCD_line_counter);
        if(e_status != LCD_5110_OK)
        {
          return e_status;
        }
      }
    }


    // Calculate from ASCII code number corresponding with "LCD_5110_font6x8" database
    symbol_ascii_number = *(p_txt_to_show++) -32;	// Get ASCII value from actual symbol (font begins at 32 ASCII character)


    // every symbol is 6 points wide
    for (i=0 ; i<6 ; i++)
    {
      e_status = LCD_5110_send_data_byte(
                                      LCD_5110_font6x8[symbol_ascii_number][i]);
      if(e_status != LCD_5110_OK)
      {
        return e_status;
      }
    }

    // After write symbol -> i_LCD_x_position should be increased by 6 (symbol length)
    i_LCD_x_position = i_LCD_x_position + 6;	// 6 points per sample -> X will "move" by 6
  }

  // Check i_add_LF_after_write
  if(i_add_LF_after_write != 0)
  {
    // If not zero -> move to new line

    // Move to next line -> for next call of this function
    i_LCD_line_counter++;
    e_status = LCD_5110_set_line(i_LCD_line_counter);

    // Check
    if(e_status != LCD_5110_OK)
    {
      return e_status;
    }

    // Change X position to begin of line
    i_LCD_x_position = 0;

    e_status = LCD_5110_set_X(0);
  }

  // Just return status
  return e_status;
}


//=============================================================================


inline e_lcd_5110_status LCD_5110_write_xy(const char * p_txt_to_show,
                                           uint8_t x_position, uint8_t y_line)
{
  // Variable for status value - default value as error - just for case
  e_lcd_5110_status e_status = LCD_5110_ERROR;

  // Temporary variable for backup auto clear feature
  uint8_t i_autoclear_backup = i_auto_clear_line_flag;

  // If You like counting started 1, instead of 0 -> uncomment following two lines
  //y_line--;		// LCDs calculate lines by 0 -> minus one
  //x_position--;	// again, for LCD is begin 0

  // Coordinates set, so set them on N5110 LCD
  e_status = LCD_5110_set_line(y_line);
  if( e_status != LCD_5110_OK)
  {
    return e_status;
  }

  e_status = LCD_5110_set_X(x_position);
  if( e_status != LCD_5110_OK)
  {
    return e_status;
  }

  // Temporary turn off auto clear feature
  i_auto_clear_line_flag = 0;

  // And finally write data on LCD(s)
  e_status = LCD_5110_write( p_txt_to_show);

  // Anyway restore auto clear value
  i_auto_clear_line_flag = i_autoclear_backup;

  // If all OK -> return OK
  return e_status;
}

//=============================================================================
e_lcd_5110_status LCD_5110_clear_line(uint8_t y_line)
{
  // Variable for status value
  e_lcd_5110_status e_status;;

  // Set line
  e_status = LCD_5110_set_line(y_line);
  if(e_status != LCD_5110_OK)
  {
    return e_status;
  }

  // Set X coordinates to 0
  e_status = LCD_5110_set_X(0);
  if(e_status != LCD_5110_OK)
  {
    return e_status;
  }

  // Simple counter
  uint8_t i_cnt;

  // Clear line
  for(i_cnt=0 ; i_cnt < 84 ; i_cnt++)
  {
    e_status = LCD_5110_send_data_byte(0x00);
    if(e_status != LCD_5110_OK)
    {
      return e_status;
    }
  }

  /* OK, we set whole line, but LCD itself changed line number. So, let's set
   * line back
   */
  e_status = LCD_5110_set_line(y_line);
  if(e_status != LCD_5110_OK)
  {
    return e_status;
  }

  // Set X coordinates back to 0
  return LCD_5110_set_X(0);
}

//=============================================================================


inline e_lcd_5110_status LCD_5110_set_X(uint8_t X_address)
{
  // Variable for status value - default value as error - just for case
  e_lcd_5110_status e_status = LCD_5110_ERROR;

  // Test for input variable
  if (X_address > 83)	// Should not be bigger than 83 -> fail
  {
    X_address = 0;
  }


  // Set X-address, command is 0x80 -> result command is (0x80 | X_address)
  e_status = LCD_5110_send_command_byte(X_address | 0x80);

  // Check if status is OK. If not, return error status
  if(e_status != LCD_5110_OK)
  {
    return e_status;
  }

  // Save information about X position
  i_LCD_x_position = X_address;

  return LCD_5110_OK;
}


//=============================================================================


inline e_lcd_5110_status LCD_5110_set_line(uint8_t y_line)
{
  // Save information about Y position. Then program will work just with "i_LCD_line_counter"
  i_LCD_line_counter = y_line;

  // Test for input variable
  if (i_LCD_line_counter > 5)	// Should not be bigger than 5 -> fail
  {
    i_LCD_line_counter = 0;
  }


  // Set Y-address, command is 0x40 -> result command is (0x40 | X_address)
  return LCD_5110_send_command_byte(i_LCD_line_counter | 0x40);


}


//=============================================================================
e_lcd_5110_status LCD_5110_write_to_line(uint8_t i_raw_data)
{
  // Variable for status value
  e_lcd_5110_status e_status;;

  // Set X coordinates to 0
  e_status = LCD_5110_set_X(0);
  if(e_status != LCD_5110_OK)
  {
    return e_status;
  }

  // Simple counter
  uint8_t i_cnt;

  for(i_cnt=0 ; i_cnt < 84 ; i_cnt++)
  {
    e_status = LCD_5110_send_data_byte(i_raw_data);
    if(e_status != LCD_5110_OK)
    {
      return e_status;
    }
  }

  /* OK, we set whole line, but LCD itself changed line number. So, let's set
   * line back
   */
  e_status = LCD_5110_set_line(i_LCD_line_counter);
  if(e_status != LCD_5110_OK)
  {
    return e_status;
  }

  // Set X coordinates back to 0
  return LCD_5110_set_X(0);
}

//============================| Runtime settings |=============================
inline void LCD_5110_auto_newline(uint8_t i_auto_LF_flag)
{
  i_add_LF_after_write = i_auto_LF_flag;
}

inline void LCD_5110_auto_clear_display(uint8_t i_auto_clear_flag)
{
  i_auto_clear_display_flag = i_auto_clear_flag;
}

inline void LCD_5110_auto_clear_line(uint8_t i_auto_clear_flag)
{
  i_auto_clear_line_flag = i_auto_clear_flag;
}

//============================| Special functions |============================

inline e_lcd_5110_status LCD_5110_write_hex8(uint8_t i_hex8)
{
  // Temporary buffer for text. Add prefix "0x" by default
  char c_text[5] = {'0', 'x'};

  // Add number to string array
  LCD_5110_add_8bit_hex_to_sring(&c_text[2], i_hex8);

  // Return value from write function
  return LCD_5110_write(&c_text[0]);
}





inline e_lcd_5110_status LCD_5110_write_hex16(uint16_t i_hex16)
{
  // Temporary buffer for text
  char c_text[7] = {'0', 'x'};

  // Little bit complicated. Must be split into two steps (2x add 8bit value)

  // Step 1) MSB Byte
  LCD_5110_add_8bit_hex_to_sring(&c_text[2], (i_hex16>>8));

  // Step 2) LCD Byte
  LCD_5110_add_8bit_hex_to_sring(&c_text[4], (i_hex16 & 0xFF));

  // Return value from write function
  return LCD_5110_write(&c_text[0]);
}



inline e_lcd_5110_status LCD_5110_write_hex32(uint32_t i_hex32)
{
  // Temporary buffer for text
  char c_text[11] = {'0', 'x'};

  /* There it is a little bit complicated. So it is useful somehow split 32bit
   * value to 8 bit value. For this is there union structure that can handle
   * this
   */
  lcd_5110_32bits u_32bit_value;

  u_32bit_value.i_32bit = i_hex32;

  // Now just add Byte by Byte to string variable... So easy
  LCD_5110_add_8bit_hex_to_sring(&c_text[2], u_32bit_value.i_8bit_3);
  LCD_5110_add_8bit_hex_to_sring(&c_text[4], u_32bit_value.i_8bit_2);
  LCD_5110_add_8bit_hex_to_sring(&c_text[6], u_32bit_value.i_8bit_1);
  LCD_5110_add_8bit_hex_to_sring(&c_text[8], u_32bit_value.i_8bit_0);

  // Return value from write function
  return LCD_5110_write(&c_text[0]);
}




e_lcd_5110_status LCD_5110_write_dec8_unsigned(uint8_t i_dec8)
{
  // Unsigned -> 0~255 -> text array @ 4 characters (3 numbers + NULL)
  char c_text[4];

  // Convert binary number to decade
  LCD_5110_add_8bit_dec_unsigned_to_string(&c_text[0], i_dec8);

  // Print and return status
  return LCD_5110_write(&c_text[0]);
}



e_lcd_5110_status LCD_5110_write_dec8_signed(int8_t i_dec8)
{
  // Text array - 1 sign (if needed), 3 numbers (max), 1 NULL -> 5
  char c_text[5];

  // Test sign
  if(i_dec8 < 0)
  {
    // Negative

    // Add "-"
    c_text[0] = '-';
    // Remove sign
    i_dec8 = ~i_dec8;
    i_dec8++;

    // Add string to array
    LCD_5110_add_8bit_dec_unsigned_to_string(&c_text[1], (uint8_t)i_dec8);
  }
  else
  {
    // Positive

    // Just add string to array
    LCD_5110_add_8bit_dec_unsigned_to_string(&c_text[0], (uint8_t)i_dec8);
  }

  // Print and return status
  return LCD_5110_write(&c_text[0]);
}



e_lcd_5110_status LCD_5110_write_dec16_unsigned(uint16_t i_dec16)
{
  // max value 65535 -> 5 digits + NULL
  char c_text[6];

  // Convert and add number to string array
  LCD_5110_add_16bit_dec_unsigned_to_string(&c_text[0], i_dec16);

  // And write all to LCD
  return LCD_5110_write(&c_text[0]);
}



e_lcd_5110_status LCD_5110_write_dec16_signed(int16_t i_dec16)
{
  // max value -65536 -> 6 digits + NULL
  char c_text[7];

  // Check for sign
  if(i_dec16 < 0)
  {
    // Negative number

    // Add "-"
    c_text[0] = '-';
    // Remove sign
    i_dec16 = ~i_dec16;
    i_dec16++;

    // Add to string array
    LCD_5110_add_16bit_dec_unsigned_to_string(&c_text[1], (uint16_t)i_dec16);
  }
  else
  {
    // Positive number

    // Just add string to array
    LCD_5110_add_16bit_dec_unsigned_to_string(&c_text[0], (uint16_t)i_dec16);
  }

  // Finally, write to LCD
  return LCD_5110_write(&c_text[0]);
}


//===========================| Low level functions |===========================

e_lcd_5110_status LCD_5110_send_data_byte(uint8_t i_data)
{
  // For store status
  e_lcd_5110_status e_status;

  // Lock SPI device if RTOS support is enabled
  LCD_5110_LOCK_SPI_MODULE_IF_RTOS

  // Send data through HAL
  e_status = LCD_5110_HAL_send_data_byte(i_data);

  // Unlock if RTOS...
  LCD_5110_UNLOCK_SPI_MODULE_IF_RTOS

  return e_status;
}

e_lcd_5110_status LCD_5110_send_command_byte(uint8_t i_cmd)
{
  // For store status
  e_lcd_5110_status e_status;

  // Lock SPI device if RTOS support is enabled
  LCD_5110_LOCK_SPI_MODULE_IF_RTOS

  // Send data through HAL
  e_status = LCD_5110_HAL_send_command_byte(i_cmd);

  // Unlock if RTOS...
  LCD_5110_UNLOCK_SPI_MODULE_IF_RTOS

  return e_status;
}




//==================| Function prototypes not seen by user |===================
inline e_lcd_5110_status LCD_5110_show_digit(uint8_t i_digit)
{
  // Temporary buffer for text - clean by NULL characters
  char c_text[2] = {0x00, 0x00};

  // Test digit - if <10 --> add different ASCII offset
  if(i_digit < 10)
  {
    // If lower than 10 -> add ASCII offset 48 (which is character '0')
    i_digit = i_digit + 48;
  }
  else
  {
    // Else add ASCII offset 55 -> character 'A' + i_digit value
    i_digit = i_digit + 55;
  }

  // Anyway - load i_digit to text variable
  c_text[0] = (char)i_digit;

  // Return value from write function
  return LCD_5110_write(&c_text[0]);
}






void LCD_5110_add_8bit_hex_to_sring( char *p_string, uint8_t i_hex_number)
{
  /* First take high 4 bits and convert them. However we need figure out
   * which ASCII offset must be added
   */
  if((i_hex_number>>4) < 10)
  {
    // If value is 0~9
    *(p_string++) = (i_hex_number>>4) + '0';
  }
  else
  {
    // Else there is A~F
    *(p_string++) = (i_hex_number>>4) + ('A'-10);
  }


  // OK, so far so good. Let's add 4 low bits
  if((i_hex_number & 0x0F) < 10 )
  {
    // Again, low 4 bits -> add correct ASCII offset
    *(p_string++) = (i_hex_number & 0x0F) + '0';
  }
  else
  {
    // Else there is A~F -> again, just set correct offset
    *(p_string++) = (i_hex_number & 0x0F) + ('A'-10);
  }


  // Finally add NULL
  *p_string = 0x00;
}



inline void LCD_5110_add_8bit_dec_unsigned_to_string(char *p_string,
                                                     uint8_t i_dec8)
{
  // Test if number is higher than 199 -> if yes, add to text "2"
  if(i_dec8 >= 200)
  {
    // 200~255
    // Hundreds
    *(p_string++) = '2';
    // Tens - just remove hundreds.
    i_dec8 = i_dec8 - 200;

    // Remove tens and add tens character to text array
    i_dec8 = i_dec8 -
             LCD_5110_add_tens_to_string(p_string++, i_dec8);
    // Add units to text array
    LCD_5110_add_unit_to_string(p_string++, i_dec8);
  }
  else if(i_dec8 >= 100)
  {
    // 100~199
    *(p_string++) = '1';
    i_dec8 = i_dec8 - 100;
    i_dec8 = i_dec8 -
        LCD_5110_add_tens_to_string(p_string++, i_dec8);
    LCD_5110_add_unit_to_string(p_string++, i_dec8);
  }
  else
  {
    // 0~99 -> no hundreds -> just test tens if 0 or not
    if(i_dec8 < 10)
    {
      // 0~9 -> just units
      LCD_5110_add_unit_to_string(p_string++, i_dec8);
    }
    else
    {
      // 10~99 -> add tens
      i_dec8 = i_dec8 -
          LCD_5110_add_tens_to_string(p_string++, i_dec8);
      // And add units
      LCD_5110_add_unit_to_string(p_string++, i_dec8);
    }
  }

  // Add NULL character
  *(p_string) = 0x00;
}


inline void LCD_5110_add_16bit_dec_unsigned_to_string(char *p_string,
                                                      uint16_t i_dec16)
{
  // Counter for decades
  uint8_t i_cnt = 0;

  // Changed to 0 when any found decade is not zero
  uint8_t i_still_zero = 1;


  // First let's count tens of thousands
  while(i_dec16 >= 10000)
  {
    // Decrease number by 10 000
    i_dec16 = i_dec16 -10000;
    // And increase counter
    i_cnt++;
  }
  // When done test counter -> if not 0 -> write number to c_text
  if(i_cnt != 0)
  {
    // Add digit to array
    *(p_string++) = '0' + i_cnt;
    // Found non zero value -> change i_still_zero to 0
    i_still_zero = 0;

    // And clear counter
    i_cnt = 0;
  }



  // OK, now thousands
  while(i_dec16 >= 1000)
  {
    i_dec16 = i_dec16 - 1000;
    i_cnt++;
  }

  // Again check for non zero i_cnt or i_still_zero flag
  if((i_cnt != 0) || (i_still_zero == 0))
  {
    // If digit should be written...
    *(p_string++) = '0' + i_cnt;
    i_still_zero = 0;

    // Clear counter
    i_cnt = 0;
  }


  // Test for hundreds
  while(i_dec16 >= 100)
  {
    i_dec16 = i_dec16 - 100;
    i_cnt++;
  }
  // Check
  if((i_cnt != 0) || (i_still_zero == 0))
  {
    *(p_string++) = '0' + i_cnt;
    i_still_zero = 0;
    i_cnt = 0;
  }

  // Tens
  while(i_dec16 >= 10)
  {
    i_dec16 = i_dec16 - 10;
    i_cnt++;
  }
  // Check
  if((i_cnt != 0) || (i_still_zero == 0))
  {
    *(p_string++) = '0' + i_cnt;
    i_still_zero = 0;
    i_cnt = 0;
  }

  // So now in i_dec16 should be just units - write anyway
  *(p_string++) = '0' + i_dec16;

  // Add NULL character
  *(p_string) = 0x00;
}




inline uint8_t LCD_5110_add_tens_to_string(char *p_string,
                                    uint8_t i_dec_number)
{
  // Test decade by decade
  if(i_dec_number < 10)
  {
    // 0~9 -> decade is certainly 0
    // Add ASCII 0
      *(p_string) = '0';
    // 0 tens detected
    return 0;
  }
  else if(i_dec_number < 20)
  {
    // 10~19
    *(p_string) = '1';
    return 10;
  }
  else if(i_dec_number < 30)
  {
    // 20~29
    *(p_string) = '2';
    return 20;
  }
  else if(i_dec_number < 40)
  {
    // 30~39
    *(p_string) = '3';
    return 30;
  }
  else if(i_dec_number < 50)
  {
    // 40~49
    *(p_string) = '4';
    return 40;
  }
  else if(i_dec_number < 60)
  {
    // 50~59
    *(p_string) = '5';
    return 50;
  }
  else if(i_dec_number < 70)
  {
    // 60~69
    *(p_string) = '6';
    return 60;
  }
  else if(i_dec_number < 80)
  {
    // 70~79
    *(p_string) = '7';
    return 70;
  }
  else if(i_dec_number < 90)
  {
    // 80~89
    *(p_string) = '8';
    return 80;
  }
  else
  {
    // 90~??? (expect maximum 99)
    *(p_string) = '9';
    return 90;
  }
}


inline void LCD_5110_add_unit_to_string(char *p_string, uint8_t i_dec_number)
{
  // Expect value 0~9, so just add ASCII value and return. So easy
  *(p_string) = '0' + i_dec_number;
}


