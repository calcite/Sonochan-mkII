/**
 * \file
 *
 * \brief Generic driver for LCD used in nokia 5110
 *
 * Used for basic operations (init, clear, write text, ...) with LCD 5510.\n
 * It need HAL driver (which depend on used architecture) to work properly.\n
 * Please include correct HAL driver files.
 *
 * Created:  04.07.2012\n
 * Modified: 06.05.2014
 *
 * \version 1.1
 * \author Martin Stejskal
 */

#ifndef _LCD_5110_H_
#define _LCD_5110_H_


//===========================| Included libraries |============================
/* HAL for LCD module !!! Please include correct driver for used architecture!
 * Also in included driver should be defined enumeration e_lcd_5110_status
 */
#include "LCD_5110_HAL_AVR32_UC3A3_HW_interface.h"


/**
 * \brief Allow enable FreeRTOS features
 *
 * Options: 0 (disabled) or 1 (enabled). When enabled, then FreeRTOS features\n
 * are enabled.
 */
#define LCD_5110_SUPPORT_RTOS             1

/**
 * \brief Allow create mutex mutexSPI when LCD_5110_init is called.
 *
 * Mutexes is needed when in RTOS two functions share same hardware. This\n
 * option allow create mutex mutexSPI, so any other functions just can use\n
 * xSemaphoreTake and xSemaphoreGive functions.\n
 * Options: 0 (disabled) or 1 (enabled).\n
 *
 * \note This option is valid only if RTOS support is enabled.
 */
#define LCD_5110_RTOS_CREATE_MUTEX        1

//===========================| Additional includes |===========================
#if LCD_5110_SUPPORT_RTOS != 0
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

extern xSemaphoreHandle mutexSPI;
#endif
//=================================| Macros |==================================

/**
 * \brief Define lock function when support for RTOS is enabled
 *
 * If LCD_5110_SUPPORT_RTOS is enabled, then there is defined function, that\n
 * "lock" TWI device, so any other device can not use it.\n
 * If LCD_5110_SUPPORT_RTOS is disabled, there is defined just void macro.
 */
#if LCD_5110_SUPPORT_RTOS != 0
  #define LCD_5110_LOCK_SPI_MODULE_IF_RTOS    \
    xSemaphoreTake( mutexSPI, portMAX_DELAY );
#else
  #define LCD_5110_LOCK_SPI_MODULE_IF_RTOS
#endif


#if LCD_5110_SUPPORT_RTOS != 0
  #define LCD_5110_UNLOCK_SPI_MODULE_IF_RTOS        \
    xSemaphoreGive( mutexSPI );
#else
  #define LCD_5110_UNLOCK_SPI_MODULE_IF_RTOS
#endif

//===========================| Functions for user |============================

//=============================| Basic functions |=============================
/**
 * \brief Initialize LCD module and call clear screen function.
 *
 * Also initialize SPI device.
 *
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_init(void);

 /**
  * \brief Clear screen and set cursor to 0,0. This function is called by\n
  * "LCD_5110_init"
  *
  * Whole LCD is rewritten by 0x00. It is called by LCD_5110_init()
  *
  * @return LCD_5110_OK (0) if all OK
  */
 e_lcd_5110_status LCD_5110_clear(void);


/**
 * \brief Write text to LCD.
 *
 * Text is given as pointer to char. Write characters until
 * is found NULL. If message is not fit on one line, then message continues on
 * the next line
 * \param p_txt_to_show Pointer to string that ends NULL character
 *
 * Example of using "LCD_5110_write":
 *
 *  i_status = some_function();\n
 *  \n
 *  // Needed include lib. "stdio.h"\n
 *  sprintf(&text[0], "Codec set status: %d", i_status);\n
 *  LCD_5110_write( &text[0] );\n
 *
 * Or:
 *
 *  LCD_5110_write("Text to show");
 *
 *
 *  @return LCD_5110_OK (0) if all OK
 */
 e_lcd_5110_status LCD_5110_write(const char * p_txt_to_show);

/**
 * \brief Write text to LCD at specified coordinates.
 *
 * Text must end with NULL
 * character. If message is not fit on one line, then message continues on the
 * next line.
 * \param p_txt_to_show Pointer to string that ends NULL character
 * \param x_position  X coordinate. Accepted value 0~83 else is set to 0
 * \param y_line Line index on LCD. Accepted value 0~5 else is set
 * to 0
 *
 * Example of using "LCD_5110_write_xy":
 *
 *  i_status = some_function();\n
 *  \n
 *  // Needed include lib. "stdio.h"\n
 *  sprintf(&text[0], "Codec set status: %d", i_status);\n
 *  LCD_5110_write_xy( &text[0], 3, 1 );\n
 *
 * Or:
 *
 *  LCD_5110_write_xy("Text to show", 10, 0);
 *
 *  @return LCD_5110_OK (0) if all OK
 */
 e_lcd_5110_status LCD_5110_write_xy(const char * p_txt_to_show,
                                     int x_position, int y_line);

/**
 * \brief Only set X coordinate on LCD nokia 5510
 *
 * \param X_address X coordinate. Accepted value 0~83 else is set to 0
 *
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_set_X(uint8_t X_address);

/**
 * \brief Only set active line on LCD nokia 5510
 *
 * \param y_line Line index on LCD. Accepted value 0~5 else is set
 *
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_set_line(uint8_t y_line);


/**
 * \brief Symbolic name for function in HAL driver to turn on backlight
 *
 * For more info please refer in selected LCD_5110_HAL_* driver\n
 * Example:\n
 *  // Try turn on backlight if support is enabled. Return 0 if all OK\n
 *  e_status = LCD_5110_backlight_on();
 */
#define LCD_5110_backlight_on           LCD_5110_HAL_backlight_on

/**
 * \brief Symbolic name for function in HAL driver to turn on backlight
 *
 * For more info please refer in selected LCD_5110_HAL_* driver\n
 * Example:\n
 *  // Try turn off backlight if support is enabled. Return 0 if all OK\n
 *  e_status = LCD_5110_backlight_off();
 */
#define LCD_5110_backlight_off          LCD_5110_HAL_backlight_off

//============================| Runtime settings |=============================
/**
 * \brief Allow to set/clear flag which enable/disable auto add new line
 *
 * When is called LCD_5110_write*() functions, then by default after every\n
 * write is new message added to new line. Sometimes it can be this annoying\n
 * so this function allow enable (1) or disable (0) this feature.
 *
 * @param i_auto_LF_flag Enable (1) or disable (0) auto add new line
 */
void LCD_5110_auto_newline(uint8_t i_auto_LF_flag);

//============================| Special functions |============================
/**
 * \brief Write given number as 8bit hexadecimal number
 *
 * Add prefix "0x" before written number
 *
 * @param i_hex8 8bit binary value
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_write_hex8(uint8_t i_hex8);


/**
 * \brief Write given number as 16bit hexadecimal number
 *
 * Add prefix "0x" before written number
 *
 * @param i_hex16 16bit binary value
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_write_hex16(uint16_t i_hex16);

/**
 * \brief Write given number as 32bit hexadecimal number
 *
 * Add prefix "0x" before written number
 *
 * @param i_hex32 32bit binary value
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_write_hex32(uint32_t i_hex32);


/**
 * \brief Write given 8bit number as decimal value without sign
 *
 * Value on LCD: 0~255
 *
 * @param i_dec8 8bit unsigned value
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_write_dec8_unsigned(uint8_t i_dec8);


/**
 * \brief Write given 8bit number as decimal value with sign
 *
 * Value on LCD: -128~127
 *
 * @param i_dec8 8bit signed value
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_write_dec8_signed(int8_t i_dec8);


/**
 * \brief Write given 16bit number as decimal value without sign
 *
 * Value on LCD: 0~65535
 *
 * @param i_dec16 16bit unsigned value
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_write_dec16_unsigned(uint16_t i_dec16);


/**
 * \brief Write given 16bit number as decimal value with sign
 *
 * Value on LCD: -32768~32767
 *
 * @param i_dec16 16bit signed value
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_write_dec16_signed(int16_t i_dec16);

//===========================| Low level functions |===========================

/**
 * \brief Send one data Byte to LCD module
 *
 * Also lock and unlock device when RTOS support is enabled
 *
 * @param i_Byte Data byte which will be send to LCD module
 *
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_send_data_byte(uint8_t i_data);

/**
 * \brief Send one command Byte to LCD module
 *
 * Also lock and unlock device when RTOS support is enabled
 *
 * @param i_cmd Command which will be send to LCD module
 * @return LCD_5110_OK (0) if all OK
 */
e_lcd_5110_status LCD_5110_send_command_byte(uint8_t i_cmd);
#endif
