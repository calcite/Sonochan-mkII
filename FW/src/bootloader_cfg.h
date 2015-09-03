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
 * \brief Library for configuring bootloader at UC3A3256
 *
 * For now is supported only uc3a3256
 *
 * Created:  2015.04.03\n
 * Modified: 2015.08.25
 *
 * \version 0.2
 * \author Martin Stejskal
 */

#ifndef _BOOTLOADER_CFG_H_
#define _BOOTLOADER_CFG_H_

//================================| Includes |=================================
// Define uint8_t and so on
#include <inttypes.h>
// Work with flash memory
#include <flashc.h>
//===============================| Definitions |===============================
/**
 * \breif Define address where is configuration word
 *
 * Bootloader configuration word is at the end of user page memory. So we\n
 * need to get address of last word (4B)
 */
#define BOOTLOADER_CFG_WORD_ADDR        \
  (AVR32_FLASHC_USER_PAGE_ADDRESS + AVR32_FLASHC_PAGE_SIZE -4)

//===============================| Structures |================================
/**
 * \brief Bootloader setting in user flash
 *
 * This structure describe configuration word for bootloader
 */
typedef struct bootloader_cfg_t{
  union
  {
    uint32_t cfg_u32;
    struct{
      // UC3A is Big endian -> MSB is in structure at top

      /* Boot key. Always 0x494F. This key is used to identify the word as
       * meaningful for the bootloader.
       */
      uint32_t isp_boot_key          :15;

      /* Active level of ISP_IO_COND_PIN that the bootloader will consider as
       * a request for starting. 0 is for low level, 1 is for high level.
       */
      uint32_t isp_io_cond_level     :1;

      /* The GPIO pin number for testing boot condition. This value is defined
       * in datasheet. For example: if you want use pin PB10 (which is default)
       * you should use number 42. For pin PX11 use 62 and so on. This is chip
       * dependent, so please, please refer to datasheet.
       */
      uint32_t isp_io_cond_pin       :8;

      /* CRC8 of configuration. It is calculated as:
       * P(X)=X^8 + X^2 + X + 1
       * This value is used to check validity of configuration word.
       * That's all. In documentation is not mentioned more... Sorry.
       */
      uint32_t isp_crc8              :8;
    }s;
  };

}bootloader_cfg_t;
//================================| Functions |================================
void set_bootloader_pin(uint8_t gpio_pin_number, uint8_t i_active_level);

#endif
