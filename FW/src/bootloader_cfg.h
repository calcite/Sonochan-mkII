/**
 * \file
 *
 * \brief Library for configuring bootloader at UC3A3256
 *
 * For now is supported only uc3a3256
 *
 * Created:  2015.04.03\n
 * Modified: 2015.04.03
 *
 * \version 0.1
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
 * Only for uc3a3256.
 */
#define BOOTLOADER_CFG_WORD_ADDR        0x808001FC

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
