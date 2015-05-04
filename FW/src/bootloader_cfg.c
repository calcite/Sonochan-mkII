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
#include "bootloader_cfg.h"

//=========================| Functions not for user |==========================
/**
 * \brief Calculate CRC8 for bootloader configuration word
 *
 * Inspirated by:
 * https://ghsi.de/CRC/index.php?Polynom=100000111&Message=929E2A
 *
 * @param i_data Data to process. Only high 24 bits are processed.
 * @return CRC8 value
 */
uint8_t calc_crc8_bootloader(uint32_t i_data)
{
  // Polynom is X^8 + X^2 + X + 1

  uint8_t i;
  uint8_t i_do_inv;

  // For saving CRC. Using union make it hell a lot of easier
  union crc_res{
    uint8_t crc_u8;
    struct s{
      // Big Endian struct
      uint8_t b7        :1;
      uint8_t b6        :1;
      uint8_t b5        :1;
      uint8_t b4        :1;
      uint8_t b3        :1;
      uint8_t b2        :1;
      uint8_t b1        :1;
      uint8_t b0        :1;
    }s;
  }crc;

  // Initial value
  crc.crc_u8 = 0;

  // Process bit by bit (only high 24 bits)
  for(i=31 ; i>7 ; i--)
  {
    i_do_inv = (0 != (i_data & (1<<i)) ) ^ crc.s.b7;

    crc.s.b7 = crc.s.b6;
    crc.s.b6 = crc.s.b5;
    crc.s.b5 = crc.s.b4;
    crc.s.b4 = crc.s.b3;
    crc.s.b3 = crc.s.b2;
    crc.s.b2 = crc.s.b1 ^ i_do_inv;
    crc.s.b1 = crc.s.b0 ^ i_do_inv;
    crc.s.b0 = i_do_inv;
  }

  return crc.crc_u8;
}



//=============================| User functions |==============================
/**
 * \brief Set bootloader pin
 *
 * Bootloader can be activated only if predefined pin is in "right" level.\n
 * This function allow easily change this activation pin and level.
 *
 * @param gpio_pin_number GPIO number defined in datasheet. For UC3A3256
 *                        pin PB10 is number 42. Just refer to datasheet and
 *                        you will be probably fine.
 *
 * @param i_active_level 0 for low level, else active level is 1
 */
void set_bootloader_pin(uint8_t gpio_pin_number, uint8_t i_active_level)
{
  // Because we want to keep bootloader alive, it is good idea to keep correct
  // values in user page. So we fill pointer correct value
  bootloader_cfg_t cfg;

  // Just be sure, that value is correct
  if(i_active_level != 0)
  {
    i_active_level=1;
  }
  // Default factory value: 0x929E2A9E
  cfg.s.isp_io_cond_pin=gpio_pin_number;    // GPIO pin
  cfg.s.isp_io_cond_level=i_active_level;   // PIN level
  cfg.s.isp_crc8=0x00;                      // Will be calculated
  cfg.s.isp_boot_key=0x494F;                // Just constant. Have to be there

  cfg.s.isp_crc8=calc_crc8_bootloader(cfg.cfg_u32);

  // Create pointer to wanted address. This is chip dependent
  uint32_t *p_bootloader_seting=(uint32_t*)BOOTLOADER_CFG_WORD_ADDR;

  // Just check if value differs. If it same, it does not have meening
  // rewriting memory.
  if(*p_bootloader_seting != cfg.cfg_u32)
  {
    flashc_memcpy((void*)p_bootloader_seting,
                  (void*)(&cfg.cfg_u32),
                  4,
                  1);
  }
}

