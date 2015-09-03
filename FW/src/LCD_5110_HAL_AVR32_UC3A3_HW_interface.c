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
 * \brief Hardware Abstraction Layer for AVR32 and LCD used in nokia 5110\n
 * Use hardware SPI module on AVR.
 *
 * Contains basic low level functions which depend on user architecture and\n
 * hardware. Thanks to this layer is possible use common higher level driver\n
 * and apply it thru many architectures and devices. Also this allow easy\n
 * updates independent to higher layer (new features and so on).
 *
 * Created:  05.05.2014\n
 * Modified: 05.05.2014
 *
 * \version 0.1
 * \author Martin Stejskal
 */

#include "LCD_5110_HAL_AVR32_UC3A3_HW_interface.h"


//=================| Definition which user should not change |=================
/**
 * \brief Number of bits witch will be send by SPI at transmit
 *
 * This variable user should not change
 */
#define LCD_5110_SPI_BITS_PER_TRANSFER     8

//==================| Function prototypes not seen by user |===================
/**
  * \brief Initialize SPI HW, set GPIO and set (enable) clock source
  */
e_lcd_5110_status LCD_5110_HAL_init_SPI(void);

/**
 * \brief Stay in loop until sending data from SPI
 *
 * This is blocking function.
 */
e_lcd_5110_status LCD_5110_HAL_wait_for_SPI_tx(void);


//================================| Functions |================================

e_lcd_5110_status LCD_5110_HAL_init(void)
{
  // Pointer to SPI memory thru SRAM - because of FDIV flag
  volatile avr32_spi_t *p_spi;
  p_spi = LCD_5110_SPI_DEVICE;

  // Pointer to GPIO
  volatile avr32_gpio_port_t *gpio_port;

  // Variable for storing status from functions - default error
  e_lcd_5110_status e_status = LCD_5110_ERROR;

  // If reset support is enabled, just set low level
#if LCD_5110_RESET_support == 1
  // Set RST pin for LCD_5110 LCD to LOW as soon as possible -> restart LCD
  LCD_5110_IO_LOW(LCD_5110_RST_PIN);
#else
  // When reset support is disabled -> wait some time in loop
  volatile uint32_t i;
  // Delay routine
  for(i=0 ; i<LCD_5110_RESET_NUM_OF_CYCLES ; i++);
#endif


  // Initialize SPI device
  e_status = LCD_5110_HAL_init_SPI();

  // Check e_status - if not OK -> return error
  if(e_status != LCD_5110_OK)
  {
    return e_status;
  }


#if LCD_5110_RESET_support == 1
  // Then, set RST pin for LCD_5110 to HIGH -> LCD will be ready to communicate
  LCD_5110_IO_HIGH(LCD_5110_RST_PIN);
#endif

  // Set pin as output with low level - define DATA/!CMD for nokia LCD (DC)
  LCD_5110_IO_LOW(LCD_5110_SPI_DC_PIN);

  /* CS enable (inside SPI module - it not guarantee that CS will be enable on
   * GPIO!
   */
  p_spi->MR.pcs = ~(1<<LCD_5110_SPI_CHIP_SELECT);

  // Initialize sequence - DATA/!CMD is already set to 0
  //==========================| Initialize_sequence |==========================
  // last: 0x0C - normal mode LCD, 0x08 - blank LCD
  uint8_t data[]={0x21, 0xC0, 0x06, 0x13, 0x20, 0x0C};
  uint8_t i;
  for (i=0 ; i<6 ; i++)
  {
    // Wait until SPI TX is ready and then send data Byte
    e_status = LCD_5110_HAL_wait_for_SPI_tx();
    if(e_status != LCD_5110_OK)
    {
      return e_status;
    }
    // Else all OK -> send data
    p_spi->TDR.td = data[i];
  }

  // Wait for complete TX data (because of change DC signal)
  e_status = LCD_5110_HAL_wait_for_SPI_tx();

  // de-select device
  p_spi->MR.pcs = 0x0F;
  p_spi->CR.lastxfer = 1;

  // Check status
  if(e_status != LCD_5110_OK)
  {
    return e_status;
  }


  // DATA

  // DC high -> now  can be written just data
  LCD_5110_IO_HIGH(LCD_5110_SPI_DC_PIN);

#if LCD_5110_LIGHT_support == 1
  // If backlight support is enabled configure pin
  LCD_5110_IO_HIGH(LCD_5110_LIGHT_PIN);
#endif

  return LCD_5110_OK;
}





inline e_lcd_5110_status LCD_5110_HAL_init_SPI(void)
{
  // For backup IRQ flags
  uint32_t flags;

  // Variable as mask
  uint32_t   mask;

  // Pointer to SPI memory thru SRAM - because of FDIV flag
  volatile avr32_spi_t *p_spi;
  p_spi = LCD_5110_SPI_DEVICE;

  //=================================| GPIO |==================================

  // Set pins for SPI0 device
  static const gpio_map_t LCD_5110_SPI_GPIO_MAP = {
#if LCD_5110_CS_support == 1
      {LCD_5110_SPI_CS_PIN,      LCD_5110_SPI_CS_FUNCTION},
#endif
      {LCD_5110_SPI_SCK_PIN,     LCD_5110_SPI_SCK_FUNCTION},
      {LCD_5110_SPI_MOSI_PIN,    LCD_5110_SPI_MOSI_FUNCTION}
  };

  // Assign I/O to SPI
  if( gpio_enable_module(
                LCD_5110_SPI_GPIO_MAP,
                sizeof(LCD_5110_SPI_GPIO_MAP) / sizeof(LCD_5110_SPI_GPIO_MAP[0]))
      != GPIO_SUCCESS)
  {
    return LCD_5110_ERROR;
  }

  //==================================| PM |===================================

  // Enable clock do device (in default should be on, but just for case)

  // Get and clear global interrupt
  flags = __builtin_mfsr(AVR32_SR);
  // Disable IRQ
  __builtin_ssrf(AVR32_SR_GM_OFFSET);
  asm volatile("" ::: "memory");


  /*
   * Poll MSKRDY before changing mask rather than after, as it's
   * highly unlikely to actually be cleared at this point.
   */
  while (!(AVR32_PM.poscsr & (1U << AVR32_PM_POSCSR_MSKRDY))) {
          /* Do nothing */
  }

  // Enable the clock to flash and PBA bridge
  mask = *(&AVR32_PM.cpumask + AVR32_PM_CLK_GRP_HSB);
  mask |= 1U << (AVR32_FLASHC_CLK_HSB % 32);
  mask |= 1U << (AVR32_HMATRIX_CLK_HSB_PBA_BRIDGE % 32);
  *(&AVR32_PM.cpumask + AVR32_PM_CLK_GRP_HSB) = mask;

  // Enable clock to SPI0 and GPIO in PBA
  mask = *(&AVR32_PM.cpumask + AVR32_PM_CLK_GRP_PBA);
  mask |= 1U << (AVR32_SPI0_CLK_PBA % 32);
  mask |= 1U << (AVR32_GPIO_CLK_PBA % 32);
  *(&AVR32_PM.cpumask + AVR32_PM_CLK_GRP_PBA) = mask;

  // Restore global interrupt flags
  asm volatile("" ::: "memory");
  __builtin_csrf(AVR32_SR_GM_OFFSET);
  asm volatile("" ::: "memory");

  //==================================| SPI |==================================

  // Reset to defined state
  p_spi->CR.swrst = 1;

  // Set master mode
  p_spi->MR.mstr = 1;

  // Disable modfault
  p_spi->MR.modfdis = 1;

  // Internally disconnect MISO and MOSI
  p_spi->MR.llb = 0;

  // Chip select will be fixed
  p_spi->MR.ps = 0;

  // Chip select are directly connected to a peripheral device
  p_spi->MR.pcsdec = 0;

  // Disable all chip selects (set them to high)
  p_spi->MR.pcs = 0x0F;

  // Set delay between two CS
  p_spi->MR.dlybcs = LCD_5110_SPI_DELAY_BETWEEN_CS;

  // Set delay CS enable -> time delay -> SCK at start

  // Set delay SCK -> time delay -> CS disable

  // Bits per one transfer (when value is 0 -> send 8 bits -> -8)

  /* Set polarity of SCK and define when data should be transmitted. This
   * depends on which chip select is used. So there is preprocessor routine
   * to optimize code. CPOL = 0 (SCK in low when idle). NCPHA = 1 (data
   * change at falling edge)
   */

  /* CS stay in low even after data transmission is complete. Wait until new
   * transfer is requested on a different chip select or until CS is disabled
   * externally
   */

  // Set baudrate register -> "speed of SPI"
#if LCD_5110_SPI_CHIP_SELECT == 0
  p_spi->CSR0.dlybs = LCD_5110_SPI_DELAY_SCK_START;
  p_spi->CSR0.dlybct = LCD_5110_SPI_DELAY_CS_STOP;
  p_spi->CSR0.bits = LCD_5110_SPI_BITS_PER_TRANSFER -8;
  p_spi->CSR0.cpol  = 0;
  p_spi->CSR0.ncpha = 1;
  p_spi->CSR0.csaat = 1;
  p_spi->CSR0.scbr = LCD_5110_SPI_SCBR;
#elif LCD_5110_SPI_CHIP_SELECT == 1
  p_spi->CSR1.dlybs = LCD_5110_SPI_DELAY_SCK_START;
  p_spi->CSR1.dlybct = LCD_5110_SPI_DELAY_CS_STOP;
  p_spi->CSR1.bits = LCD_5110_SPI_BITS_PER_TRANSFER -8;
  p_spi->CSR1.cpol  = 0;
  p_spi->CSR1.ncpha = 1;
  p_spi->CSR1.csaat = 1;
  p_spi->CSR1.scbr = LCD_5110_SPI_SCBR;
#elif LCD_5110_SPI_CHIP_SELECT == 2
  p_spi->CSR2.dlybs = LCD_5110_SPI_DELAY_SCK_START;
  p_spi->CSR2.dlybct = LCD_5110_SPI_DELAY_CS_STOP;
  p_spi->CSR2.bits = LCD_5110_SPI_BITS_PER_TRANSFER -8;
  p_spi->CSR2.cpol  = 0;
  p_spi->CSR2.ncpha = 1;
  p_spi->CSR2.csaat = 1;
  p_spi->CSR2.scbr = LCD_5110_SPI_SCBR;
#elif LCD_5110_SPI_CHIP_SELECT == 3
  p_spi->CSR3.dlybs = LCD_5110_SPI_DELAY_SCK_START;
  p_spi->CSR3.dlybct = LCD_5110_SPI_DELAY_CS_STOP;
  p_spi->CSR3.bits = LCD_5110_SPI_BITS_PER_TRANSFER -8;
  p_spi->CSR3.cpol  = 0;
  p_spi->CSR3.ncpha = 1;
  p_spi->CSR3.csaat = 1;
  p_spi->CSR3.scbr = LCD_5110_SPI_SCBR;
#else
#error "LCD_5110_SPI_CHIP_SELECT can be 0, 1, 2 or 3. Please set correct value"
#endif

  // Enable whole SPI module
  p_spi->CR.spien = 1;

  return LCD_5110_OK;
}



inline e_lcd_5110_status LCD_5110_HAL_wait_for_SPI_tx(void)
{
  // Pointer to SPI memory thru SRAM
  volatile avr32_spi_t *p_spi;
  p_spi = LCD_5110_SPI_DEVICE;

  // Counter
  volatile uint32_t i_cnt = 0;

  // Wait until all data send including delays
  while (!(p_spi->SR.txempty))
  {
    // Increase counter and check if timeout occurs
    if(i_cnt++ > LCD_5110_SPI_TIMEOUT)
    {
      return LCD_5110_ERROR_TIMEOUT;
    }
  }

  return LCD_5110_OK;
}





e_lcd_5110_status LCD_5110_HAL_send_data_byte(uint8_t i_data)

{
  // Pointer to SPI memory thru SRAM
  volatile avr32_spi_t *p_spi;
  p_spi = LCD_5110_SPI_DEVICE;


  // Wait for complete TX data
  if( LCD_5110_HAL_wait_for_SPI_tx() != LCD_5110_OK)
  {
    return LCD_5110_ERROR_TIMEOUT;
  }

  // Enable CS and check if OK
  p_spi->MR.pcs = ~(1<<LCD_5110_SPI_CHIP_SELECT);

  p_spi->TDR.td = i_data;

  // And disable chip select on nokia 5510 LCD
  p_spi->MR.pcs = 0x0F;
  p_spi->CR.lastxfer = 1;

  // If all OK return "all OK" status
  return LCD_5110_OK;
}



e_lcd_5110_status LCD_5110_HAL_send_command_byte(uint8_t i_cmd)
{
  // Pointer to SPI memory thru SRAM
  volatile avr32_spi_t *p_spi;
  p_spi = LCD_5110_SPI_DEVICE;

  // Pointer to GPIO
  volatile avr32_gpio_port_t *gpio_port;

  // Wait for complete TX data
  if( LCD_5110_HAL_wait_for_SPI_tx() != LCD_5110_OK)
  {
    return LCD_5110_ERROR_TIMEOUT;
  }
  // Chip select enable
  p_spi->MR.pcs = ~(1<<LCD_5110_SPI_CHIP_SELECT);

  // DC low -> now will be written command
  LCD_5110_IO_LOW(LCD_5110_SPI_DC_PIN);

  // Try send data
  // Send data to shift register
  p_spi->TDR.td = i_cmd;

  // Wait for complete TX data - because of change DC signal
  if( LCD_5110_HAL_wait_for_SPI_tx() != LCD_5110_OK)
  {
    return LCD_5110_ERROR_TIMEOUT;
  }

  // DC high -> now can be written just data
  LCD_5110_IO_HIGH(LCD_5110_SPI_DC_PIN);

  // And disable chip select on nokia 5510 LCD
  p_spi->MR.pcs = 0x0F;
  p_spi->CR.lastxfer = 1;

  // If all OK -> return OK
  return LCD_5110_OK;
}



inline e_lcd_5110_status LCD_5110_HAL_backlight_on(void)
{
#if LCD_5110_LIGHT_support == 1
  // Pointer to GPIO
  volatile avr32_gpio_port_t *gpio_port;

  // If support is enabled, then set output
  LCD_5110_IO_HIGH(LCD_5110_LIGHT_PIN);
  // And return success
  return LCD_5110_OK;
#else
  // When support is disabled just return error (can not set)
  return LCD_5110_ERROR;
#endif
}



inline e_lcd_5110_status LCD_5110_HAL_backlight_off(void)
{
#if LCD_5110_LIGHT_support == 1
  // Pointer to GPIO
  volatile avr32_gpio_port_t *gpio_port;

  // If support is enabled, then set output
  LCD_5110_IO_LOW(LCD_5110_LIGHT_PIN);
  // And return success
  return LCD_5110_OK;
#else
  // When support is disabled just return error (can not set)
  return LCD_5110_ERROR;
#endif
}


