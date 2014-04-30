/**
 * \file
 *
 * \brief Hardware Abstraction Layer for PLL CS2200 running on AVR32 UC3A3\n
 * architecture
 *
 * Contains basic low level functions which depend on user architecture and\n
 * hardware. Thanks to this layer is possible use common higher level driver\n
 * and apply it thru many architectures and devices. Also this allow easy\n
 * updates independent to higher layer (new features and so on).
 *
 * Created:  10.03.2014\n
 * Modified: 12.03.2014
 *
 * \version 0.1
 * \author Martin Stejskal
 */

#include "cs2200_HAL_AVR32_UC3A3_HW_interface.h"

//=================| Definition which user should not change |=================
/**
 * \brief Define when timeout occurs
 *
 * When waiting for send rest of TX data this constant define how long should\n
 * function should wait. Value should be calculated for most applications.\n
 * However user can set it explicitly here, but it is not recommended.
 */
#define CS2200_TWI_TIMEOUT        \
  (250*(CS2200_TWI_CLK_SOURCE/CS2200_TWI_BAUDRATE))


//================================| Functions |================================
/**
 * \brief Initialize TWI module on chip
 *
 * Set TWI module as master. Set I/O, enable clock to module and so on
 * @return CS2200_OK (0) if all OK
 */
cs2200_status_t cs2200_HAL_init(void)
{

  // For backup IRQ flags
  uint32_t flags;

  // Variable as mask
  uint32_t   mask;

  // Pointer to TWI address
  volatile avr32_twim_t *p_twi;
  p_twi = CS2200_TWI_DEVICE;


  //=================================| GPIO |==================================

  // Set pins for SPI0 device
  static const gpio_map_t CS2200_GPIO_MAP = {
      {CS2200_TWI_SDA_PIN,    CS2200_TWI_SDA_FUNCTION},
      {CS2200_TWI_SCL_PIN,    CS2200_TWI_SCL_FUNCTION}
  };

  // Assign I/O to SPI
  if( gpio_enable_module(
      CS2200_GPIO_MAP,
                sizeof(CS2200_GPIO_MAP) / sizeof(CS2200_GPIO_MAP[0]))
      != CS2200_OK)
  {
    return CS2200_ERROR_TWI_INVALID_PARAMETER;
  }

  //==================================| PM |===================================
  // Enable clock to device (in default should be on, but just for case)

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

  // Enable clock to TWI and GPIO in PBA
  mask = *(&AVR32_PM.cpumask + AVR32_PM_CLK_GRP_PBA);
  mask |= 1U << (AVR32_TWIM0_CLK_PBA % 32);
  mask |= 1U << (AVR32_GPIO_CLK_PBA % 32);
  *(&AVR32_PM.cpumask + AVR32_PM_CLK_GRP_PBA) = mask;

  // Restore global interrupt flags
  asm volatile("" ::: "memory");
  __builtin_csrf(AVR32_SR_GM_OFFSET);
  asm volatile("" ::: "memory");

  //==================================| TWI |==================================
  // Configure TWI module

  // Control register

  // Reset TWI module
  p_twi->CR.swrst = 1;

  // Disable all interrupts
  p_twi->ier = 0;
  p_twi->idr = ~0;

  // Disable SMBus
  p_twi->CR.smdis = 1;
  p_twi->CR.smen  = 0;

  // Set master mode
  p_twi->CR.mdis = 0;
  p_twi->CR.men  = 1;

  // Clock waveform generator register (set baudrate/speed)
  // TWI prescaller
  p_twi->CWGR.exp    = CS2200_TWI_CKDIV;
  // Time space in start and stop condition. At least should be 1
  p_twi->CWGR.stasto = CS2200_TWI_CLDIV;
  // Extra time on every data bit.
  p_twi->CWGR.data   = 0;
  // Time for high level
  p_twi->CWGR.high   = CS2200_TWI_CLDIV;
  // Time for low level
  p_twi->CWGR.low    = CS2200_TWI_CLDIV;

  // When in master read mode, then last byte should not be ACKed
  p_twi->CMDR.acklast = 0;
  // 7 bit addressing space
  p_twi->CMDR.tenbit = 0;

  // OK, all should be ready

  return CS2200_OK;
}





/**
 * \brief Write data on TWI (I2C) bus
 *
 * MCU is in master mode. Send CS2200 address and then data (argument).
 *
 * @param p_data Pointer to data array which will be send to PLL thru TWI (I2C)
 *
 * @param i_number_of_bytes Number of data bytes, which will be send
 *
 * @return CS2200_OK (0) if all OK
 */
cs2200_status_t cs2200_HAL_write_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes)
{
  // Pointer to TWI address
  volatile avr32_twim_t *p_twi;
  p_twi = CS2200_TWI_DEVICE;

  // Counter for timeout
  volatile uint32_t i_cnt_timeout = 0;

  // Clear status register
  p_twi->scr = ~0;


  // Wait until device is idle
  while(!p_twi->SR.idle)
  {
    // Increase timeout counter
    i_cnt_timeout++;
    // Test timeout counter
    if(i_cnt_timeout > CS2200_TWI_TIMEOUT)
    {
      // If higher -> return error value
      return CS2200_ERROR_TWI_TIMEOUT;
    }
  }

  // Reset counter - just for case
  i_cnt_timeout = 0;



  // Set slave address - must calculate with option that TWI use another driver
  p_twi->CMDR.sadr = CS2200_TWI_ADDRESS;

  // Set number of bytes to send
  p_twi->CMDR.nbytes = i_number_of_bytes;

  p_twi->CMDR.start = 1;
  p_twi->CMDR.stop = 1;

  // Want transmit data
  p_twi->CMDR.read = 0;

  // Data in CMDR are valid
  p_twi->CMDR.valid = 1;

  while(i_number_of_bytes != 0)
  {
    // Send byte
    p_twi->THR.txdata = *(p_data++);

    // Wait until read TX data - during this, check for timeout and NACK
    while(!p_twi->SR.txrdy)
    {
      // Again, check for timeout
      // Increase timeout counter
      i_cnt_timeout++;
      // Test timeout counter
      if(i_cnt_timeout > CS2200_TWI_TIMEOUT)
      {
        // If higher -> return error value
        return CS2200_ERROR_TWI_TIMEOUT;
      }
      // Check for NACK
      if(p_twi->SR.anak || p_twi->SR.dnak)
      {
        return CS2200_ERROR_TWI_NACK;
      }
    }

    // Decrement number of bytes which we need to send
    i_number_of_bytes--;

    // Clear counter (byte was successfully send)
    i_cnt_timeout = 0;
  }

  // When all data send and no NACK received
  return CS2200_OK;
}




/**
 * \brief Read data on TWI (I2C) bus
 *
 * MCU is in master mode. Send CS2200 address and then receive data.
 *
 * @param p_data Data are saved thru this pointer
 * @param i_number_of_bytes Number of data bytes, which will be received
 * @return CS2200_OK (0) if all OK
 */
cs2200_status_t cs2200_HAL_read_data(
    uint8_t *p_data,
    uint8_t i_number_of_bytes)
{
  uint8_t *pdb;
  pdb = p_data;

  // Pointer to TWI address
  volatile avr32_twim_t *p_twi;
  p_twi = CS2200_TWI_DEVICE;

  // Counter for timeout
  volatile uint32_t i_cnt_timeout = 0;

  // Clear status register
  p_twi->scr = ~0;

  // TWI should be idle. Let's wait a while if needed
  while(!p_twi->SR.idle)
  {
    // Increase timeout counter
    i_cnt_timeout++;
    // Test timeout counter
    if(i_cnt_timeout > CS2200_TWI_TIMEOUT)
    {
      // If higher -> return error value
      return CS2200_ERROR_TWI_TIMEOUT;
    }
  }

  // Reset counter - just for case
  i_cnt_timeout = 0;

  p_twi->CMDR.sadr = CS2200_TWI_ADDRESS;

  p_twi->CMDR.nbytes = i_number_of_bytes;

  p_twi->CMDR.start = 1;
  p_twi->CMDR.stop = 1;

  // Want receive data
  p_twi->CMDR.read = 1;

  // Data in CMDR are valid
  p_twi->CMDR.valid = 1;

  while(i_number_of_bytes != 0)
  {
    // Wait until TX is done - during this check for timeout and NACK
    while(!p_twi->SR.rxrdy)
    {
      // Again, check for timeout
      // Increase timeout counter
      i_cnt_timeout++;
      // Test timeout counter
      if(i_cnt_timeout > CS2200_TWI_TIMEOUT)
      {
        // If higher -> return error value
        return CS2200_ERROR_TWI_TIMEOUT;
      }
      // Check for NACK (address is ACKed by CS2200. Data are ACKed by AVR)
      if(p_twi->SR.anak)
      {
        return CS2200_ERROR_TWI_NACK;
      }
    }
    // If byte read, then add copy it!
    *(p_data++) = p_twi->RHR.rxdata;

    // Decrement number of bytes which we need to send
    i_number_of_bytes--;

    // Clear counter (byte was successfully received)
    i_cnt_timeout = 0;
  }

  return CS2200_OK;
}
