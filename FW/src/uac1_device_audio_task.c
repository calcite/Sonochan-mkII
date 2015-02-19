/* -*- mode: c++; tab-width: 4; c-basic-offset: 4 -*- */
/* This source file is part of the ATMEL AVR32-SoftwareFramework-AT32UC3-1.5.0 Release */

/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief Management of the USB device Audio task.
 *
 * This file manages the USB device Audio task.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a USB module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ***************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * Modified by Alex Lee and SDR-Widget team for the sdr-widget project.
 * See http://code.google.com/p/sdr-widget/
 * Copyright under GNU General Public License v2
 */

//_____  I N C L U D E S ___________________________________________________

#include <stdio.h>
#include "usart.h"     // Shall be included before FreeRTOS header files, since 'inline' is defined to ''; leading to
                       // link errors
#include "conf_usb.h"


#if USB_DEVICE_FEATURE == ENABLED

#include "board.h"

#ifdef FREERTOS_USED
#include "FreeRTOS.h"
#include "task.h"
#endif
#include "gpio.h"
#include "pdca.h"
#include "usb_drv.h"
#include "usb_descriptors.h"
#include "uac1_usb_descriptors.h"
#include "usb_standard_request.h"
#include "usb_specific_request.h"
#include "uac1_usb_specific_request.h"
#include "device_audio_task.h"
#include "uac1_device_audio_task.h"


#include "composite_widget.h"
#include "taskAK5394A.h"

//[Martin]
#include "ssc.h"
#include "sync_control.h"

#include "print_funcs.h"
#include <stdio.h>


//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N S ______________________________________________


#define FB_RATE_DELTA (1<<12)
#define FB_RATE_DELTA_NUM 2

//[Martin]
/* For testing we can set high value. Tested usually with value 50000, but
 * some audiophiles can change it if are 100% sure, that OS support nicely
 * feedback EP and they are sure that do not need auto tune function
 */
#define MAX_PLL_PPM     50000
/* Define minimum "time delay" between two frequency changes
 * (INC, DEC functions). Experimentally was set value 250, but it can be
 * changed by time
 *
 */
#define PLL_CHANGE_TIME_DIFF    250

//_____ D E C L A R A T I O N S ____________________________________________


//? why are these defined as statics?
//[Martin] Cause they are only for this file

/* Enable / disable auto tune PLL. Usually in case, that audio feedback EP
 * seems that not work, so last option is tune PLL by small steps. However
 * not every time it is best option. Under Linux seems that works good, but
 * under Win7 buffer can overflow/underflow.
 */
static uint8_t i_auto_tune_enable = 1;
// Allow switch TX data channels (L+R -> R+L)
static uint8_t i_swap_LR_TX = 0;
// Allow switch RX data channels (L+R -> R+L)
static uint8_t i_swap_LR_RX = 0;



static U32  index_i;
static U32 spk_index;
//static U16  old_gap = SPK_BUFFER_SIZE;
static U8 audio_buffer_out, spk_buffer_in;  // the ID number of the buffer used for sending out to the USB
static volatile U32 *audio_buffer_ptr;
//static volatile U32 *spk_buffer_ptr;

static U8 ep_audio_in, ep_audio_out, ep_audio_out_fb;

//!
//! @brief This function initializes the hardware/software resources
//! required for device Audio task.
//!
void uac1_device_audio_task_init(U8 ep_in, U8 ep_out, U8 ep_out_fb)
{
  index_i     =0;
  audio_buffer_out = 0;
  audio_buffer_ptr = audio_buffer_0;
  spk_index = 0;
  spk_buffer_in = 0;
//  spk_buffer_ptr = spk_buffer_0;
  mute = FALSE;
  spk_mute = FALSE;
  volume = 0x5000;
  spk_volume = 0x5000;
  ep_audio_in = ep_in;
  ep_audio_out = ep_out;
  ep_audio_out_fb = ep_out_fb;

  xTaskCreate(uac1_device_audio_task,
        configTSK_USB_DAUDIO_NAME,
        configTSK_USB_DAUDIO_STACK_SIZE,
        NULL,
        configTSK_USB_DAUDIO_PRIORITY,
        NULL);
}

//!
//! @brief Entry point of the device Audio task management
//!

void uac1_device_audio_task(void *pvParameters)
{
  Bool playerStarted = FALSE;
  static U32  time_startup=0;
  static Bool startup=TRUE;
  int i;
  int delta_num = 0;
  U16 num_samples, num_remaining, gap;
  U8 sample_HSB;
  U8 sample_MSB;
  U8 sample_SB;
  U8 sample_LSB;
  U32 sample;
  const U8 EP_AUDIO_IN = ep_audio_in;
  const U8 EP_AUDIO_OUT = ep_audio_out;
  const U8 EP_AUDIO_OUT_FB = ep_audio_out_fb;
  const U8 IN_LEFT = 0;
  const U8 IN_RIGHT = 1;
  const U8 OUT_LEFT = 0;
  const U8 OUT_RIGHT = 1;
  volatile avr32_pdca_channel_t *pdca_channel = pdca_get_handler(PDCA_CHANNEL_SSC_RX);
  volatile avr32_pdca_channel_t *spk_pdca_channel = pdca_get_handler(PDCA_CHANNEL_SSC_TX);

  //[Martin]
  // Counts how many times was PLL frequency changed
  int i_pll_diff = 0;
  // Store number of samples and if not same reset i_pll_diff
  uint16_t old_num_samples = 0;
  // Measure time. Useful for changing PLL frequency
  uint32_t i_last_time = 0;
  // Time when not in start up
  uint32_t time = 0;

  //[Martin] Keep wanted sample length
  uint8_t i_sample_length;
  brd_drv_get_data_length(&i_sample_length);


  // Just 4 debug - if needed, then uncomment
  //char c[20];


  switch(current_freq.frequency)
  {
  case 48000:
    FB_rate = 48 << 14;
    break;
  case 44100:
    FB_rate = (44 << 14) + (1 << 14)/10;
    break;
  case 32000:
    FB_rate = (32 << 14);
    break;
  case 24000:
    FB_rate = (24 << 14);
    break;
  case 22050:
    FB_rate = (22 << 14) + (50 << 14)/100;
    break;
  case 16000:
    FB_rate = (16 << 14);
    break;
  case 11025:
    FB_rate = (11 << 14) + (25 << 14)/100;
    break;
  case 8000:
    FB_rate = (8 << 14);
    break;
  default:
    // Case that something happens and frequency is wrong -> set default
    FB_rate = 48 << 14;
  }

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (TRUE) {
    vTaskDelayUntil(&xLastWakeTime, UAC1_configTSK_USB_DAUDIO_PERIOD);

    // First, check the device enumeration state
    if (!Is_device_enumerated()) { time_startup=0; startup=TRUE; continue; };

    if( startup ) {


      time_startup+=UAC1_configTSK_USB_DAUDIO_PERIOD;
#define STARTUP_LED_DELAY  10000
      ///\todo [Martin] Try to write new LED driver and "connect" it to board driver
      if ( time_startup <= 1*STARTUP_LED_DELAY ) {
        LED_On( LED0 );
        pdca_disable_interrupt_reload_counter_zero(PDCA_CHANNEL_SSC_RX);
        pdca_disable(PDCA_CHANNEL_SSC_RX);
        //              LED_On( LED1 );
      } else if( time_startup == 2*STARTUP_LED_DELAY ) LED_On( LED1 );
      else if( time_startup == 3*STARTUP_LED_DELAY ) LED_On( LED2 );
      else if( time_startup == 4*STARTUP_LED_DELAY ) LED_On( LED3 );
      else if( time_startup == 5*STARTUP_LED_DELAY ) LED_Off( LED0 );
      else if( time_startup == 6*STARTUP_LED_DELAY ) LED_Off( LED1 );
      else if( time_startup == 7*STARTUP_LED_DELAY ) LED_Off( LED2 );
      else if( time_startup == 8*STARTUP_LED_DELAY ) LED_Off( LED3 );
      else if( time_startup >= 9*STARTUP_LED_DELAY ) {
        startup=FALSE;

        audio_buffer_in = 0;
        audio_buffer_out = 0;
        spk_buffer_in = 0;
        spk_buffer_out = 0;
        index_i = 0;


        /* [Martin]
         * At this place we need to wait for next frame synchronization event.
         * This is data mode (I2S, DSP, Left justified....) dependent, so we
         * simply use SSC driver's function, that simply "know what to do"
         */
        // Wait for the next frame synchronization event
        // to avoid channel inversion.  Start with left channel - FS goes low
        ssc_wait_for_FSYNC_RX();

        // Enable now the transfer.
        pdca_enable(PDCA_CHANNEL_SSC_RX);
        pdca_enable_interrupt_reload_counter_zero(PDCA_CHANNEL_SSC_RX);

      }
    }
    //else {
    /* Now measure "loop time" - this is CPU frequency dependent, so it is
     * little bit tricky
     */
    time++;

      /*[Martin]
       * When num_samples constant -> problem. Set according to actual
       * frequency.
       *
       */
      switch(current_freq.frequency)
      {
      case 48000:
        num_samples = 48;
        break;
      case 44100:
        num_samples = 44;
        break;
      case 32000:
        num_samples = 32;
        break;
      case 24000:
        num_samples = 24;
        break;
      case 22050:
        num_samples = 22;
        break;
      case 16000:
        num_samples = 16;
        break;
      case 11025:
        num_samples = 11;
        break;
      case 8000:
        num_samples = 8;
        break;
      default:
        /* Unknown sampling frequency. This may happen when current_freq was
         * not initialized yet. So set to default (48 kHz). Do not print it
         * until work in debug mode. In Windows this may overflow UART.
         */
        /*
        print(DBG_USART,
            "uac1_device_audio_task() Unknown frequency in current_freq."
            "frequency.\n\n");*/
        num_samples = 48;
      }

      // Test if frequency was changed
      if(old_num_samples != num_samples)
      {
        // Set "old" value
        old_num_samples = num_samples;
        // And reset PLL difference
        i_pll_diff = 0;
      }


      if (usb_alternate_setting == 1) {

        if (Is_usb_in_ready(EP_AUDIO_IN)) {  // Endpoint buffer free ?

          Usb_ack_in_ready(EP_AUDIO_IN);  // acknowledge in ready

          // Sync AK data stream with USB data stream
          // AK data is being filled into ~audio_buffer_in, ie if audio_buffer_in is 0
          // buffer 0 is set in the reload register of the pdca
          // So the actual loading is occuring in buffer 1
          // USB data is being taken from audio_buffer_out

          // find out the current status of PDCA transfer
          // gap is how far the audio_buffer_out is from overlapping audio_buffer_in

          num_remaining = pdca_channel->tcr;
          if (audio_buffer_in != audio_buffer_out) {

            // AK and USB using same buffer
            if ( index_i < (AUDIO_BUFFER_SIZE - num_remaining))
            {
              gap = AUDIO_BUFFER_SIZE - num_remaining - index_i;
            }
            else
            {
              gap = AUDIO_BUFFER_SIZE - index_i + AUDIO_BUFFER_SIZE - num_remaining + AUDIO_BUFFER_SIZE;
            }
          } else {
            // usb and pdca working on different buffers
            gap = (AUDIO_BUFFER_SIZE - index_i) + (AUDIO_BUFFER_SIZE - num_remaining);
          }


          // Sync the USB stream with the AK stream
          // throttle back
          if  (gap < AUDIO_BUFFER_SIZE/2) {
            num_samples--;
            //[Martin] Check number of samples
            /**\todo [Martin] It that code really needed?
             * It looks like that num_samples is changed only by 1. Try and add
             * more conditions -> set more proper num_samples
             *
             */
            if(num_samples < 1) // 1kHz is really low rate
            {
              num_samples = 1;
            }
            //[Martin] More processing
            if(gap < AUDIO_BUFFER_SIZE/4)
            {
              num_samples--;
            }
            if(gap < AUDIO_BUFFER_SIZE/8)
            {
              num_samples--;
            }
          } else {
            // speed up
            if  (gap > (AUDIO_BUFFER_SIZE + AUDIO_BUFFER_SIZE/2)) {
              num_samples++;
            }
            //[Martin]
            if(gap > (AUDIO_BUFFER_SIZE + AUDIO_BUFFER_SIZE/2 + AUDIO_BUFFER_SIZE/8))
            {
              num_samples++;
            }
            if(gap > (AUDIO_BUFFER_SIZE + AUDIO_BUFFER_SIZE/2 + AUDIO_BUFFER_SIZE/4))
            {
              num_samples++;
            }

            //[Martin] Check if number of samples fit to buffer
            if(num_samples >= (48+10))
            {
              num_samples = 48+9;
            }
          }

          //[Martin] Get actual word length
          brd_drv_get_data_length(&i_sample_length);

          Usb_reset_endpoint_fifo_access(EP_AUDIO_IN);
          for( i=0 ; i < num_samples ; i++ ) {
            // Fill endpoint with sample raw
            if (mute==FALSE) {
              //[Martin] Shift data in register if we have lower bit resolution
              if(audio_buffer_out == 0)
              {
                // Audio buffer out 0
                audio_buffer_0[index_i+IN_LEFT] =
                    audio_buffer_0[index_i+IN_LEFT]<<(24-i_sample_length);
                audio_buffer_0[index_i+IN_RIGHT] =
                    audio_buffer_0[index_i+IN_RIGHT]<<(24-i_sample_length);
              }
              else
              {
                // Audio buffer out 1
                audio_buffer_1[index_i+IN_LEFT] =
                    audio_buffer_1[index_i+IN_LEFT]<<(24-i_sample_length);
                audio_buffer_1[index_i+IN_RIGHT] =
                    audio_buffer_1[index_i+IN_RIGHT]<<(24-i_sample_length);
              }



              if (audio_buffer_out == 0) {
                // Swap RX data if needed
                if(i_swap_LR_RX == 0)
                {
                  // Do not swap
                  sample_LSB = audio_buffer_0[index_i+IN_LEFT];
                  sample_SB = audio_buffer_0[index_i+IN_LEFT] >> 8;
                  sample_MSB = audio_buffer_0[index_i+IN_LEFT] >> 16;
                }
                else
                {
                  // Swap
                  sample_LSB = audio_buffer_0[index_i+IN_RIGHT];
                  sample_SB = audio_buffer_0[index_i+IN_RIGHT] >> 8;
                  sample_MSB = audio_buffer_0[index_i+IN_RIGHT] >> 16;
                }
              } else {
                // Swap if needed
                if(i_swap_LR_RX == 0)
                {
                  // Do not swap
                  sample_LSB = audio_buffer_1[index_i+IN_LEFT];
                  sample_SB = audio_buffer_1[index_i+IN_LEFT] >> 8;
                  sample_MSB = audio_buffer_1[index_i+IN_LEFT] >> 16;
                }
                else
                {
                  // Swap
                  sample_LSB = audio_buffer_1[index_i+IN_RIGHT];
                  sample_SB = audio_buffer_1[index_i+IN_RIGHT] >> 8;
                  sample_MSB = audio_buffer_1[index_i+IN_RIGHT] >> 16;
                }
              }

              Usb_write_endpoint_data(EP_AUDIO_IN, 8, sample_LSB);
              Usb_write_endpoint_data(EP_AUDIO_IN, 8, sample_SB);
              Usb_write_endpoint_data(EP_AUDIO_IN, 8, sample_MSB);

              if (audio_buffer_out == 0) {
                // Swap if needed
                if(i_swap_LR_RX == 0)
                {
                  // Do not swap
                  sample_LSB = audio_buffer_0[index_i+IN_RIGHT];
                  sample_SB = audio_buffer_0[index_i+IN_RIGHT] >> 8;
                  sample_MSB = audio_buffer_0[index_i+IN_RIGHT] >> 16;
                }
                else
                {
                  // Swap
                  sample_LSB = audio_buffer_0[index_i+IN_LEFT];
                  sample_SB = audio_buffer_0[index_i+IN_LEFT] >> 8;
                  sample_MSB = audio_buffer_0[index_i+IN_LEFT] >> 16;
                }

              } else {
                // Swap if needed
                if(i_swap_LR_RX == 0)
                {
                  // Do not swap
                  sample_LSB = audio_buffer_1[index_i+IN_RIGHT];
                  sample_SB = audio_buffer_1[index_i+IN_RIGHT] >> 8;
                  sample_MSB = audio_buffer_1[index_i+IN_RIGHT] >> 16;
                }
                else
                {
                  // Swap
                  sample_LSB = audio_buffer_1[index_i+IN_LEFT];
                  sample_SB = audio_buffer_1[index_i+IN_LEFT] >> 8;
                  sample_MSB = audio_buffer_1[index_i+IN_LEFT] >> 16;
                }
              }

              Usb_write_endpoint_data(EP_AUDIO_IN, 8, sample_LSB);
              Usb_write_endpoint_data(EP_AUDIO_IN, 8, sample_SB);
              Usb_write_endpoint_data(EP_AUDIO_IN, 8, sample_MSB);

              index_i += 2;
              if (index_i >= AUDIO_BUFFER_SIZE) {
                index_i=0;
                audio_buffer_out = 1 - audio_buffer_out;
              }
            } else {// Mute is true
              Usb_write_endpoint_data(EP_AUDIO_IN, 8, 0x00);
              Usb_write_endpoint_data(EP_AUDIO_IN, 8, 0x00);
              Usb_write_endpoint_data(EP_AUDIO_IN, 8, 0x00);
              Usb_write_endpoint_data(EP_AUDIO_IN, 8, 0x00);
              Usb_write_endpoint_data(EP_AUDIO_IN, 8, 0x00);
              Usb_write_endpoint_data(EP_AUDIO_IN, 8, 0x00);

            }
          }
          Usb_send_in(EP_AUDIO_IN);    // send the current bank
        }
      } // end alt setting == 1

      if (usb_alternate_setting_out == 1) {

        if ( Is_usb_in_ready(EP_AUDIO_OUT_FB) )
        {
          Usb_ack_in_ready(EP_AUDIO_OUT_FB);  // acknowledge in ready

          Usb_reset_endpoint_fifo_access(EP_AUDIO_OUT_FB);
          // Sync CS4344 spk data stream by calculating gap and provide feedback
          num_remaining = spk_pdca_channel->tcr;
          if (spk_buffer_in != spk_buffer_out) {
            // CS4344 and USB using same buffer
            if ( spk_index < (SPK_BUFFER_SIZE - num_remaining)) gap = SPK_BUFFER_SIZE - num_remaining - spk_index;
            else gap = SPK_BUFFER_SIZE - spk_index + SPK_BUFFER_SIZE - num_remaining + SPK_BUFFER_SIZE;
          } else {
            // usb and pdca working on different buffers
            gap = (SPK_BUFFER_SIZE - spk_index) + (SPK_BUFFER_SIZE - num_remaining);
          }

          if (playerStarted){
            //if ((gap < (SPK_BUFFER_SIZE/2)) && (gap < old_gap)) {
            if ((gap < SPK_BUFFER_SIZE - 10) && (delta_num > -FB_RATE_DELTA_NUM)) {
              LED_Toggle(LED0);
              FB_rate -= FB_RATE_DELTA;
              delta_num--;

              //[Martin] This should be commented. Only for debug
              //print_dbg("D-\n");

              //old_gap = gap;
            }
            else
            {
              //if ( (gap > (SPK_BUFFER_SIZE + (SPK_BUFFER_SIZE/2))) && (gap > old_gap)) {
              if ( (gap > SPK_BUFFER_SIZE + 10) && (delta_num < FB_RATE_DELTA_NUM)) {
                LED_Toggle(LED1);
                FB_rate += FB_RATE_DELTA;
                delta_num++;

                //[Martin] This should be commented. Only for debug
                //print_dbg("D+\n");


                //old_gap = gap;
              }
            }
            /* [Martin] When Feedback not work (probably not supported)
             * then is last option change PLL frequency by few PPM and pray.
             * When frequency change is too high, then data process will be
             * desynchronized. So change PLL frequency only if buffer have
             * only few % remaining.
             * Also variable auto_tune must be enabled
             */
            if((gap < SPK_BUFFER_SIZE -(SPK_BUFFER_SIZE*2/3)) &&
                    (i_pll_diff < (MAX_PLL_PPM/10)) &&
                    (delta_num <= -FB_RATE_DELTA_NUM) &&
                    ((time - i_last_time) > PLL_CHANGE_TIME_DIFF) &&
                    (i_auto_tune_enable != 0))
            {
              // We must change frequency - speed up
              if(cs2200_inc_PLL_freq() != 0)
              {
                print_dbg("INC PLL FAILED!\n");
              }

              //[Martin] This should be commented. Only for debug
              //print_dbg("I\n");

              // Increase difference number
              i_pll_diff++;
              // Save actual time
              i_last_time = time;
            }
            if((gap > SPK_BUFFER_SIZE +(SPK_BUFFER_SIZE*2/3)) &&
                    (i_pll_diff > -(MAX_PLL_PPM/10)) &&
                    (delta_num >= FB_RATE_DELTA_NUM) &&
                    ((time - i_last_time) > PLL_CHANGE_TIME_DIFF) &&
                    (i_auto_tune_enable != 0))
            {
              // We must change frequency - slow down
              if(cs2200_dec_PLL_freq() != 0)
              {
                print_dbg("DEC PLL FAILED!\n");
              }

              //[Martin] This should be commented. Only for debug
              //print_dbg("D\n");

              // Decrease difference number
              i_pll_diff--;
              // Save actual time
              i_last_time = time;
            }
          }

          if (Is_usb_full_speed_mode()) {      // FB rate is 3 bytes in 10.14 format
            sample_LSB = FB_rate;
            sample_SB = FB_rate >> 8;
            sample_MSB = FB_rate >> 16;
            Usb_write_endpoint_data(EP_AUDIO_OUT_FB, 8, sample_LSB);
            Usb_write_endpoint_data(EP_AUDIO_OUT_FB, 8, sample_SB);
            Usb_write_endpoint_data(EP_AUDIO_OUT_FB, 8, sample_MSB);
          } else {
            // HS mode
            // FB rate is 4 bytes in 12.14 format
            sample_LSB = FB_rate;
            sample_SB = FB_rate >> 8;
            sample_MSB = FB_rate >> 16;
            sample_HSB = FB_rate >> 24;
            Usb_write_endpoint_data(EP_AUDIO_OUT_FB, 8, sample_LSB);
            Usb_write_endpoint_data(EP_AUDIO_OUT_FB, 8, sample_SB);
            Usb_write_endpoint_data(EP_AUDIO_OUT_FB, 8, sample_MSB);
            Usb_write_endpoint_data(EP_AUDIO_OUT_FB, 8, sample_HSB);
            }

          Usb_send_in(EP_AUDIO_OUT_FB);
          }

        if (Is_usb_out_received(EP_AUDIO_OUT)) {
          Usb_reset_endpoint_fifo_access(EP_AUDIO_OUT);
          num_samples = Usb_byte_count(EP_AUDIO_OUT) / 6;       // Bit resolution (6)

          //[Martin] Get actual word length
          brd_drv_get_data_length(&i_sample_length);

          if(!playerStarted)
          {
            playerStarted = TRUE;
            num_remaining = spk_pdca_channel->tcr;
            spk_buffer_in = spk_buffer_out;
            spk_index = SPK_BUFFER_SIZE - num_remaining;

            spk_index = spk_index & ~((U32)1); // Clear LSB in order to start with L sample
            delta_num = 0;
          }

          for (i = 0; i < num_samples; i++) {
            if (spk_mute) {
              sample_LSB = 0;
              sample_SB = 0;
              sample_MSB = 0;
            } else {
              sample_LSB = Usb_read_endpoint_data(EP_AUDIO_OUT, 8);
              sample_SB = Usb_read_endpoint_data(EP_AUDIO_OUT, 8);
              sample_MSB = Usb_read_endpoint_data(EP_AUDIO_OUT, 8);
            }

            sample = (((U32) sample_MSB) << 16) + (((U32)sample_SB) << 8) + sample_LSB;
            //[Martin] If we want lower resolution than 24b, we must "cut"
            // sample
            sample = sample>>(24-i_sample_length);

            // Switch TX data
            if(i_swap_LR_TX == 0)
            {
              if (spk_buffer_in == 0) spk_buffer_0[spk_index+OUT_LEFT] = sample;
              else spk_buffer_1[spk_index+OUT_LEFT] = sample;
            }
            else
            {
              if (spk_buffer_in == 0) spk_buffer_0[spk_index+OUT_RIGHT] = sample;
              else spk_buffer_1[spk_index+OUT_RIGHT] = sample;
            }



            if (spk_mute) {
              sample_LSB = 0;
              sample_SB = 0;
              sample_MSB = 0;
            } else {
              sample_LSB = Usb_read_endpoint_data(EP_AUDIO_OUT, 8);
              sample_SB = Usb_read_endpoint_data(EP_AUDIO_OUT, 8);
              sample_MSB = Usb_read_endpoint_data(EP_AUDIO_OUT, 8);
            };

            sample = (((U32) sample_MSB) << 16) + (((U32)sample_SB) << 8) + sample_LSB;
            //[Martin] If we want lower resolution than 24b, we must "cut"
            // sample
            sample = sample>>(24-i_sample_length);

            // Switch TX data
            if(i_swap_LR_TX == 0)
            {
              if (spk_buffer_in == 0) spk_buffer_0[spk_index+OUT_RIGHT] = sample;
              else spk_buffer_1[spk_index+OUT_RIGHT] = sample;
            }
            else
            {
              if (spk_buffer_in == 0) spk_buffer_0[spk_index+OUT_LEFT] = sample;
              else spk_buffer_1[spk_index+OUT_LEFT] = sample;
            }


            spk_index += 2;
            if (spk_index >= SPK_BUFFER_SIZE){
              spk_index = 0;
              spk_buffer_in = 1 - spk_buffer_in;
//              spk_buffer_ptr = spk_buffer_in ? spk_buffer_0 : spk_buffer_1;
            }
          }
          Usb_ack_out_received_free(EP_AUDIO_OUT);
        }  // end usb_out_received
      } // end usb_alternate_setting_out == 1
      else
        playerStarted = FALSE;
    //}  // end startup else
  } // end while vTask

}



/**
 * \brief Set auto tune PLL option
 *
 * By setting parameter as non zero, automatic PLL tune will be enabled.\n
 * This feature allow slightly tune external PLL to avoid buffer overflow or\n
 * underflow when feedback EP not work (for some reason).\n
 *
 * @param i_enable Enable (1), disable (0)
 */
inline void uac1_device_audio_set_auto_tune(uint8_t i_enable)
{
  i_auto_tune_enable = i_enable;

  if(i_auto_tune_enable == 0)
    print_dbg("External PLL auto tune set: DISABLED\n");
  else
    print_dbg("External PLL auto tune set: ENABLED\n");
}

/**
 * Get auto tune option value
 *
 * @param p_enable Pointer to memory, where value will be written
 */
inline void uac1_device_audio_get_auto_tune(uint8_t *p_enable)
{
  *p_enable = i_auto_tune_enable;

  if(i_auto_tune_enable == 0)
    print_dbg("External PLL auto tune get: DISABLED\n");
  else
    print_dbg("External PLL auto tune get: ENABLED\n");
}



/**
 * @brief Simply swap left and right channel for transmitting audio data
 * @param i_switch_left_right Options: 0 - do not swap ; 1 - swap L and R
 */
inline void uac1_device_audio_set_swap_LR_TX(uint8_t i_switch_left_right)
{
  if(i_switch_left_right == 0)
  {
    print_dbg("LR TX swap OFF\n");
    i_swap_LR_TX = 0;
  }
  else
  {
    print_dbg("LR TX swap ON\n");
    i_swap_LR_TX = 1;
  }
}


/**
 * @brief Take actual settings of swap L and R channel for transmitter
 * @param p_i_switch_left_right Pointer to memory, where result will be\n
 * written. Options: 0 - do not swap ; 1 - swap L and R
 */
inline void uac1_device_audio_get_switch_LR_TX(uint8_t *p_i_switch_left_right)
{
  *p_i_switch_left_right = i_swap_LR_TX;
}



/**
 * @brief Simply swap left and right channel for receiving audio data
 * @param i_switch_left_right Options: 0 - do not swap ; 1 - swap L and R
 */
inline void uac1_device_audio_set_swap_LR_RX(uint8_t i_switch_left_right)
{
  if(i_switch_left_right == 0)
  {
    print_dbg("LR RX swap OFF\n");
    i_swap_LR_RX = 0;
  }
  else
  {
    print_dbg("LR RX swap ON\n");
    i_swap_LR_RX = 1;
  }
}


/**
 * @brief Take actual settings of swap L and R channel for receiver
 * @param p_i_switch_left_right Pointer to memory, where result will be\n
 * written. Options: 0 - do not swap ; 1 - swap L and R
 */
inline void uac1_device_audio_get_switch_LR_RX(uint8_t *p_i_switch_left_right)
{
  *p_i_switch_left_right = i_swap_LR_RX;
}



#endif  // USB_DEVICE_FEATURE == ENABLED
