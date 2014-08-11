/**
 * \file
 *
 * \brief Virtual bridge, that connect HW library and universal protocol
 * library
 *
 *
 * Created  24.09.2013\n
 * Modified 28.04.2014
 *
 * \version 0.4.1
 * \author Martin Stejskal
 */

#include "HW_bridge_uniprot.h"

//============================| Global variables |=============================
/**
 * \brief RX buffer
 * 
 * Static -> visible only for this .c file
 */
static uint8_t i_rx_buffer[HW_BRIDGE_UNIPROT_RX_BUFFER_SIZE];

/**
 * \brief TX buffer
 * 
 * Static -> visible only for this .c file
 */
static uint8_t i_tx_buffer[HW_BRIDGE_UNIPROT_TX_BUFFER_SIZE];

//================================| Functions |================================

void bridge_init(void)
{
  // Setup necessary hardware
  Uniprot_Init();

  // Set communication pipe (RX pipe, now will wait to request - 1B)
  Uniprot_config_RX_packet(
      HW_BRIDGE_UNIPROT_USED_PIPE,  // Pipe number
      HW_BRIDGE_UNIPROT_RX_BUFFER_SIZE,  // Maximum received Bytes
      &i_rx_buffer);

  /* Can not set TX packet, because for now we do not know witch request
   * will host (PC program) send
   */

  /* If FreeRTOS is used, then create task. Note that
   * "configTSK_HW_bridge_uniprot_*" should be defined in FreeRTOSConfig.h
   * file to keep code clear. However it should be possible set settings
   * here.
   */
#if HW_BRIDGE_UNIPROT_SUPPORT_RTOS != 0
  xTaskCreate(bridge_task_FreeRTOS,             // Task function
        configTSK_HW_bridge_uniprot_NAME,       // String name
        configTSK_HW_bridge_uniprot_STACK_SIZE, // Stack size (2^N)
        NULL,
        configTSK_HW_bridge_uniprot_PRIORITY,   // 0 is lowest
        NULL);
#endif  // FREERTOS_USED
}



#if HW_BRIDGE_UNIPROT_SUPPORT_RTOS != 0
/**
 * \brief FreeRTOS task
 *
 * Instead of complicating bridge_task() we just make function, that is ready\n
 * for FreeRTOS stuff, so code will be still easy to use.
 */
void bridge_task_FreeRTOS(void *pvParameters)
{
  portTickType xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();
  while (TRUE)
  {
    vTaskDelayUntil(&xLastWakeTime, configTSK_HW_bridge_uniprot_PERIOD);

    // Call simple task
    bridge_task();
  }
}
#endif


void bridge_task(void)
{
  // Set default state (when MCU initialize)
  static BRIDGE_STATE i_state = HW_BRIDGE_UNIPROT_STATE_WAITING_FOR_REQUEST;

  /* Define default status. To this variable will be saved status from
   * generic driver, or error codes found during basic checks
   */
  static GD_RES_CODE i_status = GD_FAIL;

  // Backup of device ID
  static uint8_t i_device_ID_backup = 0xFF;


  // Check if reinitialize request was performed -> reset
  if(Uniprot_Is_reinit_flag_set(HW_BRIDGE_UNIPROT_USED_PIPE))
  {
    // Set communication pipe (RX pipe, now will wait to request - 1B)
    Uniprot_config_RX_packet(
        HW_BRIDGE_UNIPROT_USED_PIPE,  // Pipe number
        HW_BRIDGE_UNIPROT_RX_BUFFER_SIZE,  // Maximum received Bytes
        &i_rx_buffer);

    // Set default state
    i_state = HW_BRIDGE_UNIPROT_STATE_WAITING_FOR_REQUEST;
  }

  // Universal protocol task
  Uniprot_Task();

  // Then process data from uniprot if needed...

  // Test Device ID Byte - only if all data received
  if( (i_rx_buffer[0] > hw_get_max_device_index()) &&
    Uniprot_Is_RX_pipe_ready(HW_BRIDGE_UNIPROT_USED_PIPE) &&
    (i_rx_buffer[1] != HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_NUM_OF_DEV)
    )
  {
    // Change state
    i_state = HW_BRIDGE_UNIPROT_STATE_SEND_ONLY_RETURN_CODE;

    // Throw all data from RX buffer because of invalid device ID
    Uniprot_config_RX_packet(
        HW_BRIDGE_UNIPROT_USED_PIPE,
        HW_BRIDGE_UNIPROT_RX_BUFFER_SIZE,
        &i_rx_buffer);

    // Set status
    i_status = GD_INCORRECT_DEVICE_ID;
  }

  // Pointer to i_tx_buffer
  uint8_t *p_tx_buffer;
  p_tx_buffer = &i_tx_buffer[0];


  // Define pointer to metadata
  gd_metadata*  p_metadata;
  // Define pointer to pointer to metadata
  gd_metadata ** p_p_metadata;
  p_p_metadata = &p_metadata;

  // Temporary variable for saving data when call get_setting
  gd_config_struct gd_cfg_struct_tmp;
  // Clean structure
  gd_cfg_struct_tmp.c_name[0] = 0x00;      // NULL
  gd_cfg_struct_tmp.c_descriptor[0] = 0x00;  // NULL
  gd_cfg_struct_tmp.e_in_data_type = void_type;
  gd_cfg_struct_tmp.e_out_data_type = void_type;
  gd_cfg_struct_tmp.i_cmd_id = 0;
  gd_cfg_struct_tmp.p_funtion = 0x00;    // Set pointer to address 0
  gd_cfg_struct_tmp.p_out_value = 0x00;  // Set pointer to address 0
  gd_cfg_struct_tmp.u_in_max_value.data_int32 = 0;
  gd_cfg_struct_tmp.u_in_min_value.data_int32 = 0;
  gd_cfg_struct_tmp.u_out_max_value.data_int32 = 0;
  gd_cfg_struct_tmp.u_out_min_value.data_int32 = 0;

  // Define counters
  uint8_t i_cnt = 0;
  uint8_t i_cnt2 = 0;

  // Temporary variable (32bit long)
  GD_DATA_VALUE gd_data32_tmp;

  // Temporary variable (16bit long)
  uint16_t i16_tmp;


  /* If state is "Waiting for request" and some data were received
   * successfully, then program expect user request
   */
  if( (i_state == HW_BRIDGE_UNIPROT_STATE_WAITING_FOR_REQUEST) &&
    Uniprot_Is_RX_pipe_ready(HW_BRIDGE_UNIPROT_USED_PIPE))
  {
    /* If data ready - we received data - test if Request ID is valid.
     * Request is stored in second Byte of buffer.
     */

    switch(i_rx_buffer[1])
    {
    // Request: Get setting
    case HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_SETTING:
      // Test if TX is ready
      if(Uniprot_Is_TX_pipe_ready(HW_BRIDGE_UNIPROT_USED_PIPE) != 1)
      {
        // If TX is not ready, return and try it next time
        return;
      }
      // Backup device ID
      i_device_ID_backup = i_rx_buffer[0];

      // Load CMD ID
      i16_tmp =   (i_rx_buffer[2]<<8) +
            (i_rx_buffer[3]);

      /* We already read data from RX buffer -> universal protocol can
       * receive new data
       */
      Uniprot_config_RX_packet(
          HW_BRIDGE_UNIPROT_USED_PIPE,  // Pipe number
          HW_BRIDGE_UNIPROT_RX_BUFFER_SIZE,  // Maximum received Bytes
          &i_rx_buffer);

      // Load status and settings to gd_cfg_struct_tmp
      i_status = hw_get_setting(  i_device_ID_backup,
                    i16_tmp,
                    &gd_cfg_struct_tmp);

      // Load data to buffer Byte by Byte
        // Device ID
      *(p_tx_buffer++) = i_device_ID_backup;

        // Result code
      *(p_tx_buffer++) = i_status;

        // CMD ID MSB
      *(p_tx_buffer++) = (gd_cfg_struct_tmp.i_cmd_id)>>8;
        // CMD ID LSB
      *(p_tx_buffer++) = (gd_cfg_struct_tmp.i_cmd_id) & 0xFF;

        // IN TYPE
      *(p_tx_buffer++) = gd_cfg_struct_tmp.e_in_data_type;

        // IN MIN [3]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_in_min_value.data_int32>>24;
        // IN MIN [2]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_in_min_value.data_int32>>16;
        // IN MIN [1]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_in_min_value.data_int32>>8;
        // IN MIN [0]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_in_min_value.data_int32 & 0xFF;

        // IN MAX [3]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_in_max_value.data_int32>>24;
        // IN MAX [2]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_in_max_value.data_int32>>16;
        // IN MAX [1]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_in_max_value.data_int32>>8;
        // IN MAX [0]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_in_max_value.data_int32 & 0xFF;

        // OUT TYPE
      *(p_tx_buffer++) = gd_cfg_struct_tmp.e_out_data_type;

        // OUT MIN [3]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_out_min_value.data_int32>>24;
        // OUT MIN [2]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_out_min_value.data_int32>>16;
        // OUT MIN [1]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_out_min_value.data_int32>>8;
        // OUT MIN [0]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_out_min_value.data_int32 & 0xFF;

        // OUT MAX [3]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_out_max_value.data_int32>>24;
        // OUT MAX [2]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_out_max_value.data_int32>>16;
        // OUT MAX [1]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_out_max_value.data_int32>>8;
        // OUT MAX [0]
      *(p_tx_buffer++) = gd_cfg_struct_tmp.u_out_max_value.data_int32 & 0xFF;

      /* According to output type load bits to TX buffer correctly. Also
       * clear unused bits.
       */
      // Test for 0 length data -> insert 0x00000000
      if(gd_cfg_struct_tmp.e_out_data_type == void_type)
      {
        *(p_tx_buffer++) = 0x00;
        *(p_tx_buffer++) = 0x00;
        *(p_tx_buffer++) = 0x00;
        *(p_tx_buffer++) = 0x00;
      }
      // Test 8 bit values - just copy low 8 bits
      else if((gd_cfg_struct_tmp.e_out_data_type == char_type) ||
          (gd_cfg_struct_tmp.e_out_data_type == uint8_type) ||
          (gd_cfg_struct_tmp.e_out_data_type == int8_type) )
      {
        *(p_tx_buffer++) = 0x00;
        *(p_tx_buffer++) = 0x00;
        *(p_tx_buffer++) = 0x00;
        *(p_tx_buffer++) = gd_cfg_struct_tmp.p_out_value->data_uint8;
      }
      /* Test for 16 bit values. Note that int is in most 8 bit MCU 16bit
       * so, there can be problem. Bridge process that as 32 bit value. If
       * MCU use little endian, then all is OK, but if big endian, there could
       * be problem.
       */
      else if((gd_cfg_struct_tmp.e_out_data_type == int16_type) ||
          (gd_cfg_struct_tmp.e_out_data_type == uint16_type) )
      {
        *(p_tx_buffer++) = 0x00;
        *(p_tx_buffer++) = 0x00;
        *(p_tx_buffer++) = gd_cfg_struct_tmp.p_out_value->data_uint16 >>8;
        *(p_tx_buffer++) = gd_cfg_struct_tmp.p_out_value->data_uint16 & 0xFF;
      }
      // Else value is 32 bit long
      else
      {
          // OUT VALUE [3]
        *(p_tx_buffer++) = gd_cfg_struct_tmp.p_out_value->data_int32>>24;
          // OUT VALUE [2]
        *(p_tx_buffer++) = gd_cfg_struct_tmp.p_out_value->data_int32>>16;
          // OUT VALUE [1]
        *(p_tx_buffer++) = gd_cfg_struct_tmp.p_out_value->data_int32>>8;
          // OUT VALUE [0]
        *(p_tx_buffer++) = gd_cfg_struct_tmp.p_out_value->data_int32 & 0xFF;
      }


      // Add name
      while( gd_cfg_struct_tmp.c_name[i_cnt2] != 0x00 )
      {
        *(p_tx_buffer++) = gd_cfg_struct_tmp.c_name[i_cnt2++];
      }
      // Add NULL character (increase i_cnt2 by 1)
      *(p_tx_buffer++) = 0x00;
      i_cnt2++;

      // And add descriptor
      while( gd_cfg_struct_tmp.c_descriptor[i_cnt] != 0x00 )
      {
        *(p_tx_buffer++) = gd_cfg_struct_tmp.c_descriptor[i_cnt++];
      }
      // Add NULL character (increase i_cnt by 1)
      *(p_tx_buffer++) = 0x00;

      /* Increase i_cnt (program must know how many Bytes will send).
       * In i_cnt is now number of characters, but we need transmit other
       * data too -> add
       */
      i_cnt = i_cnt + 26 + 1 + i_cnt2;

      // Prepare TX pipe
      Uniprot_config_TX_packet(  HW_BRIDGE_UNIPROT_USED_PIPE,
                    i_cnt,
                    &i_tx_buffer[0]);
      break;


    // Request: Set setting
    case HW_BRIDGE_UNIPROT_STATE_REQUEST_SET_SETTINGS:
      // Try to set setting

      // Wait until TX pipe is ready
      if(Uniprot_Is_TX_pipe_ready(HW_BRIDGE_UNIPROT_USED_PIPE) == 0)
      {
        // If not then return
        return;
      }
      // Use i16_tmp for CMD ID. Load low and high Byte
      i16_tmp = (i_rx_buffer[2]<<8) + i_rx_buffer[3];

      // Load value (4 Bytes)
      gd_data32_tmp.data_uint32 = (( (uint32_t)i_rx_buffer[4])<<24) +
                    (( (uint32_t)i_rx_buffer[5])<<16) +
                    (( (uint16_t)i_rx_buffer[6])<<8 ) +
                          (i_rx_buffer[7]);

      // [Device ID (1B), CMD ID (2B), VALUE (4B)]
      i_status = hw_set_setting(  i_rx_buffer[0],
                    i16_tmp,
                    gd_data32_tmp);

      // Backup Device ID
      i_device_ID_backup = i_rx_buffer[0];

      // Change state - send just return code
      i_state = HW_BRIDGE_UNIPROT_STATE_SEND_ONLY_RETURN_CODE;

      /* We already read data from RX buffer -> universal protocol can
       * receive new data
       */
      Uniprot_config_RX_packet(
          HW_BRIDGE_UNIPROT_USED_PIPE,  // Pipe number
          HW_BRIDGE_UNIPROT_RX_BUFFER_SIZE,  // Maximum received Bytes
          &i_rx_buffer);
      break;


    // Request: Get metadata
    case HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_METADATA:
      // Test if TX pipe is ready. If not, then call return function
      if(Uniprot_Is_TX_pipe_ready(HW_BRIDGE_UNIPROT_USED_PIPE) != 1)
      {
        // If not then return
        return;
      }
      /* Get metadata - First in RX buffer is Device ID - check if
       * metadata for wanted device exist (only in debug)
       */
#if HW_BRIDGE_UNIPROT_DEBUG != 0
      if( hw_get_device_metadata(i_rx_buffer[0], p_p_metadata)
          != GD_SUCCESS )
      {
        /* This case never should happen, because device index is
         * checked before. Anyway if problem occurs this signalize
         * that problem is in function "hw_get_device_metadata"
         */

        // If metadata does not exist -> error
        i_state = HW_BRIDGE_UNIPROT_STATE_FAILED_TO_READ_METADATA;
        return;
      }
#else
      // If not in debug - just read data. Should be OK
      hw_get_device_metadata(i_rx_buffer[0], p_p_metadata);
#endif

      // Add device ID
      *(p_tx_buffer++) = i_rx_buffer[0];

      /* Set state to GD_SUCCESS (device ID, request and metadata read
       * successfully)
       */
      *(p_tx_buffer++) = (uint8_t)GD_SUCCESS;

      // Load MAX CMD ID (High byte)
      *(p_tx_buffer++) = p_metadata->i_max_cmd_ID >>8;
      // Load MAX CMD ID (Low byte)
      *(p_tx_buffer++) = p_metadata->i_max_cmd_ID & 0xFF;
      // Load serial number
      *(p_tx_buffer++) = p_metadata->i_serial;

      // Write description
      while( p_metadata->c_description[i_cnt] != 0x00 )
      {
        *(p_tx_buffer++) = p_metadata->c_description[i_cnt++];
      }
      // Add NULL (increase i_cnt by 1)
      *(p_tx_buffer++) = 0x00;

      /* Increase counter by 6 -> now we know how many B universal
       * protocol should transmit
       * (result code, cmd_ID, serial, description)
       */
      i_cnt = i_cnt + 5 +1;

      // OK, so now data in TX buffer are ready - configure TX pipe
      Uniprot_config_TX_packet(  HW_BRIDGE_UNIPROT_USED_PIPE,
                    i_cnt,
                    &i_tx_buffer[0]);

      // Backup Device ID
      i_device_ID_backup = i_rx_buffer[0];

      /* We already read data from RX buffer -> universal protocol can
       * receive new data
       */
      Uniprot_config_RX_packet(
          HW_BRIDGE_UNIPROT_USED_PIPE,  // Pipe number
          HW_BRIDGE_UNIPROT_RX_BUFFER_SIZE,  // Maximum received Bytes
          &i_rx_buffer);

      break;


    // NUMBER OF DEVICES (MAX DEVICE INDEX)
    case HW_BRIDGE_UNIPROT_STATE_REQUEST_GET_NUM_OF_DEV:

      // Test if TX pipe is ready. If not, then call return function
      if(Uniprot_Is_TX_pipe_ready(HW_BRIDGE_UNIPROT_USED_PIPE) != 1)
      {
        // If not then return
        return;
      }

      /* First must be set device ID. Even if it is irrelevant, there
       * must be some. Highly recommended is value 0
       */
      *(p_tx_buffer++) = i_device_ID_backup = 0;

      // Set state to GD_SUCCESS (device ID, request OK)
      *(p_tx_buffer++) = (uint8_t)GD_SUCCESS;

      // Load number of devices
      *(p_tx_buffer++) = hw_get_max_device_index();

      // Configure TX packet
      Uniprot_config_TX_packet(  HW_BRIDGE_UNIPROT_USED_PIPE,
                    3,  // Want transmit 3 Bytes
                    &i_tx_buffer[0]);

      /* We already read data from RX buffer -> universal protocol can
       * receive new data
       */
      Uniprot_config_RX_packet(
          HW_BRIDGE_UNIPROT_USED_PIPE,  // Pipe number
          HW_BRIDGE_UNIPROT_RX_BUFFER_SIZE,  // Maximum received Bytes
          &i_rx_buffer);
      // And that is all
      break;


    // If in buffer is unknown request -> fail
    default:
      // Set state to invalid request
      i_status = GD_INCORRECT_PARAMETER;

      // Device ID change to 254 -> Obvious problem
      i_device_ID_backup = 254;

      // And send this problem
      i_state = HW_BRIDGE_UNIPROT_STATE_SEND_ONLY_RETURN_CODE;

      /* We already read data from RX buffer -> universal protocol can
       * receive new data
       */
      Uniprot_config_RX_packet(
          HW_BRIDGE_UNIPROT_USED_PIPE,  // Pipe number
          HW_BRIDGE_UNIPROT_RX_BUFFER_SIZE,  // Maximum received Bytes
          &i_rx_buffer);
      break;
    }
  }





  // Check if program should send just return code and nothing more
  if(i_state == HW_BRIDGE_UNIPROT_STATE_SEND_ONLY_RETURN_CODE)
  {
    // Check if data send. If yes, change state else stay in this state
    if(Uniprot_Is_TX_pipe_ready(HW_BRIDGE_UNIPROT_USED_PIPE))
    {
      // Load device ID
      i_tx_buffer[0] = i_device_ID_backup;

      // Load value to tx_buffer
      i_tx_buffer[1] = i_status;

      // Configure TX pipe
      Uniprot_config_TX_packet(HW_BRIDGE_UNIPROT_USED_PIPE,
                  2,  // How many Bytes
                  &i_tx_buffer[0]);  // Address of buffer

      i_state = HW_BRIDGE_UNIPROT_STATE_WAITING_FOR_REQUEST;
    }
  }



  /* Check for wrong device ID when reading metadata. However, this case
   * never ever should happen, but it is there for debug purposes or bugs.
   * This only work in debug version
   */
#if HW_BRIDGE_UNIPROT_DEBUG != 0
  if(i_state == HW_BRIDGE_UNIPROT_STATE_FAILED_TO_READ_METADATA)
  {
    while(1);
  }
#endif

}
