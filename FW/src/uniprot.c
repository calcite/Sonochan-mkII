/**
 * \file
 *
 * \brief Universal protocol driver
 *
 1. Set pipes: open uniprot.h file. Find "Configuration" paragraph. Configure
    pipe by following comment. However this settings can be also defined in
    driver header file, but it is not rule.
 2. Go to "Includes" paragraph. Here include low level driver header files.
    Please note, that low level driver SHOULD support uniprot. If driver does
    NOT SUPPORT uniprot, just modify it. On line where driver received data
    call function Uniprot_rx_data(). On line when transmitter is ready send
    data check uniprot TX ready - function Uniprot_Is_TX_transmission_enable().
    If function return 1 (exactly 1, if there is different number than 0 or 1
    there is error!) then can be called function Uniprot_tx_data(). That is it!
 3. Configure RX. This step is necessary only when you want use receiver.
    Just call function Uniprot_config_RX_packet()
 4. Check if RX is complete. Again, only if you use receiver. Call function
    Uniprot_Is_RX_pipe_ready()
 5. Configure TX. This step is necessary only when you want use transmitter.
    Call function Uniprot_config_TX_packet()
 6. Check if TX is complete. Again, only if you use transmitter. Call function
    Uniprot_Is_TX_pipe_ready()
 7. If there is no communication problem, everything should work as expected.
    But.... there always can be problem (bad connection, lazy operating system,
    wizard....), so for these cases is there more functions. In case, that
    uniprot can not synchronize data stream (too high BER) then is set REINIT
    flag. That means hardware driver and uniprot was reset. Obviously uniprot
    can not know what higher layers do with data, so higher layers should
    check if reset occur. This can be done by function
    Uniprot_Is_reinit_flag_set(). Function return 0 if reset not occur. When
    occur, then return 1. If there is some else problem return different
    error code. So when reset occurs that means higher layer should send
    all not confirmed data again. Please remember when reset occurs, then
    all data in RX and TX buffers are invalid! Pipe RX and TX packets MUST be
    configured again!
 *
 * Created  24.09.2013
 * Modified 02.04.2014
 *
 * \version 0.3
 * \author Martin Stejskal
 */

#include "uniprot.h"

//============================| Global variables |=============================
/// \brief Status register for pipes
static UNI_SR_t s_status_reg[UNI_MAX_PIPE+1];

/// \brief Packet configuration for every pipe
static UNI_PACKET_CONFIG_t s_packet_cfg[UNI_MAX_PIPE+1];

/// \brief Expect data (or special command) by receiver
static uint8_t i_rx_expect_data[UNI_MAX_PIPE+1];

/// \brief Expect data (or special command) by transmitter
static uint8_t i_tx_expect_data[UNI_MAX_PIPE+1];

/// \brief State register for  state machine (parser)
static UNI_PACKET_STATE_t i_rx_state[UNI_MAX_PIPE+1];

/// \brief State register for state machine (parser)
static UNI_PACKET_STATE_t i_tx_state[UNI_MAX_PIPE+1];


//=====================| Prototypes of static functions |======================
static void Uniprot_Clear_all(void);

static void Uniprot_Clear_pipe(uint8_t i_pipe_ID);

// If not AVR8 architecture, then must define _crc_xmodem_function
#ifndef __AVR_ARCH__
/**
 * \brief CRC Function
 * 
 * When we are not on AVR8 architecture we cannot use optimized CRC routines,\n
 * so there is same function, but not so optimized. 
 */
uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data)
{
        uint8_t i;

        crc = crc ^ ((uint16_t)data << 8);
        for (i=0; i<8; i++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }

        return crc;
}
#endif
//================================| Functions |================================

void Uniprot_Init(void)
{
  // Initialize pipe by pipe (and set frame length)
#ifdef UNI_PIPE0_FRAME_SIZE
  s_packet_cfg[0].i_frame_size = UNI_PIPE0_FRAME_SIZE;
  UNI_PIPE0_FUNC_INIT;
#endif
#ifdef UNI_PIPE1_FRAME_SIZE
  s_packet_cfg[1].i_frame_size = UNI_PIPE1_FRAME_SIZE;
  UNI_PIPE1_FUNC_INIT;
#endif
#ifdef UNI_PIPE2_FRAME_SIZE
  s_packet_cfg[2].i_frame_size = UNI_PIPE2_FRAME_SIZE;
  UNI_PIPE2_FUNC_INIT;
#endif
#ifdef UNI_PIPE3_FRAME_SIZE
  s_packet_cfg[3].i_frame_size = UNI_PIPE3_FRAME_SIZE;
  UNI_PIPE3_FUNC_INIT;
#endif
#ifdef UNI_PIPE4_FRAME_SIZE
  s_packet_cfg[4].i_frame_size = UNI_PIPE4_FRAME_SIZE;
  UNI_PIPE4_FUNC_INIT;
#endif
#ifdef UNI_PIPE5_FRAME_SIZE
  s_packet_cfg[5].i_frame_size = UNI_PIPE5_FRAME_SIZE;
  UNI_PIPE5_FUNC_INIT;
#endif
#ifdef UNI_PIPE6_FRAME_SIZE
  s_packet_cfg[6].i_frame_size = UNI_PIPE6_FRAME_SIZE;
  UNI_PIPE6_FUNC_INIT;
#endif


  // Set all needed variable to predefined value
  Uniprot_Clear_all();
}


static void Uniprot_Clear_all(void)
{
  uint8_t i;
  // Clear status registers and configuration for packets
  for(i=UNI_MAX_PIPE ; i<(UNI_MAX_PIPE+1) ; i--)
  {
    // Clear all errors
    s_status_reg[i].status_reg =    0;

    i_rx_expect_data[i] = true;
    i_tx_expect_data[i] = true;

    i_rx_state[i].data = UNI_STATE_HEADER_H;
    i_rx_state[i].cmd  = UNI_STATE_CMD_CHAR;
    i_tx_state[i].data = UNI_STATE_HEADER_H;
    i_tx_state[i].cmd  = UNI_STATE_CMD_CHAR;


    // Zero -> nothing to receive/transmit
    s_packet_cfg[i].i_tx_num_of_data_Bytes = 0;
    // Set pointers to buffers also as zero
    s_packet_cfg[i].p_rx_buffer = 0;
    s_packet_cfg[i].p_rx_buffer_bakup = 0;
    s_packet_cfg[i].p_tx_buffer = 0;
    s_packet_cfg[i].p_tx_buffer_backup = 0;
    // Configuration was not set yet
    s_packet_cfg[i].i_rx_config_done = false;
    s_packet_cfg[i].i_tx_config_done = false;

    // TX pipe not TX data now, but it is ready
    s_status_reg[i].flags.TX_DONE = 1;
  }
}


static void Uniprot_Clear_pipe(uint8_t i_pipe_ID)
{
  i_rx_state[i_pipe_ID].data = UNI_STATE_HEADER_H;
  i_rx_state[i_pipe_ID].cmd  = UNI_STATE_CMD_CHAR;
  i_tx_state[i_pipe_ID].data = UNI_STATE_HEADER_H;
  i_tx_state[i_pipe_ID].cmd  = UNI_STATE_CMD_CHAR;

  // Clear all errors
  s_status_reg[i_pipe_ID].status_reg =    0;

  i_rx_expect_data[i_pipe_ID] = true;
  i_tx_expect_data[i_pipe_ID] = true;

  // Zero -> nothing to receive/transmit
  s_packet_cfg[i_pipe_ID].i_tx_num_of_data_Bytes = 0;
  // Set pointers to buffers also as zero
  s_packet_cfg[i_pipe_ID].p_rx_buffer = 0;
  s_packet_cfg[i_pipe_ID].p_rx_buffer_bakup = 0;
  s_packet_cfg[i_pipe_ID].p_tx_buffer = 0;
  s_packet_cfg[i_pipe_ID].p_tx_buffer_backup = 0;
  // Configuration was not set yet
  s_packet_cfg[i_pipe_ID].i_rx_config_done = false;
  s_packet_cfg[i_pipe_ID].i_tx_config_done = false;

  // TX pipe not TX data now, but it is ready
  s_status_reg[i_pipe_ID].flags.TX_DONE = 1;
}


inline void Uniprot_Task(void)
{
  static uint8_t i_pipe_switch = 0;

  switch(i_pipe_switch)
  {
  #ifdef UNI_PIPE0_FRAME_SIZE
  case 0:
      UNI_PIPE0_FUNC_TASK;
    break;
  #endif
  #ifdef UNI_PIPE1_FRAME_SIZE
  case 1:
      UNI_PIPE1_FUNC_TASK;
    break;
  #endif
  #ifdef UNI_PIPE2_FRAME_SIZE
  case 2:
      UNI_PIPE2_FUNC_TASK;
    break;
  #endif
  #ifdef UNI_PIPE3_FRAME_SIZE
  case 3:
      UNI_PIPE3_FUNC_TASK;
    break;
  #endif
  #ifdef UNI_PIPE4_FRAME_SIZE
  case 4:
      UNI_PIPE4_FUNC_TASK;
    break;
  #endif
  #ifdef UNI_PIPE5_FRAME_SIZE
  case 5:
      UNI_PIPE5_FUNC_TASK;
    break;
  #endif
  #ifdef UNI_PIPE6_FRAME_SIZE
  case 6:
      UNI_PIPE6_FUNC_TASK;
    break;
  #endif
  // Some extra task
  default:
    break;
  }

  // According to actual task check if user want send data
  // Check if user want send data and if conditions are OK
  if((s_status_reg[i_pipe_switch].flags.DATA_IN_TX_BUFFER_READY == true)
     &&
     (i_tx_expect_data[i_pipe_switch] == true))
  {
    s_status_reg[i_pipe_switch].flags.TX_EN = 1;
  }
  
  i_pipe_switch++;
  if(i_pipe_switch >= (UNI_MAX_PIPE+1))
  {
	  i_pipe_switch = 0;
  }


}



inline UNI_RES_CODE Uniprot_config_RX_packet(  uint8_t    i_pipe_ID,
                    uint16_t  i_rx_max_num_of_data_Bytes,
                    void    *p_rx_buffer)
{
  // Test input parameter
  if(i_pipe_ID > UNI_MAX_PIPE)
  {
    return UNI_INVALID_PARAMETER;
  }

  // Ready to RX another data -> RX is not done yet
  s_status_reg[i_pipe_ID].flags.RX_DONE = 0;

  // Save pointer (it is important only address for now)
  s_packet_cfg[i_pipe_ID].p_rx_buffer = (uint8_t*)p_rx_buffer;

  /* Save backup pointer (useful when RX all data and higher layer want
   * receive data in same format -> must set pointer to begin. Or when
   * receive data fail -> must save data to buffer from begin
   */
  s_packet_cfg[i_pipe_ID].p_rx_buffer_bakup = (uint8_t*)p_rx_buffer;

  // Save information about maximum received data Bytes
  s_packet_cfg[i_pipe_ID].i_rx_max_num_of_data_Bytes =
      i_rx_max_num_of_data_Bytes;

  // Configuration is done now
  s_packet_cfg[i_pipe_ID].i_rx_config_done = 1;

  // Clear RX packet not configured
  s_status_reg[i_pipe_ID].flags.ERROR_RX_NOT_CONFIGURED = 0;

  return UNI_SUCCESS;
}



inline UNI_RES_CODE Uniprot_config_TX_packet(uint8_t i_pipe_ID,
                      uint16_t i_tx_num_of_data_Bytes,
                      void     *p_tx_buffer)
{
  // Test input parameter
  if(i_pipe_ID > UNI_MAX_PIPE)
  {
    return UNI_INVALID_PARAMETER;
  }

  // Set number of Bytes
  s_packet_cfg[i_pipe_ID].i_tx_num_of_data_Bytes = i_tx_num_of_data_Bytes;

  // Save pointer (it is important only address for now)
  s_packet_cfg[i_pipe_ID].p_tx_buffer = (uint8_t*)p_tx_buffer;

  // Save pointer to begin of tx_buffer (as backup value)
  s_packet_cfg[i_pipe_ID].p_tx_buffer_backup = (uint8_t*)p_tx_buffer;

  // Configuration is done now
  s_packet_cfg[i_pipe_ID].i_tx_config_done = true;

  /* Check if there are not any errors on TX device -> if no errors ->
   * -> set TX done flag
   */
  if(s_status_reg[i_pipe_ID].flags.ERROR_GENERAL_TX == 0)
  {
    // Set data bit, clear TX done bit (Start transmitting)
    s_status_reg[i_pipe_ID].flags.DATA_IN_TX_BUFFER_READY = 1;
    s_status_reg[i_pipe_ID].flags.TX_DONE =                 0;
  }

  // Clear TX packet not configured
  s_status_reg[i_pipe_ID].flags.ERROR_TX_NOT_CONFIGURED = 0;

  return UNI_SUCCESS;
}


inline UNI_RES_CODE Uniprot_Is_RX_pipe_ready(uint8_t i_pipe_ID)
{
  // Test input parameter
  if(i_pipe_ID > UNI_MAX_PIPE)
  {
    return UNI_INVALID_PARAMETER;
  }
  return s_status_reg[i_pipe_ID].flags.RX_DONE;
}



inline UNI_RES_CODE Uniprot_Is_TX_pipe_ready(uint8_t i_pipe_ID)
{
  // Test input parameter
  if(i_pipe_ID > UNI_MAX_PIPE)
  {
    return UNI_INVALID_PARAMETER;
  }
  return s_status_reg[i_pipe_ID].flags.TX_DONE;
}



inline UNI_RES_CODE Uniprot_Is_TX_transmission_enable(uint8_t i_pipe_ID)
{
  // Test input parameter
  if(i_pipe_ID > UNI_MAX_PIPE)
  {
    return UNI_INVALID_PARAMETER;
  }
  return s_status_reg[i_pipe_ID].flags.TX_EN;
}




void Uniprot_rx_data(uint8_t i_pipe_ID, uint8_t* p_DataArray)
{
  /* CRC variable for data. Due to load distribution, CRC is calculated
   * continuous. So program need to know last value -> static.
   */
  static uint16_t crc16_data[UNI_MAX_PIPE+1];

  /* When receiving data/cmd byte by byte, then program need to know (store
   * to memory) first 2 Bytes to make right decisions.
   * Size is 2 because we need store CMD character and CRC H byte. CRC L byte
   * is not needed because when calculating CRC than is CRC L always available
   * in p_DataArray.
   */
  static uint8_t  i_rx_tmp_bytes[UNI_MAX_PIPE+1][2];


  /* Because p_DataArray can be every time different (depend on i_frame_size
   * of every pipe) there is this counter, which count how many bytes was so
   * far processed. When this number is equal i_frame_size, that means stop
   * processing data from p_DataArray. Then must wait for new data.
   */
  uint8_t i_byte_cnt = 0;

  // Temporary variable for calculating CRC when receiving command
  uint16_t crc16_cmd = 0;

  // Test if expect command or data
  if(i_rx_expect_data[i_pipe_ID] == false)
  {// Expect commands
    // Switch according to actual state
    switch(i_rx_state[i_pipe_ID].cmd)
    {
    // First expect CMD character
    case UNI_STATE_CMD_CHAR:
      // Test character (if CMD is known or not)
      if((p_DataArray[0] != UNI_CHAR_ACK) &&
         (p_DataArray[0] != UNI_CHAR_NACK) &&
         (p_DataArray[0] != UNI_CHAR_RESET) &&
         (p_DataArray[0] != UNI_CHAR_BUFFER_OVERFLOW) )
      {
        // So, whatever. Next time receive data. Set error flag
        i_rx_expect_data[i_pipe_ID] = true;
        s_status_reg[i_pipe_ID].flags.ERROR_GENERAL_RX = 1;
        return; // Nothing to process.
      }
      // When CMD valid, then save it to memory
      i_rx_tmp_bytes[i_pipe_ID][0] = p_DataArray[0];
      // Change state
      i_rx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CRC16_H;
      // Increase byte counter and check if we have something to proceed or not
      if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
      { // Nothing to process -> return
        return;
      }
      // Else continue on next state
    case UNI_STATE_CMD_CRC16_H:
      /* Just load CRC16 High byte. Thanks to i_byte_cnt this can be done
       * very easily. So load CRC H
       */
      i_rx_tmp_bytes[i_pipe_ID][1] = p_DataArray[i_byte_cnt];
      // Change state
      i_rx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CRC16_L;
      // Increase byte counter and check if we have something to proceed or not
      if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
      { // Nothing to process -> return
        return;
      }
      // Else continue on next state
    case UNI_STATE_CMD_CRC16_L:
      // CMD character was identified. So calculate CRC and then do some action
      crc16_cmd = _crc_xmodem_update(crc16_cmd, i_rx_tmp_bytes[i_pipe_ID][0]);
      crc16_cmd = _crc_xmodem_update(crc16_cmd, i_rx_tmp_bytes[i_pipe_ID][1]);
      crc16_cmd = _crc_xmodem_update(crc16_cmd, p_DataArray[i_byte_cnt]);
      /* No matter what, next time we receive data. Status should be set to
       * begin too
       */
      i_rx_expect_data[i_pipe_ID] = true;
      i_rx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CHAR;

      // Check CRC result
      if(crc16_cmd != 0)
      {// If problem -> set flags and so on
        s_status_reg[i_pipe_ID].flags.ERROR_CRC_RX = 1;
        // Enable TX data
        s_status_reg[i_pipe_ID].flags.TX_EN = 1;
        return;
      }
      // Else CRC OK. According to CMD character process data, set flags...
      switch(i_rx_tmp_bytes[i_pipe_ID][0])
      {
      case UNI_CHAR_ACK:
        // Set TX done flag
        s_status_reg[i_pipe_ID].flags.TX_DONE = 1;
        // TX now send user data
        i_tx_expect_data[i_pipe_ID] = true;
        return;
      case UNI_CHAR_NACK:
        // Set Error flag
        s_status_reg[i_pipe_ID].flags.ERROR_RX_NACK = 1;
        // Send data again
        i_tx_expect_data[i_pipe_ID] = true;
        // Enable TX data
        s_status_reg[i_pipe_ID].flags.TX_EN = 1;
        return;
      case UNI_CHAR_BUFFER_OVERFLOW:
        // Overflow flag
        s_status_reg[i_pipe_ID].flags.ERROR_CHAR_RX_OVERFLOW = 1;
        // TX now expect user data
        i_tx_expect_data[i_pipe_ID] = true;
        return;
      case UNI_CHAR_RESET:
        s_status_reg[i_pipe_ID].flags.REINIT = 1;
        /* When higher layer will call Uniprot_Is_reinit_flag_set(),
         * then uniprot variables will be set to default and higher layer
         * do reset too
         */
        return;
      default:
        // Undefined state -> developer error -> stay in loop
        while(1);
      }
    // Undefined state -> developer error -> stay in loop
    default:
      while(1);
    }// switch(i_rx_state[i_pipe_ID].cmd)
  }// if(i_rx_expect_data[i_pipe_ID] == false)
  // Else expect data

  // Check state
  switch(i_rx_state[i_pipe_ID].data)
  {
  case UNI_STATE_HEADER_H:
    // When waiting for request -> TX expect command (ACK/NACK and so on)
    i_tx_expect_data[i_pipe_ID] = false;
    /* When receive data, then transmitter should not start TX data, because
     * maybe program will want send some command (ACK, NACK and so on)
     */
    s_status_reg[i_pipe_ID].flags.TX_EN = 0;

    // Check first Byte and check if RX pipe was configured (initialized)
    if((p_DataArray[i_byte_cnt] == UNI_CHAR_HEADER) &&
       (s_packet_cfg[i_pipe_ID].i_rx_config_done == false))
    {// Header found, but RX packet is not configured
      // Not configured yet -> set error flag and return
      s_status_reg[i_pipe_ID].flags.ERROR_RX_NOT_CONFIGURED = 1;
      return;
    }

    /* Because it is possible send emergency reset command at anytime, we must
     * save first two Bytes to some temporary buffer. Then we can check CRC
     * and decide if emergency reset was send or not.
     */
    if((p_DataArray[i_byte_cnt] == UNI_CHAR_HEADER) ||
       (p_DataArray[i_byte_cnt] == UNI_CHAR_RESET))
    {// Wanted byte -> save it!
      i_rx_tmp_bytes[i_pipe_ID][0] = p_DataArray[i_byte_cnt];
      // Change state
      i_rx_state[i_pipe_ID].data = UNI_STATE_HEADER_NOB_H;
      // Increase byte counter and check if we have something to proceed or not
      if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
      { // Nothing to process -> return
        return;
      }
      // Else continue on next state (else condition is "skipped")
    }
    else
    {// Else input Byte is incorrect. Never mind, just wait for new one.
      s_status_reg[i_pipe_ID].flags.ERROR_HEADER_RX = 1;
      s_status_reg[i_pipe_ID].flags.TX_EN = 1;
      return;
    }

  case UNI_STATE_HEADER_NOB_H:
    // Second byte. Just save it!
    i_rx_tmp_bytes[i_pipe_ID][1] = p_DataArray[i_byte_cnt];
    // Change state
    i_rx_state[i_pipe_ID].data = UNI_STATE_HEADER_NOB_L;
    // Increase byte counter and check if we have something to proceed or not
    if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
    { // Nothing to process -> return
      return;
    }
    // Else continue on next state

  case UNI_STATE_HEADER_NOB_L:
    /* Third byte. So now there are these options:
     * 1) Receiving data (standard)
     * 2) Reset command (exception)
     * 3) Some mess (First character was UNI_CHAR_RESET, but CRC is not
     *    correct)
     */
    // Test for Reset command or mess
    if(i_rx_tmp_bytes[i_pipe_ID][0] == UNI_CHAR_RESET)
    {
      // Can not continue receiving data -> set state back to begin
      i_rx_state[i_pipe_ID].data = UNI_STATE_HEADER_H;
      // Now check pre-calculated CRC
      if((i_rx_tmp_bytes[i_pipe_ID][1] == 0x7A) &&
         (p_DataArray[i_byte_cnt]      == 0xB7))
      {// Emergency reset
        /* When higher layer will call Uniprot_Is_reinit_flag_set(),
         * then uniprot variables will be set to default and higher layer
         * do reset too
         */
        s_status_reg[i_pipe_ID].flags.REINIT = 1;
        return;
      }
      // Else there is just some mess -> wait for new payload
      return;
    }// Test for Reset command or mess
    // Else there are valid data (so far). Save number of bytes value
    i_rx_tmp_bytes[i_pipe_ID][0] = p_DataArray[i_byte_cnt];

    // Change state
    i_rx_state[i_pipe_ID].data = UNI_STATE_DATA_D;
    // Increase byte counter and check if we have something to proceed or not
    if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
    { // Nothing to process -> return
      return;
    }
    // Else continue on next state

  case UNI_STATE_DATA_D:
    // Now there should be UNI_CHAR_DATA
    if(p_DataArray[i_byte_cnt] != UNI_CHAR_DATA)
    {// If there is not -> just some mess -> set status back
      i_rx_state[i_pipe_ID].data = UNI_STATE_HEADER_H;
      // Also set header error flag
      s_status_reg[i_pipe_ID].flags.ERROR_HEADER_RX = 1;
      s_status_reg[i_pipe_ID].flags.TX_EN = 1;
      return;
    }
    // Else header seems to be valid. Clear some flags
    s_status_reg[i_pipe_ID].flags.ERROR_CHAR_RX_OVERFLOW = 0;
    s_status_reg[i_pipe_ID].flags.ERROR_CRC_RX = 0;
    s_status_reg[i_pipe_ID].flags.ERROR_HEADER_RX = 0;
    s_status_reg[i_pipe_ID].flags.ERROR_TAIL_RX = 0;
    s_status_reg[i_pipe_ID].flags.RX_DONE = 0;

    // Start calculating CRC ("H" + Number of bytes (2x) + "D")
    crc16_data[i_pipe_ID] = _crc_xmodem_update(0,
                                               UNI_CHAR_HEADER);
    crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                               i_rx_tmp_bytes[i_pipe_ID][1]);
    crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                               i_rx_tmp_bytes[i_pipe_ID][0]);
    crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                               UNI_CHAR_DATA);

    // Recover pointer to RX buffer
    s_packet_cfg[i_pipe_ID].p_rx_buffer =
        s_packet_cfg[i_pipe_ID].p_rx_buffer_bakup;
    // Test if number of data Bytes is smaller or equal than maximum
    if(((i_rx_tmp_bytes[i_pipe_ID][1]<<8) | i_rx_tmp_bytes[i_pipe_ID][0]) >
      s_packet_cfg[i_pipe_ID].i_rx_max_num_of_data_Bytes)
    {// If actual is > than max defined -> throw data (do not overflow buffer)
      s_status_reg[i_pipe_ID].flags.ERROR_CHAR_RX_OVERFLOW = 1;
      // We will receive data, but data will be threw
    }
    // Set number of remaining bytes
    s_packet_cfg[i_pipe_ID].i_rx_max_num_of_data_Bytes =
        (i_rx_tmp_bytes[i_pipe_ID][1]<<8) | i_rx_tmp_bytes[i_pipe_ID][0];

    // Change state
    i_rx_state[i_pipe_ID].data = UNI_STATE_DATA;
    // Increase byte counter and check if we have something to proceed or not
    if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
    { // Nothing to process -> return
      return;
    }
    // Else continue on next state

  case UNI_STATE_DATA:
    /* Receive until all data saved (or break when there are not any other
     * data)
     */
    while(s_packet_cfg[i_pipe_ID].i_rx_max_num_of_data_Bytes != 0)
    {
      // Read byte and save it to buffer if there is not buffer overflow
      if(s_status_reg[i_pipe_ID].flags.ERROR_CHAR_RX_OVERFLOW == 0)
      {
        *(s_packet_cfg[i_pipe_ID].p_rx_buffer++) = p_DataArray[i_byte_cnt];
      }
      // Calculate CRC
      crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                                 p_DataArray[i_byte_cnt]);

      // Decrease number of bytes
      s_packet_cfg[i_pipe_ID].i_rx_max_num_of_data_Bytes--;

      // Test if there is anything else read or not...
      if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
      {// Nothing to receive... so return for now
        return;
      }
      // Else continue
    }
    // OK, when here then change state
    i_rx_state[i_pipe_ID].data = UNI_STATE_TAIL_T;

  case UNI_STATE_TAIL_T:
    // Here it is. Tail. Check it!
    if(p_DataArray[i_byte_cnt] != UNI_CHAR_TAIL)
    {// Well, even this may happen :(
      s_status_reg[i_pipe_ID].flags.ERROR_TAIL_RX = 1;
      s_status_reg[i_pipe_ID].flags.TX_EN = 1;
      i_rx_state[i_pipe_ID].data = UNI_STATE_HEADER_H;
      // Send command
      i_rx_expect_data[i_pipe_ID] = false;
      return;
    }
    // Tail OK -> calc CRC
    crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                               UNI_CHAR_TAIL);

    // Change state
    i_rx_state[i_pipe_ID].data = UNI_STATE_CRC16_H;

    // Increase byte counter and check if we have something to proceed or not
    if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
    { // Nothing to process -> return
      return;
    }
    // Else continue on next state
  case UNI_STATE_CRC16_H:
    // Just calculate CRC
    crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                               p_DataArray[i_byte_cnt]);
    // Change state
    i_rx_state[i_pipe_ID].data = UNI_STATE_CRC16_L;

    // Increase byte counter and check if we have something to proceed or not
    if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
    { // Nothing to process -> return
      return;
    }
    // Else continue on next state
  case UNI_STATE_CRC16_L:
    // Calculate CRC
    crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                               p_DataArray[i_byte_cnt]);

    // Check CRC value
    if(crc16_data[i_pipe_ID] == 0)
    {
      // Test overflow flag
      if(s_status_reg[i_pipe_ID].flags.ERROR_CHAR_RX_OVERFLOW == 0)
      {// RX buffer overflow flag not set
        // Set RX done flag
        s_status_reg[i_pipe_ID].flags.RX_DONE = 1;
        // Set SEND ACK flag
        s_status_reg[i_pipe_ID].flags.SENDING_ACK = 1;
      }
      // Send command back (report state -> ACK / OVERFLOW and so on)
      i_tx_expect_data[i_pipe_ID] = false;
      // And continue behind if{}
    }
    else
    {// Else CRC not 0 -> problem
      s_status_reg[i_pipe_ID].flags.ERROR_CRC_RX = 1;
    }
    // Send data - Enable TX
    s_status_reg[i_pipe_ID].flags.TX_EN = 1;
    // Anyway, set state to begin
    i_rx_state[i_pipe_ID].data = UNI_STATE_HEADER_H;
    return;
  // This never should happen - developer error
  default:
    while(1);
  }
  return;
}

void Uniprot_tx_data(uint8_t i_pipe_ID, uint8_t* p_DataArray)
{
  /* CRC variable for data. Due to load distribution, CRC is calculated
   * continuous. So program need to know last value -> static.
   */
  static uint16_t crc16_data[UNI_MAX_PIPE+1];

  /* Because p_DataArray can be every time different (depend on i_frame_size
   * of every pipe) there is this counter, which count how many bytes was so
   * far processed. When this number is equal i_frame_size, that means stop
   * processing data from p_DataArray. Then must wait for new data.
   */
  uint8_t i_byte_cnt = 0;

  // Variable for calculating CRC when transmitting command
  static uint16_t crc16_cmd[UNI_MAX_PIPE+1];

  // Count how many data Bytes remain to transmit
  static uint16_t i_tx_num_of_remain_data_Byes[UNI_MAX_PIPE+1];

  /* First program should look for most critical errors - generic error
   * (not defined)
   */
  if((s_status_reg[i_pipe_ID].flags.ERROR_GENERAL_RX == 1) ||
     (s_status_reg[i_pipe_ID].flags.ERROR_GENERAL_TX == 1))
  {
    /* This errors never should happen. However if they appears, that
     * means something is definitely wrong -> send RESET request
     */
    switch(i_tx_state[i_pipe_ID].cmd)
    {
    // First Byte
    case UNI_STATE_CMD_CHAR:
      p_DataArray[i_byte_cnt] = UNI_CHAR_RESET;
      // Change state
      i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CRC16_H;
      // Test if transmitter is ready for more bytes or not
      if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
      { // Nothing to process -> return
        return;
      }
    // Second Byte
    case UNI_STATE_CMD_CRC16_H:
      crc16_cmd[i_pipe_ID] = _crc_xmodem_update(0, UNI_CHAR_RESET);
      p_DataArray[i_byte_cnt] = (crc16_cmd[i_pipe_ID]>>8);
      // Change state
      i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CRC16_L;
      // Test if transmitter is ready for more bytes or not
      if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
      { // Nothing to process -> return
        return;
      }
    // Third Byte
    case UNI_STATE_CMD_CRC16_L:
      p_DataArray[i_byte_cnt] = (crc16_cmd[i_pipe_ID] & 0xFF);
      // Reset status register
      s_status_reg[i_pipe_ID].status_reg = 0;
      // Set reinitialize flag
      s_status_reg[i_pipe_ID].flags.REINIT = 1;
      /* If we got there - stop transmitting.
       * Just send this packet
       */
      s_status_reg[i_pipe_ID].flags.TX_EN  = 0;
      // Change state
      i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CHAR;
      return;
    // This should never happen -> developer error
    default:
      while(1);
    }// Switch state
  }// If general error (RX or TX) -> reset



  // OK, now normal operation. Test if expect command or data
  if(i_tx_expect_data[i_pipe_ID] == false)
  {// Expect command
    // Test for data corruption
    if((s_status_reg[i_pipe_ID].flags.ERROR_HEADER_RX == 1) ||
       (s_status_reg[i_pipe_ID].flags.ERROR_TAIL_RX   == 1) ||
       (s_status_reg[i_pipe_ID].flags.ERROR_CRC_RX    == 1))
    {
      // Switch according to state machine
      switch(i_tx_state[i_pipe_ID].cmd)
      {
      case UNI_STATE_CMD_CHAR:
        p_DataArray[i_byte_cnt] = UNI_CHAR_NACK;
        // Change state
        i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CRC16_H;
        // Test if transmitter is ready for more bytes or not
        if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
        { // Nothing to process -> return
          return;
        }
      case UNI_STATE_CMD_CRC16_H:
        crc16_cmd[i_pipe_ID] = _crc_xmodem_update(0, UNI_CHAR_NACK);
        p_DataArray[i_byte_cnt] = (crc16_cmd[i_pipe_ID]>>8);
        // Change state
        i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CRC16_L;
        // Test if transmitter is ready for more bytes or not
        if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
        { // Nothing to process -> return
          return;
        }
      case UNI_STATE_CMD_CRC16_L:
        p_DataArray[i_byte_cnt] = (crc16_cmd[i_pipe_ID] & 0xFF);
        // RX now ready to read data
        i_rx_expect_data[i_pipe_ID] = true;
        // If we got there -> stop transmitting
        s_status_reg[i_pipe_ID].flags.TX_EN = 0;
        // Change state
        i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CHAR;
        return;
      // This should never happen -> developer error
      default:
        while(1);
      }// State machine
    }// Test for data corruption

    /* Test for RX buffer overflow (number of received Bytes was higher than
     * maximum defined in configuration Uniprot_config_RX_packet). So, buffer
     * was not rewritten, but no data was stored to rx_buffer -> send
     * UNI_CHAR_RX_OVERFLOW -> receiver (PC) will recognize this command and
     * send it to higher layer
     */
    if(s_status_reg[i_pipe_ID].flags.ERROR_CHAR_RX_OVERFLOW == 1)
    {
      // Switch according to state machine
      switch(i_tx_state[i_pipe_ID].cmd)
      {
      case UNI_STATE_CMD_CHAR:
        p_DataArray[i_byte_cnt] = UNI_CHAR_BUFFER_OVERFLOW;
        // Change state
        i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CRC16_H;
        // Test if transmitter is ready for more bytes or not
        if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
        { // Nothing to process -> return
          return;
        }
      case UNI_STATE_CMD_CRC16_H:
        // Calculate CRC
        crc16_cmd[i_pipe_ID] = _crc_xmodem_update(0, UNI_CHAR_BUFFER_OVERFLOW);
        p_DataArray[i_byte_cnt] = (crc16_cmd[i_pipe_ID]>>8);
        // Change state
        i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CRC16_L;
        // Test if transmitter is ready for more bytes or not
        if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
        { // Nothing to process -> return
          return;
        }
      case UNI_STATE_CMD_CRC16_L:
        p_DataArray[i_byte_cnt] = (crc16_cmd[i_pipe_ID] & 0xFF);
        // Clear error flag (NACK will be send)
        s_status_reg[i_byte_cnt].flags.ERROR_CHAR_RX_OVERFLOW = 0;
        // Next time we can send data
        i_tx_expect_data[i_pipe_ID] = true;
        // RX ready to read data
        i_rx_expect_data[i_pipe_ID] = true;
        // If we go here - stop transmitting
        s_status_reg[i_pipe_ID].flags.TX_EN = 0;
        // Change state
        i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CHAR;
        return;
      // This should never happen -> developer error
      default:
        while(1);
      }// State machine
    }// Test for RX OVERFLOW


    /* If there is not any error and RX want send ACK send it
     */
    if(s_status_reg[i_pipe_ID].flags.SENDING_ACK == 1)
    {
      // Switch according to state machine
      switch(i_tx_state[i_pipe_ID].cmd)
      {
      case UNI_STATE_CMD_CHAR:
        p_DataArray[i_byte_cnt] = UNI_CHAR_ACK;
        // Change state
        i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CRC16_H;
        // Test if transmitter is ready for more bytes or not
        if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
        { // Nothing to process -> return
          return;
        }
      case UNI_STATE_CMD_CRC16_H:
        // Calculate CRC
        crc16_cmd[i_pipe_ID] = _crc_xmodem_update(0, UNI_CHAR_ACK);
        p_DataArray[i_byte_cnt] = (crc16_cmd[i_pipe_ID]>>8);
        // Change state
        i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CRC16_L;
        // Test if transmitter is ready for more bytes or not
        if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
        { // Nothing to process -> return
          return;
        }
      case UNI_STATE_CMD_CRC16_L:
        p_DataArray[i_byte_cnt] = (crc16_cmd[i_pipe_ID] & 0xFF);
        // Clear sending flag
        s_status_reg[i_pipe_ID].flags.SENDING_ACK = 0;
        // TX expect user data
        i_tx_expect_data[i_pipe_ID] = true;
        // RX ready to read data
        i_rx_expect_data[i_pipe_ID] = true;
        // If we got here stop transmitting
        s_status_reg[i_pipe_ID].flags.TX_EN = 0;
        // Change state
        i_tx_state[i_pipe_ID].cmd = UNI_STATE_CMD_CHAR;
        return;
      // This should never happen -> developer error
      default:
        while(1);
      }// State machine
    }
    else
    {// Testing ACK - not even ACK command
      // This should never happen -> critical error
      s_status_reg[i_pipe_ID].flags.ERROR_GENERAL_TX = 1;
      return;
    }


  }
  // Else expect data

  // Say RX function, that should expect command instead of data
  i_rx_expect_data[i_pipe_ID] = false;

  /* If NACK was send from PC or when received command have wrong CRC ->
   * -> send data from begin -> set buffer
   */
  if((s_status_reg[i_pipe_ID].flags.ERROR_RX_NACK == 1) ||
     (s_status_reg[i_pipe_ID].flags.ERROR_CRC_RX  == 1))
  {
    // Set TX buffer pointer back to begin
    s_packet_cfg[i_pipe_ID].p_tx_buffer =
        s_packet_cfg[i_pipe_ID].p_tx_buffer_backup;
    // Set state back to header
    i_tx_state[i_pipe_ID].data = UNI_STATE_HEADER_H;
    // Clear error flags
    s_status_reg[i_pipe_ID].flags.ERROR_RX_NACK = 0;
    s_status_reg[i_pipe_ID].flags.ERROR_CRC_RX  = 0;
  }

  // State machine
  switch(i_tx_state[i_pipe_ID].data)
  {
  case UNI_STATE_HEADER_H:
    // Check if TX packet was configured
    if(s_packet_cfg[i_pipe_ID].i_tx_config_done == 0)
    {
      s_status_reg[i_pipe_ID].flags.ERROR_TX_NOT_CONFIGURED = 1;
      return;
    }
    // Clear TX done flag
    s_status_reg[i_pipe_ID].flags.TX_DONE = 0;
    // Add "UNI_CHAR_HEADER" to p_DataArray and do CRC routine
    p_DataArray[i_byte_cnt] = UNI_CHAR_HEADER;
    crc16_data[i_pipe_ID] = _crc_xmodem_update(0, UNI_CHAR_HEADER);
    // Change state
    i_tx_state[i_pipe_ID].data = UNI_STATE_HEADER_NOB_H;
    // Test if transmitter is ready for more bytes or not
    if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
    { // Nothing to process -> return
      return;
    }
  case UNI_STATE_HEADER_NOB_H:
    // Add "number of Bytes" High Byte parameter + CRC
    p_DataArray[i_byte_cnt] =
        (s_packet_cfg[i_pipe_ID].i_tx_num_of_data_Bytes)>>8;
    crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                               p_DataArray[i_byte_cnt]);
    // Change state
    i_tx_state[i_pipe_ID].data = UNI_STATE_HEADER_NOB_L;
    // Test if transmitter is ready for more bytes or not
    if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
    { // Nothing to process -> return
      return;
    }
  case UNI_STATE_HEADER_NOB_L:
    // Add "number of Bytes" Low Byte parameter + CRC
    p_DataArray[i_byte_cnt] =
        (s_packet_cfg[i_pipe_ID].i_tx_num_of_data_Bytes & 0xFF);
    crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                               p_DataArray[i_byte_cnt]);
    // Set number of remaining bytes
    i_tx_num_of_remain_data_Byes[i_pipe_ID] =
        s_packet_cfg[i_pipe_ID].i_tx_num_of_data_Bytes;
    // Change state
    i_tx_state[i_pipe_ID].data = UNI_STATE_DATA_D;
    // Test if transmitter is ready for more bytes or not
    if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
    { // Nothing to process -> return
      return;
    }
  case UNI_STATE_DATA_D:
    // Add UNI_CHAR_DATA + CRC
    p_DataArray[i_byte_cnt] = UNI_CHAR_DATA;
    crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                               UNI_CHAR_DATA);
    // Change state
    i_tx_state[i_pipe_ID].data = UNI_STATE_DATA;
    // Test if transmitter is ready for more bytes or not
    if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
    { // Nothing to process -> return
      return;
    }
  case UNI_STATE_DATA:
    while(i_tx_num_of_remain_data_Byes[i_pipe_ID] != 0)
    {
      // Write one byte to DataArray
      p_DataArray[i_byte_cnt] = *(s_packet_cfg[i_pipe_ID].p_tx_buffer++);
      // Calculate CRC
      crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                                 p_DataArray[i_byte_cnt]);
      // One Byte send, decrease counter
      i_tx_num_of_remain_data_Byes[i_pipe_ID]--;
      // Test if transmitter is ready for more bytes or not
      if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
      { // Nothing to process -> return
        return;
      }
    }
    // Change state
    i_tx_state[i_pipe_ID].data = UNI_STATE_TAIL_T;
  case UNI_STATE_TAIL_T:
    // Add UNI_CHAR_TAIL + CRC
    p_DataArray[i_byte_cnt] = UNI_CHAR_TAIL;
    crc16_data[i_pipe_ID] = _crc_xmodem_update(crc16_data[i_pipe_ID],
                                               UNI_CHAR_TAIL);
    // Change state
    i_tx_state[i_pipe_ID].data = UNI_STATE_CRC16_H;
    // Test if transmitter is ready for more bytes or not
    if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
    { // Nothing to process -> return
      return;
    }
  case UNI_STATE_CRC16_H:
    // Send CRC High Byte
    p_DataArray[i_byte_cnt] = (crc16_data[i_pipe_ID]>>8);
    // Change state
    i_tx_state[i_pipe_ID].data = UNI_STATE_CRC16_L;
    // Test if transmitter is ready for more bytes or not
    if((++i_byte_cnt) >= s_packet_cfg[i_pipe_ID].i_frame_size)
    { // Nothing to process -> return
      return;
    }
  case UNI_STATE_CRC16_L:
    // Send CRC Low Byte
    p_DataArray[i_byte_cnt] = (crc16_data[i_pipe_ID] & 0xFF);
    // Set pointer to TX buffer back to begin (easier for debug)
    s_packet_cfg[i_pipe_ID].p_tx_buffer =
        s_packet_cfg[i_pipe_ID].p_tx_buffer_backup;
    // This is last frame - stop transmitting
    s_status_reg[i_pipe_ID].flags.TX_EN = 0;
    // Say RX function, that should expect command instead of data
    i_rx_expect_data[i_pipe_ID] = false;
    /* Clear flag UNI_SR_DATA_IN_TX_BUFFER_READY (now can be TX buffer
     * rewritten)
     */
    s_status_reg[i_pipe_ID].flags.DATA_IN_TX_BUFFER_READY = 0;

    // Fill rest of TX frame (if any) 0xFF (it is recommended in specification)
    while((++i_byte_cnt) < s_packet_cfg[i_pipe_ID].i_frame_size)
    {
      p_DataArray[i_byte_cnt] = 0xFF;
    }
    // Change state
    i_tx_state[i_pipe_ID].data = UNI_STATE_HEADER_H;
    return;
  // Default situation never should happen. If yes, then it is developer error
  default:
    while(1);
  }

  return;
}


inline UNI_RES_CODE Uniprot_Is_reinit_flag_set(uint8_t i_pipe_ID)
{
  static uint8_t i_first_time[UNI_MAX_PIPE+1];

  // Test input parameter
  if(i_pipe_ID > UNI_MAX_PIPE)
  {
    return UNI_INVALID_PARAMETER;
  }

  // Check if flag is set
  if( s_status_reg[i_pipe_ID].flags.REINIT == 1 )
  {
    /* If this flag is set for the first time since device was powered,
     * reinitialize would done a lot of damage, because reinitialize
     * procedure stay in loop (reinitialize is done every time when USB
     * is plugged)
     */
    if(i_first_time[i_pipe_ID] == 0)
    {
      // Change static variable
      i_first_time[i_pipe_ID] = 1;

      // Clear flag
      s_status_reg[i_pipe_ID].flags.REINIT = 0;

      // And return 0 -> reinitialize is not needed
      return 0;
    }
    /* Else this is not first time, then clear all needed variables. Now
     * reinitialize from side uniprot is done
     */
    Uniprot_Clear_pipe(i_pipe_ID);

    // Return 1 -> higher layer should do reinitialize routine
    return 1;
  }

  // If not set -> return zero
  return 0;
}



inline UNI_RES_CODE Uniprot_Set_init_reinit_flag(uint8_t i_pipe_ID)
{
  s_status_reg[i_pipe_ID].flags.REINIT = 1;
  return UNI_SUCCESS;
}
