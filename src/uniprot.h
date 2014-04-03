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

#ifndef _UNIPROT_H_
#define _UNIPROT_H_
//==============================| Configuration |==============================
/* DEFINE PIPE PARAMETERS HERE. This settings can be also in header file of
 * driver.
 * However, PLEASE MAKE SURE that used pipe numbers not collide!
 * Always count pipes from zero! If you "skip" some pipes program will waste
 * RAM, which you really do not want!
 * Structure is UNI_PIPEx_(FRAME_SIZE/FUNC_INIT/FUNC_TASK) where "x" is pipe
 * number.
 * Example:
 *
 * // Define frame size (number of bytes per one transmission)
 * #define UNI_PIPE0_FRAME_SIZE    8
 * // Initialize function which will be called in initialization process
 * #define UNI_PIPE0_FUNC_INIT     USB_Init()
 * // Driver task
 * #define UNI_PIPE0_FUNC_TASK     USB_Task()
 *
 * #define UNI_PIPE1_FRAME_SIZE    3    // Every transmission is 3 bytes long
 * #define UNI_PIPE1_FUNC_INIT     I2C_Init()
 * #define UNI_PIPE1_FUNC_TASK     I2C_Task()
 *
 * #define UNI_PIPE1_FRAME_SIZE    1    // Every transmission is 1 byte long
 * #define UNI_PIPE1_FUNC_INIT     UART_Init()
 * #define UNI_PIPE1_FUNC_TASK     UART_Task()
 */


//================================| Includes |=================================
// HERE INCLUDE NECCESSARY LOW LEVEL DRIVERS WHICH USED PIPES USE
#include "device_generic_hid.h"













//=============================| Other includes |==============================

// Data types - uint8, uint16....
#include <inttypes.h>

// CRC library
// Include only if architecture is AVR8 (else define own CRC function)
#ifdef __AVR_ARCH__
#include <util/crc16.h>
#endif


//===============================| Definitions |===============================

///\brief Character, which will be used as first Byte in header of packet
#define UNI_CHAR_HEADER                 'H'

///\brief Character, which will be used as first Byte in "data" section of packet
#define UNI_CHAR_DATA                   'D'

///\brief Character, which will be used as first Byte in tail of packet
#define UNI_CHAR_TAIL                   'T'

///\brief Character, which will be used as ACK
#define UNI_CHAR_ACK                    'A'

///\brief Character, which will be used as NACK
#define UNI_CHAR_NACK                   'N'

///\brief Character, which will be used as RESET (reset just protocol, not HW)
#define UNI_CHAR_RESET                  'R'

///\brief Character, which will be used as RX buffer overflow
#define UNI_CHAR_BUFFER_OVERFLOW        'O'






#ifdef UNI_PIPE6_FRAME_SIZE
#define UNI_MAX_PIPE  6
#else
#ifdef UNI_PIPE5_FRAME_SIZE
#define UNI_MAX_PIPE  5
#else
#ifdef UNI_PIPE4_FRAME_SIZE
#define UNI_MAX_PIPE  4
#else
#ifdef UNI_PIPE3_FRAME_SIZE
#define UNI_MAX_PIPE  3
#else
#ifdef UNI_PIPE2_FRAME_SIZE
#define UNI_MAX_PIPE  2
#else
#ifdef UNI_PIPE1_FRAME_SIZE
#define UNI_MAX_PIPE  1
#else
#ifdef UNI_PIPE0_FRAME_SIZE
#define UNI_MAX_PIPE  0
#else
#error "At least one pipe MUST be defined! Please take a look at Configuration\
  subsection."
#endif
#endif
#endif
#endif
#endif
#endif
#endif
//===============================| Structures |================================
/**
 * \brief Error codes returned by functions
 */
typedef enum{
  UNI_SUCCESS = 0,                //!< UNI_SUCCESS
  UNI_FAIL = 2,                   //!< UNI_FAIL
  UNI_INVALID_PARAMETER = 3,      //!< UNI_INVALID_PARAMETER
  UNI_TX_PACKET_NOT_CONFIGURED = 4//!< UNI_TX_PACKET_NOT_CONFIGURED
} UNI_RES_CODE;

/**
 * \brief Enumeration of possible data packet states (header, data, tail)
 *
 * Because there is possibility that interface device will support only 8bit\n
 * communication, all states must be divided into 8bit values.
 *
 * \note Numbers are intent start from 7 to avoid miss when programmer try set
 * cmd state by data state value.
 */
typedef enum{
  UNI_STATE_HEADER_H = 7,       //!< UNI_STATE_HEADER_H Header
  UNI_STATE_HEADER_NOB_H = 8,   //!< UNI_STATE_HEADER_NOB_H Number of Bytes
  UNI_STATE_HEADER_NOB_L = 9,   //!< UNI_STATE_HEADER_NOB_L Number of bytes
  UNI_STATE_DATA_D = 10,        //!< UNI_STATE_DATA_D Data introduction
  UNI_STATE_DATA = 11,          //!< UNI_STATE_DATA Data
  UNI_STATE_TAIL_T = 12,        //!< UNI_STATE_TAIL_T Tail
  UNI_STATE_CRC16_H = 13,       //!< UNI_STATE_CRC16_H CRC - High 8 bits
  UNI_STATE_CRC16_L = 14        //!< UNI_STATE_CRC16_L CRC - Low 8 bits
} UNI_PACKET_STATE_DATA;

/**
 * \brief Enumeration of possible command packet frames
 *
 * Because there is possibility that interface device will support only 8bit\n
 * communication, all states must be divided into 8bit values
 */
typedef enum{
  UNI_STATE_CMD_CHAR = 0,//!< CMD_CHAR Command character
  UNI_STATE_CMD_CRC16_H = 1, //!< CRC16_H CRC - High 8 bits
  UNI_STATE_CMD_CRC16_L = 2, //!< CRC16_L CRC - Low 8 bits
} UNI_PACKET_STATE_CMD;

/**
 * \brief Structure which can hold data and command state
 */
typedef struct{
  UNI_PACKET_STATE_DATA data    :4;
  UNI_PACKET_STATE_CMD  cmd     :2;
} UNI_PACKET_STATE_t;

/// \brief Configuration structure for RX and TX packet
typedef struct{
  // Maximal number of RX Bytes. Also use as received Byte counter
  uint16_t i_rx_max_num_of_data_Bytes;
  // Pointer to buffer (where store data)
  uint8_t  *p_rx_buffer;
  // Backup for begin address of buffer
  uint8_t  *p_rx_buffer_bakup;
  // When configuration for RX is done
  uint8_t  i_rx_config_done;


  /* Number of TX Bytes to transmit. Should not be changed for NACK case (all
   * data will be send again)
   */
  uint16_t i_tx_num_of_data_Bytes;
  // Pointer to buffer (where load data)
  uint8_t  *p_tx_buffer;
  // Backup for begin address of buffer
  uint8_t  *p_tx_buffer_backup;
  // When configuration for TX is done
  uint8_t  i_tx_config_done;


  // Number of bytes transmitted by one transmission (common for RX and TX)
  uint8_t  i_frame_size;
} UNI_PACKET_CONFIG_t;


/// \brief Status register flags
typedef struct{
  /**
   * \brief When 1 then low level driver can begin transmission
   *
   * This flag is set and cleared by uniprot.
   */
  uint16_t TX_EN                      :1;
  /**
   * \brief When USB is connected to the host, this flag is set and all layers\n
   * that use UNIPROT should do initialization routine again. It is because of\n
   * user which can disconnect device at any time and if device use battery MCU\n
   * do not perform restart itself. After all re-initialize all necessary\n
   * functions this FLAG MUST be CLEARED HIGHER LAYER!
   */
  uint16_t REINIT                     :1;
  /**
   * \brief When second side send UNI_CHAR_RX_OVERFLOW, message must be shorted.\n
   * This flag signal higher layer, that buffer on the second side have not\n
   * enough memory to save all data (payload).
   */
  uint16_t ERROR_CHAR_RX_OVERFLOW     :1;
  /**
   * \brief When number of data Bytes is higher then configured, then this flag\n
   * is set and all data will be lost. Back will be send UNI_CHAR_RX_OVERFLOW
   */
  uint16_t ERROR_RX_BUFFER_OVERFLOW   :1;
  /**
   * \brief If data in TX buffer are ready -> user must set this flag
   */
  uint16_t DATA_IN_TX_BUFFER_READY    :1;
  /**
   * \brief This flag is set when TX pipe is not configured
   */
  uint16_t ERROR_TX_NOT_CONFIGURED    :1;
  /**
   * \brief This flag is set when RX pipe is not configured
   */
  uint16_t ERROR_RX_NOT_CONFIGURED    :1;
  /**
   * \brief When to receiver comes NACK, then this event is registered to this\n
   * flag, so transmitter will know, that should send data from begin from now.
   */
  uint16_t ERROR_RX_NACK              :1;
  /**
   * \brief This flag is set, when device successfully receive data, but still\n
   * must send ACK to host to confirm received data
   */
  uint16_t SENDING_ACK                :1;
  ///\brief Define position of header error flag for RX stream in i_uni_status_reg
  uint16_t ERROR_HEADER_RX            :1;
  ///\brief Define position of CRC error flag for RX stream in i_uni_status
  uint16_t ERROR_CRC_RX               :1;
  ///\brief Define position of tail error flag for RX stream in i_uni_status
  uint16_t ERROR_TAIL_RX              :1;
  /**
   * \brief Define position of general error (for debug - never should happen)
   * flag for RX stream in i_uni_status *
   */
  uint16_t ERROR_GENERAL_RX           :1;
  /**
   * \brief Define position of general error (for debug - never should happen)
   * flag for TX stream in i_uni_status *
   */
  uint16_t ERROR_GENERAL_TX           :1;
  ///\brief Flag data receive is done
  uint16_t RX_DONE                    :1;
  ///\brief Flag data transmission is done
  uint16_t TX_DONE                    :1;
} UNI_SR_flags;

/**
 * \brief Status register
 *
 * Functions can use structure flags, so every single flag will be easily\n
 * available. When whole all flags have to be clean/set then it can be easily\n
 * done thru status_reg which represents all flags in single variable.
 */
typedef union{
  ///\brief Status registers flags
  UNI_SR_flags  flags;
  ///\brief Complete status register
  uint16_t      status_reg;
} UNI_SR_t;

//===========================| Function prototypes |===========================

/**
 * \brief Configures the board hardware and chip peripherals. Automatic enable
 * global interrupt!
 */
void Uniprot_Init(void);

/**
 * \brief This task handle in and out data stream
 */
void Uniprot_Task(void);

/**
 * \brief Configure RX packet on defined pipe
 *
 * When this function is called, that means that rx_buffer will be\n
 * overwritten!
 *
 * @param i_pipe_ID ID number of pipe. Must not be higher than UNI_MAX_PIPE
 *
 * @param p_rx_buffer Pointer to RX buffer (store data here)
 * @param i_rx_max_num_of_data_Bytes Define maximum received Bytes. Also\n
 * define maximum used size of rx_buffer in Bytes
 * @return  UNI_SUCCESS if all right
 */
UNI_RES_CODE Uniprot_config_RX_packet(    uint8_t i_pipe_ID,
                      uint16_t i_rx_max_num_of_data_Bytes,
                      void     *p_rx_buffer);


/**
 * \brief Configure TX packet on defined pipe
 *
 * When called this function data transmitting start!
 *
 * @param i_pipe_ID ID number of pipe. Must not be higher than UNI_MAX_PIPE
 * @param i_tx_num_of_data_Bytes Number of data Bytes, which driver should
 * expect
 * @param p_tx_buffer Pointer to TX buffer (load data from here)
 * @return  UNI_SUCCESS if all right
 */
UNI_RES_CODE Uniprot_config_TX_packet(    uint8_t i_pipe_ID,
                      uint16_t i_tx_num_of_data_Bytes,
                      void     *p_tx_buffer);

/**
 * \brief Test if RX pipe is ready to receive new data
 * @param i_pipe_ID ID number of pipe. Must not be higher than UNI_MAX_PIPE
 * @return true if ready (1). Else false (0). If i_pipe_ID is invalid, then\n
 * return UNI_INVALID_PARAMETER (2)
 */
UNI_RES_CODE Uniprot_Is_RX_pipe_ready(uint8_t i_pipe_ID);

/**
 * \brief Test if TX pipe is ready transmit new data
 * @param i_pipe_ID ID number of pipe. Must not be higher than UNI_MAX_PIPE
 * @return true if ready (1). Else false (0). If i_pipe_ID is invalid, then\n
 * return UNI_INVALID_PARAMETER (2)
 */
UNI_RES_CODE Uniprot_Is_TX_pipe_ready(uint8_t i_pipe_ID);


/**
 * \brief Test if all uniprot low level data are ready to send
 *
 * This function return 1 if low level driver can begin transmit data
 * (call function Uniprot_tx_data() ) or 0 when not ready yet.
 * @param i_pipe_ID ID number of pipe. Must not be higher than UNI_MAX_PIPE
 * @return True if flag is set (1). Else false (0). If i_pipe_ID is invalid,\n
 *  then return UNI_INVALID_PARAMETER (2)
 */
UNI_RES_CODE Uniprot_Is_TX_transmission_enable(uint8_t i_pipe_ID);


/**
 * \brief This function is called when data came from driver
 *
 * When low level driver receive data, it should call this function with\n
 * correct pipe ID number.
 * @param i_pipe_ID ID number of pipe. Must not be higher than UNI_MAX_PIPE
 * @param p_DataArray Pointer to received data
 */
void Uniprot_rx_data(uint8_t i_pipe_ID, uint8_t* p_DataArray);

/**
 * \brief This function is called when driver (and uniprot) is ready to send\n
 * data
 * @param i_pipe_ID ID number of pipe. Must not be higher than UNI_MAX_PIPE
 * @param p_DataArray Pointer to data to transmit
 */
void Uniprot_tx_data(uint8_t i_pipe_ID, uint8_t* p_DataArray);



/**
 * \brief Check if reinitialize flag is set
 *
 * When flag is set, then return true (1) and clear flag. Also do routines\n
 * needed to uniprot.
 *
 * @param i_pipe_ID Pipe ID (number of used pipe)
 * @return True if flag is set (1). Else false (0). If i_pipe_ID is invalid,\n
 *  then return UNI_INVALID_PARAMETER (2)
 */
UNI_RES_CODE Uniprot_Is_reinit_flag_set(uint8_t i_pipe_ID);








/**
 * \brief Set initialize/reinitialize flag.
 *
 * This flag should set only uniprot itself (only uniprot drivers). Because\n
 * it is internal function, there is not checking of pipe ID.\n
 * This function can be called externally. For example when USB is\n
 * enumerated again. In that case all data are invalid and reinitialize flag\n
 * should be set.
 * @param i_pipe_ID Pipe ID (number of used pipe)
 * @return UNI_SUCCESS if all right
 */
UNI_RES_CODE Uniprot_Set_init_reinit_flag(uint8_t i_pipe_ID);


#endif
