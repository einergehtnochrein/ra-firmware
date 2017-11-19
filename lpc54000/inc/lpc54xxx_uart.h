/* Copyright (c) 2016, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 * Neither the name of the author nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file
 *  \brief UART driver interface.
 *
 *  This file contains the API for the UART block.
 */


#ifndef __LPC54XXX_UART_H__
#define __LPC54XXX_UART_H__

/** \defgroup UART
 *  \ingroup API
 *  @{
 */

#include "lpc54xxx_libconfig.h"
#include "lpclib_types.h"



/** \defgroup UART_Public_Types UART Types, enums, macros
 *  @{
 */


#define UART_NUM_UARTS                      4

/** Enumerator for UART names. */
typedef enum UART_Name {
    UART0 = 0,                              /**< UART0 */
    UART1 = 1,                              /**< UART1 */
    UART2 = 2,                              /**< UART2 */
    UART3 = 3,                              /**< UART3 */
} UART_Name;


/** Handle for a UART block. */
typedef struct UART_Context *UART_Handle;


/** Opcodes for \ref UART_ioctl */
typedef enum UART_Opcode {
    UART_OPCODE_INVALID = 0,                /**< (List terminator) */
    UART_OPCODE_SET_BAUDRATE,               /**< Set baudrate by value */
    UART_OPCODE_SET_BAUDRATE_BY_DIVIDERS,   /**< Set baudrate by dividers */
    UART_OPCODE_SET_ASYNC_FORMAT,           /**< Set frame format for asynchronous mode */
    UART_OPCODE_SET_IRDA_FORMAT,            /**< Set IrDA format */
    UART_OPCODE_SET_SYNC_FORMAT,            /**< Set frame format for synchronous mode */
    UART_OPCODE_SET_CALLBACK,               /**< Install callback handler */
    UART_OPCODE_SET_TX_BUFFER,              /**< Specify location and size of TX buffer */
    UART_OPCODE_SET_RX_BUFFER,              /**< Specify location and size of RX buffer */
    UART_OPCODE_SET_FIFO_THRESHOLD,         /**< Set RX FIFO threshold */
    UART_OPCODE_SET_TX_BLOCKING,            /**< TX blocking mode */
    UART_OPCODE_SET_HARDWARE_HANDSHAKE,     /**< Hardware handshake CTS (RTS) */
} UART_Opcode;


/** Enumerator for number of data bits in UART characters */
typedef enum UART_Databits {
    UART_DATABITS_7 = 0,                    /**< 7 bits/frame */
    UART_DATABITS_8 = 1,                    /**< 8 bits/frame */
    UART_DATABITS_9 = 2,                    /**< 8 bits/frame */
} UART_Databits;

/** Enumerator for number of stop bits in UART characters */
typedef enum UART_Stopbits {
    UART_STOPBITS_1 = 0,                    /**< Send 1 stop bit */
    UART_STOPBITS_2 = 1,                    /**< Send 2 stop bits */
} UART_Stopbits;

/** Enumerator for parity option in UART characters */
typedef enum UART_Parity {
    UART_PARITY_NONE = 0,                   /**< No parity bit */
    UART_PARITY_ODD  = 3,                   /**< Odd parity */
    UART_PARITY_EVEN = 2,                   /**< Even parity */
} UART_Parity;

/** Descriptor for UART character format. */
struct UART_ConfigAsyncFormat {
    UART_Databits databits;                 /**< Number of data bits */
    UART_Stopbits stopbits;                 /**< Number of stop bits */
    UART_Parity parity;                     /**< Parity configuration */
};

/** Enumerator for number of stop bits in UART characters */
typedef enum UART_SyncMaster {
    UART_SYNC_SLAVE = 0,                    /**< Slave (clock is input) */
    UART_SYNC_MASTER = 1,                   /**< Master (clock is output) */
} UART_SyncMaster;

/** Enumerator for number of stop bits in UART characters */
typedef enum UART_SamplingClock {
    UART_SAMPLING_FALLING_EDGE = 0,         /**< Sample data with falling clock edge */
    UART_SAMPLING_RISING_EDGE = 1,          /**< Sample data with rising clock edge */
} UART_SamplingClock;

/** Descriptor for UART synchronous format. */
struct UART_ConfigSyncFormat {
    UART_Databits databits;                 /**< Number of data bits */
    UART_SamplingClock samplingClock;       /**< Clock edge for sampling (shift with other edge) */
    UART_SyncMaster syncMaster;             /**< Master/Slave select for clock */
};

/** TX pulse width in IrDA mode. */
typedef enum UART_IrDAPulseWidth {
    UART_IRDA_PULSEWIDTH_VARIABLE   = 0,
    UART_IRDA_PULSEWIDTH_2PCLK      = 1,
    UART_IRDA_PULSEWIDTH_4PCLK      = 3,
    UART_IRDA_PULSEWIDTH_8PCLK      = 5,
    UART_IRDA_PULSEWIDTH_16PCLK     = 7,
    UART_IRDA_PULSEWIDTH_32PCLK     = 9,
    UART_IRDA_PULSEWIDTH_64PCLK     = 11,
    UART_IRDA_PULSEWIDTH_128PCLK    = 13,
    UART_IRDA_PULSEWIDTH_256PCLK    = 15,
} UART_IrDAPulseWidth;

/** Descriptor for IrDA format. */
struct UART_ConfigIrDA {
    UART_IrDAPulseWidth pulseWidth;         /**< Width of TX pulse */
    LPCLIB_Switch invertInput;              /**< Invert input polarity */
};

/** UART baud rate divider (integer and fractional). */
typedef struct UART_BaudrateDividers {
    uint16_t integer;                       /**< Integer part of divider (--> DLM/DLL) */
    uint8_t fracDivAdd;                     /**< Fractional part: DIVADD */
    uint8_t fracMul;                        /**< Fractional part: MUL */
} UART_BaudrateDividers;

struct UART_ConfigBuffer {
    uint8_t *pBuffer;                       /**< Buffer location */
    uint32_t size;                          /**< Buffer size in bytes */
};

typedef enum UART_FifoThreshold {
    UART_FIFO_THRESHOLD_1 = 0,              /**< RX interrupt after 1 character */
    UART_FIFO_THRESHOLD_4 = 1,              /**< RX interrupt after 4 characters */
    UART_FIFO_THRESHOLD_8 = 2,              /**< RX interrupt after 8 characters */
    UART_FIFO_THRESHOLD_14 = 3,             /**< RX interrupt after 14 characters */
} UART_FifoThreshold;

/** Callback configuration. */
struct UART_ConfigCallback {
    LPCLIB_Callback callback;               /**< New callback handler */
    LPCLIB_Callback *pOldCallback;          /**< Takes previously installed callback handler */
};

/** Blocking mode. */
typedef enum UART_BlockingMode {
    UART_NON_BLOCKING = 0,
    UART_BLOCKING,
} UART_BlockingMode;



/** Configuration descriptor in UART ioctl calls. */
typedef struct UART_Config {
    UART_Opcode opcode;                     /**< Config action opcode */

    union {
        uint32_t baudrate;                  /**< Config baudrate */
        struct UART_ConfigAsyncFormat asyncFormat;  /**< Config format for asynchronous mode */
        struct UART_ConfigSyncFormat syncFormat;    /**< Config format for synchronous mode */
        UART_BaudrateDividers *pDividers;   /**< Config baudrate by dividers */
        struct UART_ConfigCallback callback;/**< Callback handler */
        struct UART_ConfigBuffer buffer;    /**< Buffer location and size */
        UART_FifoThreshold threshold;       /**< RX FIFO threshold */
        UART_BlockingMode txBlocking;       /**< TX blocking mode */
        struct UART_ConfigIrDA irda;        /**< IrDA operating mode */
        LPCLIB_Switch hardwareHandshake;
    };
} UART_Config;

/** Config list terminator. */
#define UART_CONFIG_END \
    {.opcode = UART_OPCODE_INVALID}


typedef enum UART_CallbackEvent {
    UART_EVENT_STATUS_LINE,                 /**< Status line event (CTS) */
    UART_EVENT_MODEM,                       /**< Modem line event */
    UART_EVENT_RX,                          /**< Receive character */
} UART_CallbackEvent;

/** @} UART Types, enums, macros */


/** \defgroup UART_Public_Functions UART API Functions
 *  @{
 */


/** Open a UART and obtain a handle.
 *
 *  Always return a valid handle. Handle mustn't be used if false is returned!
 *  Select format 8N1 as a default.
 *
 *  \param[in] bus UART number
 *  \param[out] pHandle Pointer to returned handle
 *  \retval LPCLIB_SUCCESS Success. Handle can be used.
 *  \retval LPCLIB_BUSY UART is alread open (by another task).
 */
LPCLIB_Result UART_open (UART_Name bus, UART_Handle *pHandle);


/** Close a UART channel.
 *
 *  Make sure there is no more activity before calling this function.
 *
 *  \param[in] pHandle UART handle
 */
void UART_close (UART_Handle *pHandle);


/** Read from UART receive buffer.
 *
 *  \param[in] handle Handle for UART
 *  \param[out] pBuffer Buffer that takes RX data
 *  \param[in] nbytes (Maximum) number of bytes to read
 *  \return Number of bytes read
 */
int UART_read (UART_Handle handle, void *pBuffer, int nbytes);


/** Read a complete line (terminated by either CR or LF).
 * 
 *  Returns:
 *  \return 0 if no complete line is in buffer
 *  \return >0 if a complete line was found and buffer was large enough
 *  \return <0 if either a complete line was found but too long, or the RX FIFO overflowed.
 *          In that case the buffer is filled with a string of length (nbytes-1).
 */
int UART_readLine (UART_Handle handle, void *buffer, int nbytes);


/** Write to UART transmit buffer.
 *
 *  Send the specified number of characters. If the transmit buffer cannot take
 *  enough characters, less than nbytes characters are sent. The function
 *  returns the number of actually sent characters. This is an asynchronous function,
 *  which returns immediately and does not wait for the characters to be sent.
 *
 *  \param[in] handle Handle for UART
 *  \param[in] pBuffer Buffer that has TX data
 *  \param[in] nbytes Number of bytes to be written
 *  \return Number of bytes written
 */
int UART_write (UART_Handle handle, const void *pBuffer, int nbytes);


/** Determine number of free TX buffer entries
 *  \return Number of free entries, or -1 if no TX FIFO exists.
 */
int UART_getTxFree (UART_Handle handle);


/** Adjust UART operating parameters.
 *
 *  \param[in] handle Handle for UART
 *  \param[in] pConfig Configuration descriptor
 */
void UART_ioctl (UART_Handle handle, const UART_Config *pConfig);



/** Find optimum fractional divider settings.
 *
 *  Based on the desired baud rate and the main clock frequency, find the optimum
 *  set of fractional divider settings.
 *
 *  \param[in] handle Device handle.
 *  \param[in] baudrate Desired baud rate.
 *  \param[in] expectedPclk PCLK at the time the UART is used (0 = current PCLK).
 *  \param[out] pSettings Calculated divider settings (valid if LPCLIB_SUCCESS is returned).
 *  \retval LPCLIB_SUCCESS ok
 */
LPCLIB_Result UART_findDividers (UART_Handle handle,
                                 uint32_t baudrate,
                                 uint32_t expectedPclk,
                                 UART_BaudrateDividers *pSettings);



#if LPCLIB_UART_MODEM_CONTROL
enum {
    UART_MODEM_EVENT_CTS_CHANGE = (1u << 0),
    UART_MODEM_EVENT_DSR_CHANGE = (1u << 1),
    UART_MODEM_EVENT_RI_CHANGE = (1u << 2),
    UART_MODEM_EVENT_DCD_CHANGE = (1u << 3),
    UART_MODEM_EVENT_CTS_STATE = (1u << 4),
    UART_MODEM_EVENT_DSR_STATE = (1u << 5),
    UART_MODEM_EVENT_RI_STATE = (1u << 6),
    UART_MODEM_EVENT_DCD_STATE = (1u << 7),
};
#endif


/** @} UART API Functions */

/** @} UART */

#endif /* #ifndef __LPC54XXX_UART_H__ */

