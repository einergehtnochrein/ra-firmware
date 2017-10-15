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
 *  \brief CRC driver interface.
 *
 *  This file contains the API for the CRC block.
 */


#ifndef __LPC54XXX_CRC_H__
#define __LPC54XXX_CRC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** \defgroup CRC
 *  \ingroup API
 *  @{
 */

#include "lpc54xxx_libconfig.h"
#include "lpc54xxx_clkpwr.h"


/** \defgroup CRC_Public_Types CRC Types, enums, macros
 *  @{
 */


/** Handle for the CRC block. */
typedef struct CRC_Context *CRC_Handle;


typedef enum CRC_Polynomial {
    CRC_POLY_CRCCCITT = 0,                  /**< CRC-CCITT */
    CRC_POLY_CRC16 = 1,                     /**< CRC-16 */
    CRC_POLY_CRC32 = 2,                     /**< CRC-32 */
} CRC_Polynomial;


typedef enum CRC_DataBitOrder {
    CRC_DATAORDER_NORMAL = 0,               /**< Normal bit order */
    CRC_DATAORDER_REVERSE = 1,              /**< Reverse bit order */
} CRC_DataBitOrder;


typedef enum CRC_SumBitOrder {
    CRC_SUMORDER_NORMAL = 0,                /**< Normal bit order */
    CRC_SUMORDER_REVERSE = 1,               /**< Reverse bit order */
} CRC_SumBitOrder;


typedef enum CRC_DataPolarity {
    CRC_DATAPOLARITY_NORMAL = 0,            /**< Data not inverted */
    CRC_DATAPOLARITY_INVERSE = 1,           /**< Data inverted (one's complement) */
} CRC_DataPolarity;


typedef enum CRC_SumPolarity {
    CRC_SUMPOLARITY_NORMAL = 0,             /**< Sum not inverted */
    CRC_SUMPOLARITY_INVERSE = 1,            /**< Sum inverted (one's complement) */
} CRC_SumPolarity;


typedef uint32_t CRC_Mode;


typedef enum CRC_CallbackEvent {
    CRC_EVENT_COMPLETE = 0,                 /**< Transaction completed. */
} CRC_CallbackEvent;


/** Predefined CRC modes. */
#define CRC_MODE_HDLC \
    CRC_makeMode(CRC_POLY_CRC16, \
                 CRC_DATAORDER_NORMAL, CRC_SUMORDER_NORMAL, \
                 CRC_DATAPOLARITY_NORMAL, CRC_SUMPOLARITY_INVERSE)



/** @} CRC Types enums, macros */


/** \defgroup CRC_Public_Functions CRC API Functions
 *  @{
 */


/** Get access to the CRC module.
 *
 *  \param[in] mode Mode to select CRC type.
 *  \param[out] pHandle Handle to be used in future API calls to the CRC module.
 *  \retval LPCLIB_SUCCESS Success. \ref handle contains a valid handle.
 *  \retval LPCLIB_BUSY Failure (interface already open).
 *  \retval LPCLIB_ILLEGAL_PARAMETER
 */
LPCLIB_Result CRC_open (CRC_Mode mode, CRC_Handle *pHandle);



/** Close access to the CRC module.
 */
LPCLIB_Result CRC_close (CRC_Handle *pHandle);



/** Read the CRC sum.
 *
 *  \return CRC sum.
 */
static uint32_t CRC_read (CRC_Handle handle);



/** Write a seed value to the CRC register.
 *
 *  The value is processed according to the current data bit order and complement mode!
 *
 *  \param[in] handle CRC handle
 *  \param[in] seed bit seed value
 */
static void CRC_seed (CRC_Handle handle, uint32_t seed);


/** Write a block of data to the CRC module.
 *
 *  \param[in] handle CRC handle
 *  \param[in] pData Points to start of data block
 *  \param[in] numBytes Number of bytes in data block.
 *  \param[in] callback Callback handler (or NULL for synchronous mode).
 *  \param[in] dma DMA channel handle (or NULL if no DMA support required).
 *  \return CRC sum.
 */
struct DMA_ChannelContext;
LPCLIB_Result CRC_write (CRC_Handle handle,
                         void *pData,
                         uint32_t numBytes,
                         LPCLIB_Callback callback,
                         struct DMA_ChannelContext *dma);


/** @} CRC API Functions */


__FORCEINLINE(uint32_t CRC_read (CRC_Handle handle))
{
    (void) handle;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return 0;
    }

    return LPC_CRC->SUM;
}


__FORCEINLINE(CRC_Mode CRC_makeMode (
        CRC_Polynomial polynomial,
        CRC_DataBitOrder dataBitOrder,
        CRC_SumBitOrder sumBitOrder,
        CRC_DataPolarity dataPolarity,
        CRC_SumPolarity sumPolarity))
{
    return (CRC_Mode)(
        (polynomial << 0)       |
        (dataBitOrder << 2)     |
        (sumBitOrder << 4)      |
        (dataPolarity << 3)     |
        (sumPolarity << 5)
        );
}

__FORCEINLINE(void CRC_seed (CRC_Handle handle, uint32_t seed))
{
    if (handle != LPCLIB_INVALID_HANDLE) {
        LPC_CRC->SEED = seed;
    }
}


/** @} CRC API Functions */

/** @} CRC */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef __LPC54Xxx_CRC_H__ */

