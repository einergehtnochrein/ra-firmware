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

#include "lpc54xxx_libconfig.h"

/** \file
 *  \brief CRC driver implementation.
 *
 *  This file contains the driver code for the CRC peripheral.
 */

#include <string.h>                         /* for memcpy() */

#include "lpc54xxx_crc.h"
//#include "lpc54xxx_dma.h"

LPCLIB_DefineRegBit(CRC_MODE_CRC_POLY,          0,  2);
LPCLIB_DefineRegBit(CRC_MODE_BIT_RVS_WR,        2,  1);
LPCLIB_DefineRegBit(CRC_MODE_CMPL_WR,           3,  1);
LPCLIB_DefineRegBit(CRC_MODE_BIT_RVS_SUM,       4,  1);
LPCLIB_DefineRegBit(CRC_MODE_CMPL_SUM,          5,  1);


/** Local context of CRC block. */
static struct CRC_Context {
    LPCLIB_Switch inUse;                    /**< Set if interface open */
#if LPCLIB_DMA
    volatile bool dmaBusy;                  /**< Waiting for DMA callback */ //TODO: need semaphore?
#endif
} crcContext;


/* Get access to the CRC block. */
LPCLIB_Result CRC_open (CRC_Mode mode, CRC_Handle *pHandle)
{
    CRC_Handle handle = &crcContext;

    /* Protect against wrong parameter */
    if (pHandle == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Cannot open CRC twice */
    if (handle->inUse) {
        *pHandle = LPCLIB_INVALID_HANDLE;
        return LPCLIB_BUSY;
    }

    handle->inUse = LPCLIB_YES;

    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_CRC);
    LPC_CRC->MODE = mode & 0xFFFF;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Close the CRC module. */
LPCLIB_Result CRC_close (CRC_Handle *pHandle)
{
    if (pHandle == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_CRC);

    (*pHandle)->inUse = LPCLIB_NO;
    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}



static const LPCLIB_Event crcEvent = {
    .id = LPCLIB_EVENTID_CRC,
    .opcode = CRC_EVENT_COMPLETE,
};


/** Callback for DMA events. */
#if LPCLIB_DMA
static LPCLIB_Result CRC_dmaCallback (LPCLIB_Event event)
{
    switch (event.opcode) {
    case DMA_EVENT_PHASE_COMPLETE:
    case DMA_EVENT_STOP:
        crcContext.dmaBusy = false;
        break;
    }

    return LPCLIB_SUCCESS;
}
#endif



/* Write a block of data to the CRC module. */
LPCLIB_Result CRC_write (CRC_Handle handle,
                         void *pData,
                         uint32_t numBytes,
                         LPCLIB_Callback callback,
                         struct DMA_ChannelContext *dma)
{
    uint32_t n;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (numBytes == 0) {
        return LPCLIB_SUCCESS;                          /* No callback! */
    }

    /* DMA or CPU? */
#if LPCLIB_DMA
    if (dma) {
        ;
        ; //TODO: set up DMA transfer (byte wise)
        ;

        if (callback) {
            return LPCLIB_PENDING;
        }

        /* Wait for end of transfer */
        while (handle->dmaBusy)
            ;
    }
    else {
#else
    (void) dma;
#endif
        /* TODO: Provide optimized copy routine (word write where possible) */
        for (n = 0; n < numBytes; n++) {
            LPC_CRC->WR_DATA8 = ((uint8_t *)pData)[n];
        }

        if (callback) {
            callback(crcEvent);
        }
#if LPCLIB_DMA
    }
#endif

    return LPCLIB_SUCCESS;
}


/** @} */

/** @} addtogroup CRC */

