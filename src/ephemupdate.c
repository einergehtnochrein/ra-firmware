/* Copyright (c) 2016, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 * Neither the name of the author nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"

#include "sys.h"
#include "rinex.h"
#include "ephemupdate.h"



#define EPHEMUPDATE_QUEUE_LENGTH        10

typedef struct {
    uint8_t opcode;
} EPHEMUPDATE_Message;


/** Message opcodes for SYS task. */
enum {
    EPHEMUPDATE_OPCODE_xxx,
};



/** Identifiers for OS timers. */

enum {
    EPHEMUPDATE_TIMERMAGIC_ACTION,
};

typedef enum {
    EPHEMUPDATE_STATE_IDLE = 0,
    EPHEMUPDATE_STATE_TRANSFER,

    __EPHEMUPDATE_STATE_FLASH_ACTION__,

    EPHEMUPDATE_STATE_TRANSFER_WAIT_ERASE,
    EPHEMUPDATE_STATE_TRANSFER_WAIT_PROGRAM,
    EPHEMUPDATE_STATE_FINALIZE_WAIT_ERASE,
    EPHEMUPDATE_STATE_FINALIZE_WAIT_PROGRAM,
} EPHEMUPDATE_State;


#define EPHEMUPDATE_RESPONSE_OK     "0"
#define EPHEMUPDATE_RESPONSE_ERROR  "1"


struct EPHEMUPDATE_Context {
    struct pt pt;

    osTimerId actionTick;

    uint16_t transferSize;
    uint16_t crc16;
    uint16_t currentPosition;
    EPHEMUPDATE_State state;

    uint16_t currentPage;
    bool pageDirty;
    uint8_t page[256] __ALIGN(4);

    bool terminate;
} euContext;


__SECTION(".ephemsdownload")
GPS_Ephemeris _ephemerisDownload;



LPCLIB_Result EPHEMUPDATE_open (EPHEMUPDATE_Handle *pHandle)
{
    *pHandle = &euContext;

    return LPCLIB_SUCCESS;
}




static void _EPHEMUPDATE_sendResponse (int action, const char *status)
{
    char s[24];
    sprintf(s, "%d,%s", action, status);
    SYS_send2Host(HOST_CHANNEL_EPHEMUPDATE, s);
}


static bool _EPHEMUPDATE_mustUpdate (void)
{
    return true;
}


static LPCLIB_Result _EPHEMUPDATE_flushPage (EPHEMUPDATE_Handle handle)
{
    LPCLIB_Result iapResult;
    uint32_t sectorNumber;
    uint32_t pageNumber;
    uint32_t address;


    if (!handle->pageDirty) {
        return LPCLIB_SUCCESS;
    }

    __disable_irq();

    address = (uint32_t)&_ephemerisDownload + 256 * handle->currentPage;
    IAP_address2SectorNumber(address, &sectorNumber, NULL);
    IAP_address2PageNumber(address, &pageNumber, NULL);

    iapResult = IAP_prepareSectorsForWriteOperation(
            sectorNumber,
            sectorNumber,
            NULL);
    if (iapResult == LPCLIB_SUCCESS) {
        iapResult = IAP_erasePages(
                pageNumber,
                pageNumber,
                NULL);
        if (iapResult == LPCLIB_SUCCESS) {
            iapResult = IAP_prepareSectorsForWriteOperation(
                    sectorNumber,
                    sectorNumber,
                    NULL);
            if (iapResult == LPCLIB_SUCCESS) {
                iapResult = IAP_copyRamToFlash(
                        address,
                        (uint32_t)&handle->page,
                        256,
                        NULL);
            }
        }
    }
    handle->pageDirty = false;

    __enable_irq();

    return iapResult;
}


/* Update the ephemeris structure from the uploaded temporary copy. */
static LPCLIB_Result _EPHEMUPDATE_finalizeUpdate (EPHEMUPDATE_Handle handle)
{
    LPCLIB_Result iapResult;
    uint32_t sectorNumber;
    uint32_t fromPage, toPage;
    uint32_t srcAddress;
    uint32_t dstAddress;
    (void) handle;

    __disable_irq();

    srcAddress = (uint32_t)&_ephemerisDownload;
    dstAddress = (uint32_t)ephemeris;
    IAP_address2SectorNumber(dstAddress, &sectorNumber, NULL);
    IAP_address2PageNumber(dstAddress, &fromPage, NULL);
    IAP_address2PageNumber(dstAddress + sizeof(GPS_Ephemeris) - 1, &toPage, NULL);

    iapResult = IAP_prepareSectorsForWriteOperation(
            sectorNumber,
            sectorNumber,
            NULL);
    if (iapResult == LPCLIB_SUCCESS) {
        iapResult = IAP_erasePages(
                fromPage,
                toPage,
                NULL);
        if (iapResult == LPCLIB_SUCCESS) {
            /* Copy page-wise from temporary buffer in flash to RAM, and from there copy to destination in flash */
            int nPages = (sizeof(GPS_Ephemeris) + 255) / 256;
            int i;
            for (i = 0; i < nPages; i++) {
                /* Get data from temporary flash buffer */
                memcpy(handle->page, (void *)(srcAddress + 256 * i), 256);

                /* Prepare sector, then flash it */
                iapResult = IAP_prepareSectorsForWriteOperation(
                    sectorNumber,
                    sectorNumber,
                    NULL);
                if (iapResult == LPCLIB_SUCCESS) {
                    iapResult = IAP_copyRamToFlash(
                            dstAddress + 256 * i,
                            (uint32_t)handle->page,
                            256,
                            NULL);
                    if (iapResult != LPCLIB_SUCCESS) {
                        break;
                    }
                }
            }
        }
    }

    __enable_irq();

    return iapResult;
}


static LPCLIB_Result _EPHEMUPDATE_processHexRecord (EPHEMUPDATE_Handle handle, const char *record)
{
    LPCLIB_Result result = LPCLIB_ERROR;
    unsigned int i;
    const char *p;
    unsigned int byte;
    uint8_t checksum;
    bool checksumOk = false;

    /* The record must have the format:
     * :LLaaaa00xxcc
     * LL = number of payload bytes (xx)
     * aaaa = address offset of first byte in record
     * xx = payload byte (repeat LL times)
     * cc = checksum
     */

    /* Read count and address */
    unsigned int count, address;
    if (sscanf(record, ":%2X%4X", &count, &address) == 2) {
        /* Formal verification of checksum */
        p = &record[9];
        checksum = count + (address % 256) + (address / 256);
        for (i = 0; i < count; i++) {
            if (sscanf(p, "%2X", &byte) != 1) {
                break;
            }
            checksum += byte;
            p += 2;
        }
        if (i == count) {                   /* Have we read all bytes? */
            /* Checksum should match */
            if (sscanf(p, "%2X", &byte) == 1) {
                checksum += byte;
                if (checksum == 0) {        /* If all adds up to zero the line is ok */
                    checksumOk = true;
                }
            }
        }

        /* Do the values make any sense?
         * Address must start from current position and must not exceed 8 KiB

         */
        if (checksumOk &&
            (count > 0) &&
            (address < 8192) &&
            (address + count - 1 < 8192) &&
            (address == handle->currentPosition)) {

            result = LPCLIB_SUCCESS;

            /* Read payload again and write to page buffer. Flush page if necessary. */
            p = &record[9];
            for (i = 0; i < count; i++) {
                /* Flush and preload page if new */
                if ((address + i) / 256 != handle->currentPage) {
                    if (_EPHEMUPDATE_flushPage(handle) != LPCLIB_SUCCESS) {
                        result = LPCLIB_ERROR;
                    }

                    handle->currentPage = (address + i) / 256;
                    memcpy(handle->page, &((uint8_t *)ephemeris)[256 * handle->currentPage], 256);
                }

                if (sscanf(p, "%2X", &byte) != 1) {
                    result = LPCLIB_ERROR;
                    break;
                }
                handle->page[(address + i) % 256] = byte;
                p += 2;

                handle->pageDirty = true;
            }

            handle->currentPosition += count;
        }
    }

    return result;
}



/* Calculate the CRC32 checksum of the ephemerides in flash (use seed = 0xFFFFFFFF) */
static uint32_t _EPHEMUPDATE_getEphemerisChecksum (void)
{
    CRC_Handle crc = LPCLIB_INVALID_HANDLE;
    CRC_Mode crcMode;
    uint32_t checksum = 0;

    crcMode = CRC_makeMode(
            CRC_POLY_CRC32,
            CRC_DATAORDER_NORMAL,
            CRC_SUMORDER_NORMAL,
            CRC_DATAPOLARITY_NORMAL,
            CRC_SUMPOLARITY_NORMAL
            );
    if (CRC_open(crcMode, &crc) == LPCLIB_SUCCESS) {
        CRC_seed(crc, 0xFFFFFFFF);
        CRC_write(crc, ephemeris, sizeof(*ephemeris), NULL, NULL);
        checksum = CRC_read(crc);

        CRC_close(&crc);
    }

    return checksum;
}



LPCLIB_Result EPHEMUPDATE_processCommand (EPHEMUPDATE_Handle handle, const char *commandLine)
{
    const char *response = EPHEMUPDATE_RESPONSE_ERROR;

    /* Read action (second field) */
    int action;
    if (sscanf(commandLine, "#%*d,%d", &action) == 1) {
        switch (action) {
        case 0:     /* Start new transfer */
            /* If there is still flash activity going on, just note that we need to terminate */
            if (handle->state > __EPHEMUPDATE_STATE_FLASH_ACTION__) {
                handle->terminate = true;
            }
            else {
                /* Extract transfer size */
                int size;
                if (sscanf(commandLine, "#%*d,%*d,%d", &size) == 1) {
                    /* If size seems reasonable, start a new transfer now. */
                    if (size == sizeof(GPS_Ephemeris)) {
                        handle->transferSize = size;
                        handle->state = EPHEMUPDATE_STATE_TRANSFER;
                        handle->crc16 = 0;
                        handle->currentPosition = 0;
                        handle->currentPage = (uint16_t)-1;
                        handle->pageDirty = false;
                        response = EPHEMUPDATE_RESPONSE_OK;
                    }
                }
            }
            break;

        case 1:     /* HEX record */
            /* New records can only be accepted in transfer state */
            if (handle->state == EPHEMUPDATE_STATE_TRANSFER) {
                /* A HEX record must start with ':'. Process from there. */
                char *record = strchr(commandLine, ':');
                if (record) {
                    if (_EPHEMUPDATE_processHexRecord(handle, record) == LPCLIB_SUCCESS) {
                        /* Successfully processed */
                        response = EPHEMUPDATE_RESPONSE_OK;
                    }
                }
            }
            break;

        case 2:     /* Finalize transmission */
            /* We can only finalize if we are in transfer state */
            if (handle->state == EPHEMUPDATE_STATE_TRANSFER) {
                /* Did we receive exactly the amount of data that was announced? */
                if (handle->transferSize == handle->currentPosition) {
                    /* Flush partially filled buffer */
                    if (_EPHEMUPDATE_flushPage(handle) == LPCLIB_SUCCESS) {
                        /* Store the new data. If it is identical to what we already have, we're done now. */
                        if (!_EPHEMUPDATE_mustUpdate()) {
                            response = EPHEMUPDATE_RESPONSE_OK;
                        }
                        else {
                            /* Otherwise do the final update (overwrite existing ephemeris data) */
                            if (_EPHEMUPDATE_finalizeUpdate(handle) == LPCLIB_SUCCESS) {
                                response = EPHEMUPDATE_RESPONSE_OK;
                            }
                            handle->state = EPHEMUPDATE_STATE_IDLE;

                            /* Update some values that are extracted only once */
                            EPHEMERIS_init();
                        }
                    }
                }
            }
            break;

        case 9:     /* Calculate checksum of existing ephemerides */
            {
                uint32_t checksum = _EPHEMUPDATE_getEphemerisChecksum();
                static char s[20];
                sprintf(s, "%"PRIu32, checksum);
                response = s;
            }
            break;
        }

        _EPHEMUPDATE_sendResponse(action, response);
    }

    return LPCLIB_SUCCESS;
}


static bool _EPHEMUPDATE_checkEvent (EPHEMUPDATE_Handle handle)
{
    (void) handle;

    bool haveEvent = false;

    return haveEvent;
}



PT_THREAD(EPHEMUPDATE_thread (EPHEMUPDATE_Handle handle))
{
    PT_BEGIN(&handle->pt);

//    sysContext.queue = osMailCreate(osMailQ(sysQueueDef), NULL);
//    sysContext.rssiTick = osTimerCreate(osTimer(rssiTimer), osTimerPeriodic, (void *)SYS_TIMERMAGIC_RSSI);
// //    osTimerStart(handle->actionTick, 200);

    while (1) {
        /* Wait for an event */
        PT_WAIT_UNTIL(&handle->pt, _EPHEMUPDATE_checkEvent(handle));
#if 0
        /* Is there a new message? */
        if (handle->rtosEvent.status == osEventMail) {
            pMessage = (SYS_Message *)handle->rtosEvent.value.p;
            switch (pMessage->opcode) {
            case EPHEMUPDATE_OPCODE_xxx:
                break;
            }

            osMailFree(handle->queue, pMessage);
        }
#endif
    }

    PT_END(&handle->pt);
}


