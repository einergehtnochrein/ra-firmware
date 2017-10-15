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

#include <string.h>

#include "lpclib_types.h"
#include "lpc54xxx_clkpwr.h"
#include "lpc54xxx_iap.h"




/* Definitions for IAP calls. */

typedef uint32_t IAP_COMMAND[5];
typedef uint32_t IAP_RESULT[5];

/* IAP entry */
typedef void (* IAP)(IAP_COMMAND command, IAP_RESULT result);
#define IAP_ENTRY(command,result) ((IAP)0x03000205)(command,result)

enum {
    IAP_OPCODE_PREPARE_SECTORS_FOR_WRITE_OPERATION = 50,
    IAP_OPCODE_COPY_RAM_TO_FLASH = 51,
    IAP_OPCODE_ERASE_SECTORS = 52,
    IAP_OPCODE_BLANK_CHECK_SECTORS = 53,
    IAP_OPCODE_READ_PART_ID = 54,
    IAP_OPCODE_READ_BOOT_CODE_VERSION = 55,
    IAP_OPCODE_COMPARE = 56,
    IAP_OPCODE_REINVOKE_ISP = 57,
    IAP_OPCODE_READ_DEVICE_SERIAL_NUMBER = 58,
    IAP_OPCODE_ERASE_PAGES = 59,
    IAP_OPCODE_READ_SIGNATURE = 70,
};



/* Read part ID. */
LPCLIB_Result IAP_readPartId (uint32_t *pId)
{
    IAP_COMMAND command;
    IAP_RESULT result;

    if (pId == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    command[0] = IAP_OPCODE_READ_PART_ID;
    IAP_ENTRY(command, result);

    *pId = result[1];

    return LPCLIB_SUCCESS;
}



/* Read boot code version. */
LPCLIB_Result IAP_readBootCodeVersion (uint8_t *pMajor, uint8_t *pMinor)
{
    IAP_COMMAND command;
    IAP_RESULT result;

    if ((pMajor == NULL) || (pMinor == NULL)) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    command[0] = IAP_OPCODE_READ_BOOT_CODE_VERSION;
    IAP_ENTRY(command, result);

    *pMajor = (result[1] >> 8) & 0xFF;
    *pMinor = (result[1] >> 0) & 0xFF;

    return LPCLIB_SUCCESS;
}



/* Read serial number. */
LPCLIB_Result IAP_readDeviceSerialNumber (IAP_SerialNumber *pSerialNumber)
{
    IAP_COMMAND command;
    IAP_RESULT result;

    if (pSerialNumber == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    command[0] = IAP_OPCODE_READ_DEVICE_SERIAL_NUMBER;
    IAP_ENTRY(command, result);

    pSerialNumber->number[0] = result[1];
    pSerialNumber->number[1] = result[2];
    pSerialNumber->number[2] = result[3];
    pSerialNumber->number[3] = result[4];

    return LPCLIB_SUCCESS;
}



/* Prepare sectors for write operation (erase or program). */
LPCLIB_Result IAP_prepareSectorsForWriteOperation (uint32_t firstSector,
                                                   uint32_t lastSector,
                                                   IAP_StatusCode *pExtendedStatus)
{
    IAP_COMMAND command;
    IAP_RESULT result;

    command[0] = IAP_OPCODE_PREPARE_SECTORS_FOR_WRITE_OPERATION;
    command[1] = firstSector;
    command[2] = lastSector;
    IAP_ENTRY(command, result);

    if (pExtendedStatus) {
        *pExtendedStatus = (IAP_StatusCode)result[0];
    }

    return (result[0] == IAP_STATUS_CMD_SUCCESS) ? LPCLIB_SUCCESS : LPCLIB_ERROR;
}



/* Erase sector range. */
LPCLIB_Result IAP_eraseSectors (uint32_t firstSector,
                                uint32_t lastSector,
                                IAP_StatusCode *pExtendedStatus)
{
    IAP_COMMAND command;
    IAP_RESULT result;

    command[0] = IAP_OPCODE_ERASE_SECTORS;
    command[1] = firstSector;
    command[2] = lastSector;
    command[3] = SystemCoreClock / 1024;    /* Dividing by 1024 is only 2% off,
                                             * but uses shift operation only!
                                             */
    IAP_ENTRY(command, result);

    if (pExtendedStatus) {
        *pExtendedStatus = (IAP_StatusCode)result[0];
    }

    return (result[0] == IAP_STATUS_CMD_SUCCESS) ? LPCLIB_SUCCESS : LPCLIB_ERROR;
}



/* Erase page range. */
LPCLIB_Result IAP_erasePages (uint32_t firstPage,
                              uint32_t lastPage,
                              IAP_StatusCode *pExtendedStatus)
{
    IAP_COMMAND command;
    IAP_RESULT result;

    command[0] = IAP_OPCODE_ERASE_PAGES;
    command[1] = firstPage;
    command[2] = lastPage;
    command[3] = SystemCoreClock / 1024;    /* Dividing by 1024 is only 2% off,
                                             * but uses shift operation only!
                                             */
    IAP_ENTRY(command, result);

    if (pExtendedStatus) {
        *pExtendedStatus = (IAP_StatusCode)result[0];
    }

    return (result[0] == IAP_STATUS_CMD_SUCCESS) ? LPCLIB_SUCCESS : LPCLIB_ERROR;
}



/* Copy RAM to flash. */
LPCLIB_Result IAP_copyRamToFlash (uint32_t toAddress,
                                  uint32_t fromAddress,
                                  uint32_t length,
                                  IAP_StatusCode *pExtendedStatus)
{
    IAP_COMMAND command;
    IAP_RESULT result;

    command[0] = IAP_OPCODE_COPY_RAM_TO_FLASH;
    command[1] = toAddress;
    command[2] = fromAddress;
    command[3] = length;
    command[4] = SystemCoreClock / 1024;    /* Dividing by 1024 is only 2% off,
                                             * but uses shift operation only!
                                             */
    IAP_ENTRY(command, result);

    if (pExtendedStatus) {
        *pExtendedStatus = (IAP_StatusCode)result[0];
    }

    return (result[0] == IAP_STATUS_CMD_SUCCESS) ? LPCLIB_SUCCESS : LPCLIB_ERROR;
}


/* Return the sector number and sector size for a given address. */
LPCLIB_Result IAP_address2SectorNumber (uint32_t address,
                                        uint32_t *pSectorNumber,
                                        uint32_t *pSectorSizeBytes)
{
    /* Sort out illegal address */
    if (address >= 0x80000) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Validate pointers */
    if (!pSectorNumber) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Uniform 32 KiB sector layout starting at address 0 */
    *pSectorNumber = address / 32768;
    if (pSectorSizeBytes) {
        *pSectorSizeBytes = 32768;
    }

    return LPCLIB_SUCCESS;
}


/* Return the page number and page size for a given address. */
LPCLIB_Result IAP_address2PageNumber (uint32_t address,
                                      uint32_t *pPageNumber,
                                      uint32_t *pPageSizeBytes)
{
    /* Sort out illegal address */
    if (address >= 0x80000) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Validate pointers */
    if (!pPageNumber) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Uniform 256 Bytes page layout starting at address 0 */
    *pPageNumber = address / 256;
    if (pPageSizeBytes) {
        *pPageSizeBytes = 256;
    }

    return LPCLIB_SUCCESS;
}



