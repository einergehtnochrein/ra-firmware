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
 *  \brief FLASH/EEPROM driver interface.
 *  This file defines all interface objects needed to use the FLASH/EEPROM driver.
 */


#ifndef __LPC54XXX_IAP_H__
#define __LPC54XXX_IAP_H__

/** \defgroup IAP
 *  \ingroup API
 *  @{
 */

#include "lpc54xxx_libconfig.h"

#include "lpclib_types.h"


/** \defgroup IAP_Public_Types IAP Types, enums, macros
 *  @{
 */


/** Native status codes returned by IAP ROM routines. */
typedef enum IAP_StatusCode {
    IAP_STATUS_CMD_SUCCESS = 0,
    IAP_STATUS_INVALID_COMMAND = 1,
    IAP_STATUS_SRC_ADDR_ERROR = 2,
    IAP_STATUS_DST_ADDR_ERROR = 3,
    IAP_STATUS_SRC_ADDR_NOT_MAPPED = 4,
    IAP_STATUS_DST_ADDR_NOT_MAPPED = 5,
    IAP_STATUS_COUNT_ERROR = 6,
    IAP_STATUS_INVALID_SECTOR = 7,
    IAP_STATUS_SECTOR_NOT_BLANK = 8,
    IAP_STATUS_SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION = 9,
    IAP_STATUS_COMPARE_ERROR = 10,
    IAP_STATUS_BUSY = 11,
} IAP_StatusCode;

/** Unique serial number. */
typedef struct IAP_SerialNumber {
    uint32_t number[4];
} IAP_SerialNumber;

/** @} IAP Types, enums, macros */



/** \defgroup IAP_Public_Functions IAP API Functions
 *  @{
 */



/** Read part ID.
 */
LPCLIB_Result IAP_readPartId (uint32_t *pId);


/** Read boot code version.
 */
LPCLIB_Result IAP_readBootCodeVersion (uint8_t *pMajor, uint8_t *pMinor);


/** Read serial number.
 */
LPCLIB_Result IAP_readDeviceSerialNumber (IAP_SerialNumber *pSerialNumber);


/** Prepare sectors for write operation (erase or program).
 *
 *  \param[in] firstSector
 *  \param[in] lastSector
 *  \param[out] pExtendedStatus
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ERROR Detailed error specified in extended status code
 */
LPCLIB_Result IAP_prepareSectorsForWriteOperation (uint32_t firstSector,
                                                   uint32_t lastSector,
                                                   IAP_StatusCode *pExtendedStatus);


/** Erase sector range.
 *
 *  \param[in] firstSector
 *  \param[in] lastSector
 *  \param[out] pExtendedStatus
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ERROR Detailed error specified in extended status code
 */
LPCLIB_Result IAP_eraseSectors (uint32_t firstSector,
                                uint32_t lastSector,
                                IAP_StatusCode *pExtendedStatus);


/** Erase page range.
 *
 *  \param[in] firstPage
 *  \param[in] lastPage
 *  \param[out] pExtendedStatus
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ERROR Detailed error specified in extended status code
 */
LPCLIB_Result IAP_erasePages (uint32_t firstPage,
                              uint32_t lastPage,
                              IAP_StatusCode *pExtendedStatus);


/** Copy RAM to flash.
 *
 *  \param[in] toAddress
 *  \param[in] fromAddress
 *  \param[in] length
 *  \param[out] pExtendedStatus
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ERROR Detailed error specified in extended status code
 */
LPCLIB_Result IAP_copyRamToFlash (uint32_t toAddress,
                                  uint32_t fromAddress,
                                  uint32_t length,
                                  IAP_StatusCode *pExtendedStatus);


/** Return the sector number for a given address.
 *
 *  \param[in] address
 *  \param[out] pSectorNumber
 *  \param[out] pSectorSizeBytes (optional)
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Address outside flash area
 */
LPCLIB_Result IAP_address2SectorNumber (uint32_t address,
                                        uint32_t *pSectorNumber,
                                        uint32_t *pSectorSizeBytes);


/** Return the page number for a given address.
 *
 *  \param[in] address
 *  \param[out] pPageNumber
 *  \param[out] pPageSizeBytes (optional)
 *  \retval LPCLIB_SUCCESS ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Address outside flash area
 */
LPCLIB_Result IAP_address2PageNumber (uint32_t address,
                                      uint32_t *pPageNumber,
                                      uint32_t *pPageSizeBytes);


/** @} IAP API Functions */

/** @} IAP */

#endif /* #ifndef __LPC54XXX_IAP_H__ */

