/* Copyright (c) 2014, NXP Semiconductors
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
 *  \brief ROM Firmware library interface.
 *  This file defines helper objects to access the drivers in ROM.
 */


#ifndef __LPC54XXX_ROM_H__
#define __LPC54XXX_ROM_H__

/** \defgroup ROM
 *  \ingroup API
 *  @{
 */

#include "lpc54xxx_libconfig.h"

#include "lpclib_types.h"



/** \defgroup ROM_Public_Types ROM Types, enums, macros
 *  @{
 */


struct PWRD;
struct I2CD;
struct DMAD;
struct SPID;
struct ADCD;
struct UARTD;


/** ROM driver table */
typedef struct ROM_Table {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    void *reserved0;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    struct USBD_API *pUsbd;
#endif
    void *reserved1;
    void *reserved2;
    struct PWRD *pPwrd;                     /**< Power Profiles driver */
    void *reserved4;
    struct I2CD *pI2cd;                     /**< I2C driver */
    struct DMAD *pDmad;                     /**< DMA driver */
    struct SPID *pSpid;                     /**< SPI driver */
    struct ADCD *pAdcd;                     /**< ADC driver */
    struct UARTD *pUsartd;                  /**< USART driver */
    void *reserved10;
    void *reserved11;
} ROM_Table;


#define pRom ((const ROM_Table **)0x03000200)[0]


/** ROM Driver Error Codes. */
typedef enum ROM_ErrorCode {
    ROM_ERR_SUCCESS                             = 0,
    ROM_ERR_GENERAL                             = 1,

    ROM_ERR_I2C_NAK                             = 0x00060001,
    ROM_ERR_I2C_BUFFER_OVERFLOW                 = 0x00060002,
    ROM_ERR_I2C_BYTE_COUNT_ERR                  = 0x00060003,
    ROM_ERR_I2C_LOSS_OF_ARBITRATION             = 0x00060004,
    ROM_ERR_I2C_SLAVE_NOT_ADDRESSED             = 0x00060005,
    ROM_ERR_I2C_LOSS_OF_ARBITRATION_NAK_BIT     = 0x00060006,
    ROM_ERR_I2C_GENERAL_FAILURE                 = 0x00060007,
    ROM_ERR_I2C_REGS_SET_TO_DEFAULT             = 0x00060008,
} ROM_ErrorCode;


/** @} ROM Types, enums, macros */



/** \defgroup ROM_Public_Functions ROM API Functions
 *  @{
 */

/** @} ROM API Functions */

/** @} ROM */

#endif /* #ifndef __LPC54XXX_ROM_H__ */

