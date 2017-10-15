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
 *  \brief FLEXCOMM driver interface.
 */


#ifndef __LPC54XXX_FLEXCOMM_H__
#define __LPC54XXX_FLEXCOMM_H__

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X

/** \defgroup FLEXCOMM
 *  \ingroup API
 *  @{
 */

#include "lpc54xxx_libconfig.h"

#include "lpclib_types.h"


/** \defgroup FLEXCOMM_Public_Types FLEXCOMM Types, enums, macros
 *  @{
 */

/** Enumerator for FLEXCOMM names. */
typedef enum FLEXCOMM_Name {
    FLEXCOMM0 = 0,
    FLEXCOMM1 = 1,
    FLEXCOMM2 = 2,
    FLEXCOMM3 = 3,
    FLEXCOMM4 = 4,
    FLEXCOMM5 = 5,
    FLEXCOMM6 = 6,
    FLEXCOMM7 = 7,
} FLEXCOMM_Name;

#define FLEXCOMM_COUNT                      8


/* Type of IRQ handler */
typedef void (*FLEXCOMM_IRQHandler_t)(FLEXCOMM_Name flexcomm);


/** @} FLEXCOMM Types, enums, macros */



/** \defgroup FLEXCOMM_Public_Functions FLEXCOMM API Functions
 *  @{
 */


/** Install an interrupt handler for a FLEXCOMM interface
 *
 *  \param[in] flexcomm The number of the FLEXCOMM interface
 *  \param[in] IRQHandler Pointer to IRQ handler function
 *  \return Bus clock in Hz
 */
void FLEXCOMM_installHandler (FLEXCOMM_Name flexcomm, FLEXCOMM_IRQHandler_t handler);


/** @} FLEXCOMM API Functions */

/** @} FLEXCOMM */

#endif

#endif /* #ifndef __LPC54XXX_FLEXCOMM_H__ */

