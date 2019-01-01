/* Copyright (c) 2018, DF9DQ
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
 *  \brief MRT driver interface.
 *
 *  This file contains the API for the MRT block.
 */

#ifndef __LPC54XXX_MRT_H__
#define __LPC54XXX_MRT_H__

#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup MRT
 *  \ingroup API
 *  @{
 */

#include "lpc54xxx_libconfig.h"
#include "lpclib_types.h"


/** \defgroup MRT_Public_Types MRT Types, enums, macros
 *  @{
 */


/** Handle for a timer block */
typedef struct MRT_Context *MRT_Handle;


typedef enum MRT_Mode {
    MRT_MODE_REPETITIVE = 0,                /**< Mode: Repetitive */
    MRT_MODE_ONESHOT = 1,                   /**< Mode: One-shot (normal) */
    MRT_MODE_ONESHOT_STALL = 2,             /**< Mode: One-shot (with CPU stall) */
} MRT_Mode;


typedef enum MRT_Opcode {
    MRT_OPCODE_INVALID = 0,                 /**< (List terminator) */
} MRT_Opcode;


typedef struct MRT_Config {
    MRT_Opcode opcode;                      /**< Control action */
} MRT_Config;

/** Config list terminator. */
#define MRT_CONFIG_END \
    {.opcode = MRT_OPCODE_INVALID}



typedef enum MRT_Event {
    MRT_EVENT_TICK = 1,                     /**< Timer tick event has occurred */
} MRT_Event;


/** @} MRT types, enums, macros */


/** \defgroup MRT_Public_Functions MRT API Functions
 *  @{
 */


/** Open a Timer block for use.
 *
 *  \param[out] pHandle Handle to be used in future API calls to the MRT module.
 *  \retval LPCLIB_SUCCESS Access to timer module granted.
 *  \retval LPCLIB_BUSY Timer is in use
 */
LPCLIB_Result MRT_open (MRT_Handle *pHandle);


/** Close the MRT module.
 *
 *  \param[in] pHandle MRT handle
 *  \retval LPCLIB_SUCCESS Ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Invalid handle, or timer not open.
 */
LPCLIB_Result MRT_close (MRT_Handle *pHandle);


/** Adjust the configuration of the timer block.
 *
 *  \param[in] handle Timer handle
 *  \param[in] pConfig Describes the configuration change
 */
LPCLIB_Result MRT_ioctl (MRT_Handle handle, const MRT_Config *pConfig);


/** Stop a channel without further action.
 *
 *  \param[in] handle Timer handle
 */
LPCLIB_Result MRT_stopChannel (
        MRT_Handle handle,
        uint8_t channel);
                                  

/** Start a one-shot timer for multiples of a millisecond).
 *
 *  \param[in] handle Timer handle
 *  \retval LPCLIB_SUCCESS Time has elapsed (this was a blocking call)
 *  \retval LPCLIB_PENDING MR channel started. Callback will follow.
 *  \retval LPCLIB_ILLEGAL_PARAMETER Time value too long for MRT.
 *  \retval LPCLIB_BUSY All MRT channels in use.
 */
LPCLIB_Result MRT_oneshot_millisecs (
        MRT_Handle handle,
        uint32_t millisecs,
        uint8_t *pChannel,
        LPCLIB_Callback callback);
                                  

/** @} MRT API Functions */

/** @} MRT */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef __LPC54XXX_MRT_H__ */

