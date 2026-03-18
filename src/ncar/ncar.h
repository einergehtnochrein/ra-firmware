/* Copyright (c) 2025, DF9DQ
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


#ifndef __NCAR_H
#define __NCAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lpclib.h"
#include "pt.h"



typedef struct NCAR_Context *NCAR_Handle;

typedef enum {
    NCAR_LOGMODE_NONE = 0,
    NCAR_LOGMODE_RAW,
} NCAR_LogMode;


LPCLIB_Result NCAR_open (NCAR_Handle *pHandle);
LPCLIB_Result NCAR_setLogMode (NCAR_Handle handle, uint32_t id, NCAR_LogMode mode);
LPCLIB_Result NCAR_processBlock (
        NCAR_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime);
LPCLIB_Result NCAR_resendLastPositions (NCAR_Handle handle);
LPCLIB_Result NCAR_removeFromList (NCAR_Handle handle, uint32_t id, float *frequency);


#ifdef __cplusplus
}
#endif
#endif
