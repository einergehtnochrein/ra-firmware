/* Copyright (c) 2015, DF9DQ
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


#ifndef __RS92_H
#define __RS92_H

#include "lpclib.h"
#include "pt.h"



typedef struct RS92_Context *RS92_Handle;
typedef uint8_t RS92_RawData[234];

typedef enum {
    RS92_LOGMODE_NONE = 0,
    RS92_LOGMODE_RAW,
} RS92_LogMode;


LPCLIB_Result RS92_open (RS92_Handle *pHandle);
LPCLIB_Result RS92_setLogMode (RS92_Handle handle, uint32_t id, RS92_LogMode mode);
LPCLIB_Result RS92_processBlock (RS92_Handle handle, void *buffer, uint32_t numBits, float rxFrequencyHz);
LPCLIB_Result RS92_resendLastPositions (RS92_Handle handle);
LPCLIB_Result RS92_removeFromList (RS92_Handle handle, uint32_t id, float *frequency);
LPCLIB_Result RS92_setSatelliteSnrThreshold (RS92_Handle handle, float threshold);

#endif
