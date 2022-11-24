/* Copyright (c) 2015-2019, DF9DQ
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


#ifndef __SONDE_H
#define __SONDE_H

#include "lpclib.h"
#include "pt.h"

/* Common interface for all sonde types */
typedef enum {
    SONDE_UNDEFINED = 0,
    SONDE_RS41,
    SONDE_RS92,
    SONDE_DFM_NORMAL,
    SONDE_DFM_INVERTED,
    SONDE_C34,
    SONDE_C50,
    SONDE_IMET_RSB,
    SONDE_M10,
    SONDE_M20,
    SONDE_BEACON,
    SONDE_PILOT,
    SONDE_MEISEI_CONFIG,
    SONDE_MEISEI_GPS,
    SONDE_RSG20,
    SONDE_MRZ,
    SONDE_GTH3_CF06AH,
    SONDE_PSB3,
    SONDE_IMET54,
} SONDE_Type;


/* Decoder: Group of sondes that can be decoded the same way */
typedef enum {
    SONDE_DECODER_RS41 = 0,
    SONDE_DECODER_RS92 = 1,
    SONDE_DECODER_DFM = 2,
    SONDE_DECODER_C34_C50 = 3,
    SONDE_DECODER_IMET = 4,
    SONDE_DECODER_MODEM = 5,
    SONDE_DECODER_BEACON = 6,
    SONDE_DECODER_PILOT = 7,
    SONDE_DECODER_MEISEI = 8,
    SONDE_DECODER_JINYANG = 9,
    SONDE_DECODER_IMET54 = 10,
    SONDE_DECODER_MRZ = 11,
    SONDE_DECODER_CF06 = 12,
    SONDE_DECODER_GTH3 = 13,
    SONDE_DECODER_PSB3 = 14,
    _SONDE_DECODER_UNDEFINED_ = -1,
} SONDE_Decoder;


/* Detector: Types of sondes the receiver can detect in parallel */
typedef enum {
    SONDE_DETECTOR_RS41_RS92 = 0,
    SONDE_DETECTOR_DFM = 1,
    SONDE_DETECTOR_C34_C50 = 2,
    SONDE_DETECTOR_IMET = 3,
    SONDE_DETECTOR_MODEM = 4,
    SONDE_DETECTOR_BEACON = 5,
    SONDE_DETECTOR_MEISEI = 6,
    SONDE_DETECTOR_PILOT = 7,
    SONDE_DETECTOR_JINYANG = 8,
    SONDE_DETECTOR_IMET54 = 9,
    SONDE_DETECTOR_MRZ = 10,
    SONDE_DETECTOR_ASIA1 = 11,
    SONDE_DETECTOR_PSB3 = 12,
    _SONDE_DETECTOR_UNDEFINED_ = -1,
} SONDE_Detector;


typedef struct _SONDE_Context *SONDE_Handle;

LPCLIB_Result SONDE_open (SONDE_Handle *pHandle);
LPCLIB_Result SONDE_initID (SONDE_Handle handle, uint32_t startID);
uint32_t SONDE_getNewID (SONDE_Handle handle);

#endif
