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


#ifndef __SONDE_H
#define __SONDE_H

#include "lpclib.h"
#include "pt.h"

/* Common interface for all sonde types */
typedef enum {
    SONDE_UNDEFINED = 0,
    SONDE_RS41,
    SONDE_RS92,
    SONDE_DFM06,
    SONDE_DFM09,
    SONDE_C34,
    SONDE_C50,
    SONDE_IMET_RSB,
    SONDE_PS15,
    SONDE_M10,
} SONDE_Type;


typedef enum {
    SONDE_DECODER_RS41_RS92 = 0,
    SONDE_DECODER_DFM,
    SONDE_DECODER_C34_C50,
    SONDE_DECODER_IMET,
    SONDE_DECODER_MODEM,
    _SONDE_DECODER_UNDEFINED_,
} SONDE_Decoder;



typedef struct _SONDE_Context *SONDE_Handle;
typedef struct _SONDE_Context {
    SONDE_Type type;                        /* Type of sonde */
    SONDE_Decoder decoder;
    uint32_t frequencyHz;                   /* RX frequency */

    /* Common procedures */
    LPCLIB_Result (*processBlock) (SONDE_Handle handle, void *rxBuffer, uint32_t length);
    size_t (*getPrivateSize) (void);

    /* Access to private data */
    void *private;                          /* Data specific to each sonde type */
} SONDE_Context;


#endif
