/* Copyright (c) 2017, DF9DQ
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


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"

#include "app.h"
#include "pdm.h"


LPCLIB_DefineRegBit(DMIC_CHANEN_EN_CH0,             0,  1);
LPCLIB_DefineRegBit(DMIC_CHANEN_EN_CH1,             1,  1);


#define PDM_SAMPLES_PER_BUFFER              256
#define PDM_N_BUFFERS                       3

static struct _PDM_Context {
    PDM_Callback callback;

    int32_t audioBuffer[PDM_N_BUFFERS][PDM_SAMPLES_PER_BUFFER];
    int activeBufferIndex;
    int completedBufferIndex;
    int bufferWrIndex;

    float dcOffset;
} pdmContext;



void PDM_open (int dummy, PDM_Handle *pHandle)
{
    (void) dummy;

    *pHandle = &pdmContext;

    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_DMIC);
}


void PDM_run (PDM_Handle handle, int decimationRatio, PDM_Callback callback)
{
    (void) decimationRatio;

    if (handle == NULL) {
        return;
    }

    NVIC_DisableIRQ(DMIC_IRQn);

    handle->callback = callback;

    /* Sample value range after CIC filter is determined by decimation ratio R: +/-(R^5) / 256 + 20%,
     * where the 20% represent the maximum gain of the FIR passband correction filter.
     * For R=50, the range is < +/-1.5e6, which requires a 21.5 bit representation.
     * In order to use the saturation feature of the DC filter (16 bits), we use the DC filter
     * scaler to right-shift and reach 16 bits.
     * We shift by 4 bits and therefore allow clipping in the upper +/-0.75 bits (4.5 dB)
     */
    /* NOTE: We use 1FS mode (CIC decimates down to 4FS), and so we should use PREAC4FSCOEF0 to
     * configure the FIR filter. However, it seems that PREAC2FSCOEF0 is always used, independent of
     * the 1FS/2FS selection.
     */

//TODO hardware independent...
    LPC_DMIC->OSR0 = decimationRatio / 4;
    LPC_DMIC->DIVHFCLK0 = 0;
    LPC_DMIC->PREAC2FSCOEF0 = 3;            /* See NOTE above */
    LPC_DMIC->GAINSHIFT0 = (0 & 0x3F);
//    LPC_DMIC->FIFOCTRL0 = 0x00070007;   // Trigger level = 8
LPC_DMIC->FIFOCTRL0 = 0x00080007;   // Trigger level = 9
    LPC_DMIC->PDMSRCCFG0 = 0x0001;   // falling edge sampling
    LPC_DMIC->DCCTRL0 = 0
//            | (2 << 0)                      /* 78 Hz */
    | (0 << 0)                      /* No DC filter */
            | (4 << 4)                      /* 21.5 bits --> 17.5 bits */
            | (1 << 8)                      /* Saturation on (16 bits) */
            ;

    LPC_DMIC->IOCFG = 0;
    LPC_DMIC->USE2FS = 0;                   /* 1FS mode */
    LPC_DMIC->CHANEN = (1u << DMIC_CHANEN_EN_CH0_Pos);

    NVIC_EnableIRQ(DMIC_IRQn);
    NVIC_SetPriority(DMIC_IRQn, 3);
    NVIC_EnableIRQ(IRQ38_IRQn);
}


void PDM_stop (PDM_Handle handle)
{
    (void)handle;

    NVIC_DisableIRQ(IRQ38_IRQn);
    NVIC_DisableIRQ(DMIC_IRQn);
}


void PDM_close (PDM_Handle *pHandle)
{
    if (pHandle == NULL) {
        return;
    }

    if (*pHandle == NULL) {
        return;
    }

    PDM_stop(*pHandle);

    CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_DMIC);

    *pHandle = NULL;
}


/* Get DC offset (Ra2 only) */
float PDM_getDcOffset (PDM_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return NAN;
    }

    return handle->dcOffset;
}


void DMIC_IRQHandler (void)
{
    PDM_Handle handle = &pdmContext;
    int i;
    uint32_t status;
    int readBuffer4dc;

    status = LPC_DMIC->FIFOSTAT0;

    if (status & 1) {
        /* DC averaging is over one buffer length */
        readBuffer4dc = (handle->activeBufferIndex + PDM_N_BUFFERS - 1) % PDM_N_BUFFERS;

        /* Get samples from FIFO */
        for (i = 0; i < 8; i++) {  //TODO
            if (handle->bufferWrIndex < PDM_SAMPLES_PER_BUFFER) {
                /* The FIFO entries contain signed 24-bit values in [23:0].
                 * Only [15:0] are relevant since we use the saturation feature of the DC filter.
                 * Bits [31:25] always read as 0. FIFO data is therefore shifted left by 8 bits
                 * to create a 32-bit signed value (int32_t), which is then scaled down to 16 bits.
                 */
                handle->audioBuffer[handle->activeBufferIndex][handle->bufferWrIndex] =
                    (int32_t)(LPC_DMIC->FIFODATA0 << 8) / 256;

                /* DC filter (moving average of last PDM_SAMPLES_PER_BUFFER samples */
                float f = handle->audioBuffer[handle->activeBufferIndex][handle->bufferWrIndex]
                        - handle->audioBuffer[readBuffer4dc][handle->bufferWrIndex];
                handle->dcOffset = handle->dcOffset - f / PDM_SAMPLES_PER_BUFFER;
//    DC[n] = DC[n-1] - (1/N)*(x[n-N] + x[n])
            }
            ++handle->bufferWrIndex;
        }

        if (handle->bufferWrIndex >= PDM_SAMPLES_PER_BUFFER) {
            handle->bufferWrIndex = 0;

            handle->completedBufferIndex = handle->activeBufferIndex;
            handle->activeBufferIndex = (handle->activeBufferIndex + 1) % PDM_N_BUFFERS;

            /* Trigger deferred handling (IRQ with lower priority) */
            NVIC_SetPendingIRQ(IRQ38_IRQn);
        }
    }

    LPC_DMIC->FIFOSTAT0 = status & 7;
}



void IRQ38_IRQHandler (void);
void IRQ38_IRQHandler (void)
{
    PDM_Handle handle = &pdmContext;

    if (handle->callback) {
        handle->callback(handle->audioBuffer[handle->completedBufferIndex], PDM_SAMPLES_PER_BUFFER);
    }
}

