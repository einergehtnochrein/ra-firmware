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

#include "lpc54xxx_libconfig.h"

/** \file
 *  \brief MRT driver implementation.
 *
 *  This file contains the implementation of the MRT block driver.
 */


#include "lpc54xxx_clkpwr.h"
#include "lpc54xxx_mrt.h"


LPCLIB_DefineRegBit(MRT_INTVAL_IVALUE,              0,  24);
LPCLIB_DefineRegBit(MRT_INTVAL_LOAD,                31, 1);

LPCLIB_DefineRegBit(MRT_CTRL_INTEN,                 0,  1);
LPCLIB_DefineRegBit(MRT_CTRL_MODE,                  1,  2);

LPCLIB_DefineRegBit(MRT_STAT_INTFLAG,               0,  1);
LPCLIB_DefineRegBit(MRT_STAT_RUN,                   1,  1);
LPCLIB_DefineRegBit(MRT_STAT_INUSE,                 2,  1);

LPCLIB_DefineRegBit(MRT_MODCFG_NOC,                 0,  4);
LPCLIB_DefineRegBit(MRT_MODCFG_NOB,                 4,  5);
LPCLIB_DefineRegBit(MRT_MODCFG_MULTITASK,           31, 1);

LPCLIB_DefineRegBit(MRT_IDLE_CH_CHAN,               4,  4);

LPCLIB_DefineRegBit(MRT_IRQ_FLAG_GFLAG0,            0,  1);
LPCLIB_DefineRegBit(MRT_IRQ_FLAG_GFLAG1,            1,  1);
LPCLIB_DefineRegBit(MRT_IRQ_FLAG_GFLAG2,            2,  1);
LPCLIB_DefineRegBit(MRT_IRQ_FLAG_GFLAG3,            3,  1);


#define MRT_NUM_CHANNELS                    4
#define MRT_MAX_COUNT                       ((1u << 24) - 1)

static struct MRT_Context {
    LPCLIB_Callback callbacks[MRT_NUM_CHANNELS];
} mrtContext;



/* Open the MRT */
LPCLIB_Result MRT_open (MRT_Handle *pHandle)
{
    *pHandle = &mrtContext;

    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_MRT);

    LPC_MRT->MODCFG = 0
            | (1u << MRT_MODCFG_MULTITASK_Pos)
            ;

    return LPCLIB_SUCCESS;
}


/* Close the MRT */
LPCLIB_Result MRT_close (MRT_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    for (int i = 0; i < MRT_NUM_CHANNELS; i++) {
        LPC_MRT->channel[i].INTVAL = 0                  /* Stop and reset timer */
                | (0 << MRT_INTVAL_IVALUE_Pos)
                | (1u << MRT_INTVAL_LOAD_Pos)
                ;
        LPC_MRT->channel[i].CTRL = 0;                   /* Disable interrupt */
    }

    CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_MRT);

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}



/* Adjust timer operating parameters */
LPCLIB_Result MRT_ioctl (MRT_Handle handle, const MRT_Config *pConfig)
{
    (void)handle;

    while (pConfig->opcode != MRT_OPCODE_INVALID) {
        switch (pConfig->opcode) {
            case MRT_OPCODE_INVALID:
                break;
        }

        ++pConfig;
    }

    return LPCLIB_SUCCESS;
}



/* Stop a channel without further action. */
LPCLIB_Result MRT_stopChannel (
        MRT_Handle handle,
        uint8_t channel)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (channel < MRT_NUM_CHANNELS) {
        LPC_MRT->channel[channel].CTRL &= ~MRT_CTRL_INTEN_Msk;
        LPC_MRT->channel[channel].INTVAL = 0
                | (1u << MRT_INTVAL_LOAD_Pos)
                ;

        LPC_MRT->channel[channel].STAT = 0
                | (1u << MRT_STAT_INTFLAG_Pos)
                ;
        LPC_MRT->channel[channel].STAT = 0
                | (1u << MRT_STAT_INUSE_Pos)
                ;
    }

    return LPCLIB_SUCCESS;
}


LPCLIB_Result MRT_oneshot_millisecs (
        MRT_Handle handle,
        uint32_t millisecs,
        uint8_t *pChannel,
        LPCLIB_Callback callback)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Convert milliseconds into timer ticks */
    uint32_t ticks = millisecs * (SystemCoreClock / 1000);
    if (ticks > MRT_MAX_COUNT) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    uint32_t channel = (LPC_MRT->IDLE_CH & MRT_IDLE_CH_CHAN_Msk) >> MRT_IDLE_CH_CHAN_Pos;
    *pChannel = channel;
    if (channel < MRT_NUM_CHANNELS) {
        handle->callbacks[channel] = callback;

        LPC_MRT->channel[channel].CTRL = 0
                | (1u << MRT_CTRL_INTEN_Pos)
                | (MRT_MODE_ONESHOT << MRT_CTRL_MODE_Pos)
                ;

        LPC_MRT->channel[channel].INTVAL = 0
                | (ticks << MRT_INTVAL_IVALUE_Pos)
                | (1u << MRT_INTVAL_LOAD_Pos)
                ;

        if (callback == NULL) {
            while (LPC_MRT->channel[channel].STAT & MRT_STAT_RUN_Msk)
                ;
        }
    }

    return LPCLIB_BUSY;
}


/** MRT interrupt handler.
 */
void MRT_IRQHandler (void)
{
    MRT_Handle handle = &mrtContext;
    uint32_t ir_reg = LPC_MRT->IRQ_FLAG;
    LPCLIB_Event event;
    int i;


    for (i = 0; i < MRT_NUM_CHANNELS; i++) {
        /* Timeout of this channel? */
        if (ir_reg & (1u << (MRT_IRQ_FLAG_GFLAG0_Pos + i))) {
            /* Interrupt enabled for this channel? */
            if (LPC_MRT->channel[i].CTRL & MRT_CTRL_INTEN_Msk) {
                LPC_MRT->channel[i].STAT = 0
                        | (1u << MRT_STAT_INTFLAG_Pos)
                        ;

                /* Callback? */
                if (handle->callbacks[i]) {
                    event.id = LPCLIB_EVENTID_MRT;
                    event.opcode = MRT_EVENT_TICK;
                    event.block = i;
                    handle->callbacks[i](event);
                }
            }
        }
    }
}


/** @} */

