/* Copyright (c) 2015, DF9DQ
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
 *  \brief TIMER driver implementation.
 *
 *  This file contains the implementation of the TIMER block driver.
 */


/** \addtogroup TIMER
 *  @{
 */


#include "lpc54xxx_timer.h"
#include "lpc54xxx_clkpwr.h"


static struct TIMER_Context {
    LPC_TIMER_Type *hardware;               /** MUST be the first element in this structure. */
    TIMER_Name timer;
    LPCLIB_Switch open;
    LPCLIB_Callback callback;
} timerContext[TIMER_NUM_TIMERS];


static LPC_TIMER_Type * const timerPtr[TIMER_NUM_TIMERS] =
    {LPC_TIMER0,
     LPC_TIMER1,
     LPC_TIMER2,
     LPC_TIMER3,
     LPC_TIMER4};
static const CLKPWR_Clockswitch timerClockswitch[TIMER_NUM_TIMERS] =
    {CLKPWR_CLOCKSWITCH_TIMER0,
     CLKPWR_CLOCKSWITCH_TIMER1,
     CLKPWR_CLOCKSWITCH_TIMER2,
     CLKPWR_CLOCKSWITCH_TIMER3,
     CLKPWR_CLOCKSWITCH_TIMER4};




/* Open a timer channel */
LPCLIB_Result TIMER_open (TIMER_Name timer, TIMER_Handle *pHandle)
{
    if (timerContext[timer].open) {
        return LPCLIB_BUSY;
    }

    timerContext[timer].hardware = timerPtr[timer];
    timerContext[timer].open = ENABLE;
    *pHandle = &timerContext[timer];

    CLKPWR_enableClock(timerClockswitch[timer]);

    timerPtr[timer]->TCR = 2;                           /* Stop and reset the timer */
    timerPtr[timer]->MCR = 0;                           /* No match interrupts */
    timerPtr[timer]->CCR = 0;                           /* No capture interrupts */
    timerPtr[timer]->IR = 0xFF;                         /* Acknowledge pending interrupt requests */

    return LPCLIB_SUCCESS;
}


/* Close a timer channel */
LPCLIB_Result TIMER_close (TIMER_Handle *pTimer)
{
    if (*pTimer == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (!(*pTimer)->open) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    (*pTimer)->hardware->TCR = 0;                       /* Stop */
    (*pTimer)->hardware->MCR = 0;                       /* No match interrupts */
    (*pTimer)->hardware->CCR = 0;                       /* No capture interrupts */
    (*pTimer)->hardware->EMR = 0;                       /* No match outputs */
    (*pTimer)->hardware->IR = 0xFF;                     /* Acknowledge pending interrupt requests */

    CLKPWR_disableClock(timerClockswitch[(*pTimer)->timer]);

    (*pTimer)->open = DISABLE;

    *pTimer = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}



/* Adjust timer operating parameters */
LPCLIB_Result TIMER_ioctl (TIMER_Handle handle, const TIMER_Config *pConfig)
{
    uint32_t value;
    uint32_t mask;

    while (pConfig->opcode != TIMER_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case TIMER_OPCODE_CONFIG_MATCH:
            value = (pConfig->match.intOnMatch << 0) |
                    (pConfig->match.resetOnMatch << 1) |
                    (pConfig->match.stopOnMatch << 2);
            value <<= (3*pConfig->match.channel);
            mask = 7u << (3*pConfig->match.channel);
            handle->hardware->MCR = (handle->hardware->MCR & ~mask) | value;

            value = pConfig->match.function << (4 + 2*pConfig->match.channel);
            mask = 3u << (4 + 2*pConfig->match.channel);
            handle->hardware->EMR = (handle->hardware->EMR & ~mask) | value;
            break;

        case TIMER_OPCODE_CONFIG_CAPTURE:
            value = (pConfig->capture.risingEdge << 0) |
                    (pConfig->capture.fallingEdge << 1) |
                    (pConfig->capture.intOnCapture << 2);
            value <<= (3*pConfig->capture.channel);
            mask = 7u << (3*pConfig->capture.channel);
            handle->hardware->CCR = (handle->hardware->CCR & ~mask) | value;
            break;

        case TIMER_OPCODE_MODE:
            handle->hardware->CTCR = (pConfig->mode.mode << 0) |
                                     (pConfig->mode.channel << 2);
            break;

        case TIMER_OPCODE_SET_CALLBACK:
            handle->callback = pConfig->callback;
            break;

        case TIMER_OPCODE_INVALID:
            break;
        }

        ++pConfig;
    }

    return LPCLIB_SUCCESS;
}



/** Common (for all Timer blocks) interrupt handler.
 *
 *  \param[in] timer Indicator that selects a Timer block
 */
static void TIMER_commonIRQHandler (TIMER_Name timer)
{
    TIMER_Handle handle = &timerContext[timer];
    uint32_t ir_reg = handle->hardware->IR;
    LPCLIB_Event event;
    int i;


    if (handle->callback) {
        /* Callback for each event */
        event.id = LPCLIB_EVENTID_TIMER;
        event.opcode = TIMER_EVENT_MATCH;
        event.block = timer;
        for (i = 0; i < 4; i++) {                   /* Check the four match channels */
            if (ir_reg & (1u << i)) {
                event.channel = i;
                handle->callback(event);
            }
        }

        event.opcode = TIMER_EVENT_CAPTURE;
        for (i = 0; i < 1; i++) {                   /* Check the capture channel */
            if (ir_reg & (0x10u << i)) {
                event.channel = i;
                event.parameter = (void *)(handle->hardware->CR[i]);
                handle->callback(event);
            }
        }
    }

    handle->hardware->IR = ir_reg;                      /* Acknowledge interrupts */
}


void TIMER0_IRQHandler (void)
{
    TIMER_commonIRQHandler(TIMER0);
}

void TIMER1_IRQHandler (void)
{
    TIMER_commonIRQHandler(TIMER1);
}

void TIMER2_IRQHandler (void)
{
    TIMER_commonIRQHandler(TIMER2);
}

void TIMER3_IRQHandler (void)
{
    TIMER_commonIRQHandler(TIMER3);
}

void TIMER4_IRQHandler (void)
{
    TIMER_commonIRQHandler(TIMER4);
}

/** @} */

/** @} addtogroup TIMER */

