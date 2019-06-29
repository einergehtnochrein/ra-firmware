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
 *  \brief FLEXCOMM driver implementation.
 */

/** \addtogroup FLEXCOMM
 *  @{
 */

#include "lpc54xxx_libconfig.h"


#include "lpclib_types.h"
#include "lpc54xxx_flexcomm.h"


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X


/* IRQ handlers */
static FLEXCOMM_IRQHandler_t IRQHandlers [FLEXCOMM_COUNT];



/* Install an interrupt handler for a FLEXCOMM interface */
void FLEXCOMM_installHandler (FLEXCOMM_Name flexcomm, FLEXCOMM_IRQHandler_t handler)
{
    IRQHandlers[flexcomm] = handler;
}


__SECTION(".vectors")
void FLEXCOMM0_IRQHandler (void)
{
    if (IRQHandlers[0]) {
        IRQHandlers[0](0);
    }
}

__SECTION(".vectors")
void FLEXCOMM1_IRQHandler (void)
{
    if (IRQHandlers[1]) {
        IRQHandlers[1](1);
    }
}

__SECTION(".vectors")
void FLEXCOMM2_IRQHandler (void)
{
    if (IRQHandlers[2]) {
        IRQHandlers[2](2);
    }
}

__SECTION(".vectors")
void FLEXCOMM3_IRQHandler (void)
{
    if (IRQHandlers[3]) {
        IRQHandlers[3](3);
    }
}

__SECTION(".vectors")
void FLEXCOMM4_IRQHandler (void)
{
    if (IRQHandlers[4]) {
        IRQHandlers[4](4);
    }
}

__SECTION(".vectors")
void FLEXCOMM5_IRQHandler (void)
{
    if (IRQHandlers[5]) {
        IRQHandlers[5](5);
    }
}

__SECTION(".vectors")
void FLEXCOMM6_IRQHandler (void)
{
    if (IRQHandlers[6]) {
        IRQHandlers[6](6);
    }
}

__SECTION(".vectors")
void FLEXCOMM7_IRQHandler (void)
{
    if (IRQHandlers[7]) {
        IRQHandlers[7](7);
    }
}


/** @} */

#endif

/** @} FLEXCOMM */
