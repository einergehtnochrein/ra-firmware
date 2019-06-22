/* Copyright (c) 2016-2017, DF9DQ
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

#include "lpc54xxx_clkpwr.h"
#include "lpc54xxx_flexcomm.h"
#include "lpc54xxx_uart.h"


LPCLIB_DefineRegBit(UART_CFG_ENABLE,                0,  1);
LPCLIB_DefineRegBit(UART_CFG_DATALEN,               2,  2);
LPCLIB_DefineRegBit(UART_CFG_PARITYSEL,             4,  2);
LPCLIB_DefineRegBit(UART_CFG_STOPLEN,               6,  1);
LPCLIB_DefineRegBit(UART_CFG_MODE32K,               7,  1);
LPCLIB_DefineRegBit(UART_CFG_LINMODE,               8,  1);
LPCLIB_DefineRegBit(UART_CFG_CTSEN,                 9,  1);
LPCLIB_DefineRegBit(UART_CFG_SYNCEN,                11, 1);
LPCLIB_DefineRegBit(UART_CFG_CLKPOL,                12, 1);
LPCLIB_DefineRegBit(UART_CFG_SYNCMST,               14, 1);
LPCLIB_DefineRegBit(UART_CFG_LOOP,                  15, 1);
LPCLIB_DefineRegBit(UART_CFG_OETA,                  18, 1);
LPCLIB_DefineRegBit(UART_CFG_AUTOADDR,              19, 1);
LPCLIB_DefineRegBit(UART_CFG_OESEL,                 20, 1);
LPCLIB_DefineRegBit(UART_CFG_OEPOL,                 21, 1);
LPCLIB_DefineRegBit(UART_CFG_RXPOL,                 22, 1);
LPCLIB_DefineRegBit(UART_CFG_TXPOL,                 23, 1);

LPCLIB_DefineRegBit(UART_CTL_TXBRKEN,               1,  1);
LPCLIB_DefineRegBit(UART_CTL_ADDRDET,               2,  1);
LPCLIB_DefineRegBit(UART_CTL_TXDIS,                 6,  1);
LPCLIB_DefineRegBit(UART_CTL_CC,                    8,  1);
LPCLIB_DefineRegBit(UART_CTL_CLRCCONRX,             9,  1);
LPCLIB_DefineRegBit(UART_CTL_AUTOBAUD,              16, 1);

LPCLIB_DefineRegBit(UART_STAT_RXRDY,                0,  1);
LPCLIB_DefineRegBit(UART_STAT_RXIDLE,               1,  1);
LPCLIB_DefineRegBit(UART_STAT_TXRDY,                2,  1);
LPCLIB_DefineRegBit(UART_STAT_TXIDLE,               3,  1);
LPCLIB_DefineRegBit(UART_STAT_CTS,                  4,  1);
LPCLIB_DefineRegBit(UART_STAT_DELTACTS,             5,  1);
LPCLIB_DefineRegBit(UART_STAT_TXDISSTAT,            6,  1);
LPCLIB_DefineRegBit(UART_STAT_OVERRUNINT,           8,  1);
LPCLIB_DefineRegBit(UART_STAT_RXBRK,                10, 1);
LPCLIB_DefineRegBit(UART_STAT_DELTARXBRK,           11, 1);
LPCLIB_DefineRegBit(UART_STAT_START,                12, 1);
LPCLIB_DefineRegBit(UART_STAT_FRAMEERRINT,          13, 1);
LPCLIB_DefineRegBit(UART_STAT_PARITYERRINT,         14, 1);
LPCLIB_DefineRegBit(UART_STAT_RXNOISEINT,           15, 1);
LPCLIB_DefineRegBit(UART_STAT_ABERR,                16, 1);

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
LPCLIB_DefineRegBit(UART_INTENSET_RXRDYEN,          0,  1);
LPCLIB_DefineRegBit(UART_INTENSET_TXRDYEN,          2,  1);
#endif
LPCLIB_DefineRegBit(UART_INTENSET_TXIDLEEN,         3,  1);
LPCLIB_DefineRegBit(UART_INTENSET_DELTACTSEN,       5,  1);
LPCLIB_DefineRegBit(UART_INTENSET_TXDISEN,          6,  1);
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
LPCLIB_DefineRegBit(UART_INTENSET_OVERRUNEN,        8,  1);
#endif
LPCLIB_DefineRegBit(UART_INTENSET_DELTARXBRKEN,     11, 1);
LPCLIB_DefineRegBit(UART_INTENSET_STARTEN,          12, 1);
LPCLIB_DefineRegBit(UART_INTENSET_FRAMEERREN,       13, 1);
LPCLIB_DefineRegBit(UART_INTENSET_PARITYERREN,      14, 1);
LPCLIB_DefineRegBit(UART_INTENSET_RXNOISECLREN,     15, 1);
LPCLIB_DefineRegBit(UART_INTENSET_ABERREN,          16, 1);

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
LPCLIB_DefineRegBit(UART_INTENCLR_RXRDYCLR,         0,  1);
LPCLIB_DefineRegBit(UART_INTENCLR_TXRDYCLR,         2,  1);
#endif
LPCLIB_DefineRegBit(UART_INTENCLR_TXIDLECLR,        3,  1);
LPCLIB_DefineRegBit(UART_INTENCLR_DELTACTSCLR,      5,  1);
LPCLIB_DefineRegBit(UART_INTENCLR_TXDISCLR,         6,  1);
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
LPCLIB_DefineRegBit(UART_INTENCLR_OVERRUNCLR,       8,  1);
#endif
LPCLIB_DefineRegBit(UART_INTENCLR_DELTARXBRKCLR,    11, 1);
LPCLIB_DefineRegBit(UART_INTENCLR_STARTCLR,         12, 1);
LPCLIB_DefineRegBit(UART_INTENCLR_FRAMEERRCLR,      13, 1);
LPCLIB_DefineRegBit(UART_INTENCLR_PARITYERRCLR,     14, 1);
LPCLIB_DefineRegBit(UART_INTENCLR_RXNOISECLRCLR,    15, 1);
LPCLIB_DefineRegBit(UART_INTENCLR_ABERRCLR,         16, 1);

LPCLIB_DefineRegBit(UART_OSR_OSRVAL,                0,  4);

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
LPCLIB_DefineRegBit(UART_FIFOCFG_ENABLETX,          0,  1);
LPCLIB_DefineRegBit(UART_FIFOCFG_ENABLERX,          1,  1);
LPCLIB_DefineRegBit(UART_FIFOCFG_SIZE,              4,  2);
LPCLIB_DefineRegBit(UART_FIFOCFG_DMATX,             12, 1);
LPCLIB_DefineRegBit(UART_FIFOCFG_DMARX,             13, 1);
LPCLIB_DefineRegBit(UART_FIFOCFG_WAKETX,            14, 1);
LPCLIB_DefineRegBit(UART_FIFOCFG_WAKERX,            15, 1);
LPCLIB_DefineRegBit(UART_FIFOCFG_EMPTYTX,           16, 1);
LPCLIB_DefineRegBit(UART_FIFOCFG_EMPTYRX,           17, 1);

LPCLIB_DefineRegBit(UART_FIFOTRIG_TXLVLENA,         0,  1);
LPCLIB_DefineRegBit(UART_FIFOTRIG_RXLVLENA,         1,  1);
LPCLIB_DefineRegBit(UART_FIFOTRIG_TXLVL,            8,  4);
LPCLIB_DefineRegBit(UART_FIFOTRIG_RXLVL,            16, 4);

LPCLIB_DefineRegBit(UART_FIFOINTENSET_TXERR,        0,  1);
LPCLIB_DefineRegBit(UART_FIFOINTENSET_RXERR,        1,  1);
LPCLIB_DefineRegBit(UART_FIFOINTENSET_TXLVL,        2,  1);
LPCLIB_DefineRegBit(UART_FIFOINTENSET_RXLVL,        3,  1);

LPCLIB_DefineRegBit(UART_FIFOINTENCLR_TXERR,        0,  1);
LPCLIB_DefineRegBit(UART_FIFOINTENCLR_RXERR,        1,  1);
LPCLIB_DefineRegBit(UART_FIFOINTENCLR_TXLVL,        2,  1);
LPCLIB_DefineRegBit(UART_FIFOINTENCLR_RXLVL,        3,  1);

LPCLIB_DefineRegBit(UART_FIFOINTSTAT_TXERR,         0,  1);
LPCLIB_DefineRegBit(UART_FIFOINTSTAT_RXERR,         1,  1);
LPCLIB_DefineRegBit(UART_FIFOINTSTAT_TXLVL,         2,  1);
LPCLIB_DefineRegBit(UART_FIFOINTSTAT_RXLVL,         3,  1);
LPCLIB_DefineRegBit(UART_FIFOINTSTAT_PERINT,        4,  1);
#endif


/** Local context of UART's. */
static struct UART_Context {
    UART_Name bus;                          /**< Bus identifier */
    LPCLIB_Switch inUse;                    /**< Set if interface open */

    osMutexId accessMutex;
    osSemaphoreId syncSema;
    LPCLIB_Callback callback;

    uint8_t *pRxBuffer;
    uint32_t rxBufferSize;
  #if LPCLIB_UART_ERROR_HANDLING
    uint8_t rxErrors[LPCLIB_UART_RX_BUFFER_SIZE];
  #endif
    volatile uint32_t rxWriteIndex;
    uint32_t rxReadIndex;

    uint8_t *pTxBuffer;
    uint32_t txBufferSize;
    uint32_t txWriteIndex;
    volatile uint32_t txReadIndex;
    bool txActive;                          /**< IRQ driven TX active */
    bool txSyncMode;
} uartContext[UART_NUM_UARTS];


//TODO make sure we have enough (and not more) mutexes/syncs!
//TODO must have *globally* unique name...
osMutexDef(uartAccessMutexDef0);
osMutexDef(uartAccessMutexDef1);
osMutexDef(uartAccessMutexDef2);
osMutexDef(uartAccessMutexDef3);
osSemaphoreDef(uartSyncSemaDef0);
osSemaphoreDef(uartSyncSemaDef1);
osSemaphoreDef(uartSyncSemaDef2);
osSemaphoreDef(uartSyncSemaDef3);

static const osMutexDef_t * const uartMutexes[UART_NUM_UARTS] = {
    osMutex(uartAccessMutexDef0),
    osMutex(uartAccessMutexDef1),
    osMutex(uartAccessMutexDef2),
    osMutex(uartAccessMutexDef3),
    };
static const osSemaphoreDef_t * const uartSemas[UART_NUM_UARTS] = {
    osSemaphore(uartSyncSemaDef0),
    osSemaphore(uartSyncSemaDef1),
    osSemaphore(uartSyncSemaDef2),
    osSemaphore(uartSyncSemaDef3),
    };

static LPC_UART_Type * const uartPtr[UART_NUM_UARTS] = {
    LPC_USART0,(LPC_UART_Type *)LPC_UART1, LPC_USART2, LPC_USART3,};
#if 0
static const IRQn_Type uartIrqs[UART_NUM_UARTS] = {
    UART0_IRQn,
    UART1_IRQn,
    UART2_IRQn,
    UART3_IRQn,};
#endif
static const CLKPWR_Clockswitch uartClockSwitch[UART_NUM_UARTS] = {
    CLKPWR_CLOCKSWITCH_UART0, CLKPWR_CLOCKSWITCH_UART1, CLKPWR_CLOCKSWITCH_UART2, CLKPWR_CLOCKSWITCH_UART3,};
static const CLKPWR_Clock uartClock[UART_NUM_UARTS] = {
    CLKPWR_CLOCK_UART0, CLKPWR_CLOCK_UART1, CLKPWR_CLOCK_UART2, CLKPWR_CLOCK_UART3,};


/* Prototypes */
static void UART_commonIRQHandler (UART_Name uartNum);


/* Open a UART channel. */
LPCLIB_Result UART_open (UART_Name uartNum, UART_Handle *pHandle)
{
    LPC_UART_Type *uart = uartPtr[uartNum];
    UART_Handle handle = &uartContext[uartNum];

    /* Enable UART peripheral clock to allow access to registers */
    CLKPWR_enableClock(uartClockSwitch[uartNum]);

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    ((LPC_FLEXCOMM_Type *)uart)->PSELID = 1;    //TODO
    FLEXCOMM_installHandler((FLEXCOMM_Name)uartNum, (FLEXCOMM_IRQHandler_t)UART_commonIRQHandler);

    uart->FIFOCFG = 0
                    | (1u << UART_FIFOCFG_ENABLETX_Pos)
                    | (1u << UART_FIFOCFG_ENABLERX_Pos)
                    | (0  << UART_FIFOCFG_DMATX_Pos)
                    | (0  << UART_FIFOCFG_DMARX_Pos)
                    ;
    uart->FIFOTRIG = 0
                    | (1u << UART_FIFOTRIG_TXLVLENA_Pos)
                    | (1u << UART_FIFOTRIG_RXLVLENA_Pos)
                    | (0  << UART_FIFOTRIG_TXLVL_Pos)
                    | (0  << UART_FIFOTRIG_RXLVL_Pos)
                    ;
#endif

    /* Cannot open a UART twice */
    if (!uartContext[uartNum].inUse) {
        handle->accessMutex = osMutexCreate(uartMutexes[uartNum]);
        handle->syncSema = osSemaphoreCreate(uartSemas[uartNum], 1);

        uart->INTENCLR = 0xFFFFFFFF;                    /* Disable interrupts */  //TODO
        uart->OSR = 16 - 1;                             /* Default oversampling rate = 16 */
        uart->BRG = 0;                                  /* Baud rate divider = 1 */
        uart->CTL = 0                                   /* Normal */
                    | (0 << UART_CTL_TXDIS_Pos)
                    ;
        uart->CFG = 0                                   /* Enable UART 8N1 */
                    | (1 << UART_CFG_ENABLE_Pos)
                    | (UART_DATABITS_8 << UART_CFG_DATALEN_Pos)
                    | (UART_PARITY_NONE << UART_CFG_PARITYSEL_Pos)
                    | (UART_STOPBITS_1 << UART_CFG_STOPLEN_Pos)
                    ;

        *pHandle = &uartContext[uartNum];
        handle->bus = uartNum;
        handle->inUse = LPCLIB_YES;
        handle->txActive = false;

        /* Enable interrupts */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
        uart->INTENSET = 0
                    | UART_INTENSET_RXRDYEN_Msk
                    ;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
        uart->FIFOINTENSET = 0
                    | UART_FIFOINTENSET_RXLVL_Msk
                    ;
#endif

        return LPCLIB_SUCCESS;
    }

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_BUSY;
}


/* Close a UART channel */
void UART_close (UART_Handle *pHandle)
{
    LPC_UART_Type *uart;

    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    uart = uartPtr[(*pHandle)->bus];
    (*pHandle)->inUse = LPCLIB_NO;
    (*pHandle)->txActive = false;

    uart->INTENCLR = 0xFFFFFFFF;                        /* Disable interrupts */  //TODO

    CLKPWR_disableClock(uartClockSwitch[(*pHandle)->bus]);  /* Disable peripheral clock */

    *pHandle = LPCLIB_INVALID_HANDLE;
}


int UART_read (UART_Handle handle, void *buffer, int nbytes)
{
    int nread;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return 0;
    }

    nread = 0;

    while (nbytes > 0) {
        if (handle->rxWriteIndex != handle->rxReadIndex) {
            ((uint8_t *)buffer)[nread] = handle->pRxBuffer[handle->rxReadIndex];
            ++nread;
            ++handle->rxReadIndex;
            if (handle->rxReadIndex >= handle->rxBufferSize) {
                handle->rxReadIndex = 0;
            }
        }
        else {
            break;      /* There is no more RX data available */
        }

        --nbytes;
    }

    return nread;
}


/* Read a complete line (terminated by either CR or LF). */
int UART_readLine (UART_Handle handle, void *buffer, int nbytes)
{
    int nread = 0;
    uint32_t ri = handle->rxReadIndex;
    bool lineComplete = false;
    bool overflow = false;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return 0;
    }

    /* Verify existence of RX FIFO */
    if (!handle->pRxBuffer) {
        return 0;
    }

    /* Verify buffer has enough room */
    if (nbytes < 1) {
        return 0;
    }

    /* Loop over all characters in RX FIFO, until either all characters are consumed
     * or an end-of-line marker is found.
     */
    while (ri != handle->rxWriteIndex) {
        char c = handle->pRxBuffer[ri];

        ++ri;
        if (ri >= handle->rxBufferSize) {
            ri = 0;
        }

        /* Store in buffer (if room left) */
        if (nbytes > 1) {   /* We need room for terminating 0! */
            ((uint8_t *)buffer)[nread] = c;
            --nbytes;
            ++nread;
        }
        else {
            overflow = true;
        }

        /* Line end? */
        if ((c == '\n') || (c == '\r')) {
            lineComplete = true;

            /* Terminate string */
            ((uint8_t *)buffer)[nread] = 0;

            /* Remove characters from RX buffer */
            handle->rxReadIndex = ri;

            if (overflow) {
                nread = -1;
            }
            break;
        }
    }

    /* RX FIFO full without EOL character? */
    if ((uint32_t)(nread + 1) >= handle->rxBufferSize) {
        /* Overflow error (line too long) */
        nread = -1;

        /* Remove characters from RX buffer */
        handle->rxReadIndex = ri;
        lineComplete = true;
    }

    if (!lineComplete) {
        nread = 0;
    }

    return nread;
}


/* Determine number of free TX buffer entries */
int UART_getTxFree (UART_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return -1;
    }

    if (handle->pTxBuffer == NULL) {
        return -1;
    }

    uint32_t rdIndex = handle->txReadIndex;
    if (rdIndex > handle->txWriteIndex) {
        return rdIndex - handle->txWriteIndex;
    }

    return handle->txBufferSize - 1 - (handle->txWriteIndex - rdIndex);
}


int UART_write (UART_Handle handle, const void *buffer, int nbytes)
{
    int nwritten;
    uint32_t temp;
    LPC_UART_Type * const uart = uartPtr[handle->bus];


    if (handle == LPCLIB_INVALID_HANDLE) {
        return 0;
    }

    nwritten = 0;

    /* If we have no TX buffer, ignore all data */
    if ((handle->pTxBuffer == NULL) && !handle->txActive) {
        handle->txActive = true;
    }
    else {
        while (nbytes > 0) {
            /* Calculate next index */
            temp = handle->txWriteIndex + 1;
            if (temp >= handle->txBufferSize) {
                temp = 0;
            }

            /* If we would reach the buffer tail now, the buffer is already full! */
            if (temp != handle->txReadIndex) {
                handle->pTxBuffer[handle->txWriteIndex] = ((uint8_t *)buffer)[nwritten];
                handle->txWriteIndex = temp;
                ++nwritten;
                --nbytes;
            }
            else {
                /* Buffer full.
                * Synchronous mode: Wait for signal from TX interrupt.
                * Asynchronous mode: Exit
                *///TODO
                if (handle->txSyncMode) {
                    handle->txActive = true;
                    /* Enable TX interrupt. */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
                    uart->INTENSET = UART_INTENSET_TXRDYEN_Msk;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
                    uart->FIFOINTENSET = UART_FIFOINTENSET_TXLVL_Msk;
#endif

                    osSemaphoreWait(handle->syncSema, 100 /*TODO*/);
                }
                else {
                    break;
                }
            }
        }
    }

    /* Enable TX interrupt. */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    uart->INTENSET = UART_INTENSET_TXRDYEN_Msk;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    uart->FIFOINTENSET = UART_FIFOINTENSET_TXLVL_Msk;
#endif

    return nwritten;
}



void UART_ioctl (UART_Handle handle, const UART_Config *pConfig)
{
    uint32_t temp;
    LPC_UART_Type * const uart = uartPtr[handle->bus];

    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    while (pConfig->opcode != UART_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case UART_OPCODE_SET_BAUDRATE:
            {
                /* Determine peripheral clock for baud rate generation */
                uint32_t clock = CLKPWR_getBusClock(uartClock[handle->bus]);

                if (uart->CFG & UART_CFG_SYNCEN_Msk) {
                    /* Synchronous mode (no oversampling) */
                    uart->BRG = clock / pConfig->baudrate - 1;  //TODO check parameter
                }
                else {
                    /* Asynchronous mode (uses oversampling) */
                    //TODO: Fractional divider: a) setting support, b) calculation support
                    uint32_t OSR;
                    uint32_t bestOSR;
                    int32_t diff;
                    uint32_t bestDiff;
                    uint32_t BRG;
                    /* Try some OSR values, and keep the one that produces the closest baud rate match */
                    bestDiff = (uint32_t)(-1);
                    bestOSR = 16;
                    for (OSR = 10; OSR <= 16; OSR++) {
                        BRG = (int)(clock / (OSR * pConfig->baudrate));
                        diff = (int)pConfig->baudrate - clock / (OSR * BRG);
                        if (diff < 0) {
                            diff = -diff;
                        }
                        if ((uint32_t)diff < bestDiff) {
                            bestOSR = OSR;
                            bestDiff = diff;
                        }
                    }
                    uart->OSR = (bestOSR - 1) << UART_OSR_OSRVAL_Pos;
                    uart->BRG = clock
                        / ((((uart->OSR & UART_OSR_OSRVAL_Msk) >> UART_OSR_OSRVAL_Pos) + 1) * pConfig->baudrate) - 1;
                }
            }
            break;

#if 0
        case UART_OPCODE_SET_BAUDRATE_BY_DIVIDERS:
            uart->DLL = pConfig->pDividers->integer & 0xFF;
            uart->DLM = (pConfig->pDividers->integer >> 8) & 0xFF;
            uart->FDR = ((pConfig->pDividers->fracMul << UART_FDR_MULVAL_Pos) & UART_FDR_MULVAL_Msk)
                      | ((pConfig->pDividers->fracDivAdd << UART_FDR_DIVADDVAL_Pos) & UART_FDR_DIVADDVAL_Msk)
                      ;
            break;
#endif
        case UART_OPCODE_SET_ASYNC_FORMAT:
            temp = uart->CFG & ~(0
                | UART_CFG_DATALEN_Msk
                | UART_CFG_PARITYSEL_Msk
                | UART_CFG_STOPLEN_Msk
                | UART_CFG_LINMODE_Msk
                | UART_CFG_SYNCEN_Msk
                | UART_CFG_CLKPOL_Msk
                | UART_CFG_SYNCMST_Msk
                );
            uart->CFG = temp
                | ((uint32_t)pConfig->asyncFormat.databits << UART_CFG_DATALEN_Pos)
                | ((uint32_t)pConfig->asyncFormat.stopbits << UART_CFG_STOPLEN_Pos)
                | ((uint32_t)pConfig->asyncFormat.parity << UART_CFG_PARITYSEL_Pos)
                ;
            break;

        case UART_OPCODE_SET_SYNC_FORMAT:
            temp = uart->CFG & ~(0
                | UART_CFG_DATALEN_Msk
                | UART_CFG_PARITYSEL_Msk
                | UART_CFG_STOPLEN_Msk
                | UART_CFG_LINMODE_Msk
                | UART_CFG_SYNCEN_Msk
                | UART_CFG_CLKPOL_Msk
                | UART_CFG_SYNCMST_Msk
                );
            uart->CFG = temp
                | ((uint32_t)pConfig->syncFormat.databits << UART_CFG_DATALEN_Pos)
                | ((uint32_t)pConfig->syncFormat.samplingClock << UART_CFG_CLKPOL_Pos)
                | ((uint32_t)pConfig->syncFormat.syncMaster << UART_CFG_SYNCMST_Pos)
                | (1 << UART_CFG_SYNCEN_Pos)
                ;
            break;

        case UART_OPCODE_SET_CALLBACK:
            if (pConfig->callback.pOldCallback) {       /* Return current callback if requested */
                *(pConfig->callback.pOldCallback) = handle->callback;
            }
            handle->callback = pConfig->callback.callback;
            break;

        case UART_OPCODE_SET_TX_BUFFER:
            handle->pTxBuffer = pConfig->buffer.pBuffer;
            handle->txBufferSize = pConfig->buffer.size;
            break;

        case UART_OPCODE_SET_RX_BUFFER:
            handle->pRxBuffer = pConfig->buffer.pBuffer;
            handle->rxBufferSize = pConfig->buffer.size;
            break;
#if 0
        case UART_OPCODE_SET_FIFO_THRESHOLD:
            /* NOTE: This should only be called once immediately after opening the port.
             *       (otherwise disable interrupt)
             */
            uart->SCR = (uart->SCR & ~UART_CONTEXT_FIFO_MASK)
                      | ((pConfig->threshold >> 8) & 0xFF);
            uart->FCR = (pConfig->threshold & 0xFF) | UART_FCR_FIFOENABLE_Msk;
            break;
#endif

        case UART_OPCODE_SET_HARDWARE_HANDSHAKE:
            uart->CFG = (uart->CFG & ~UART_CFG_CTSEN_Msk)
                    | (pConfig->hardwareHandshake ? UART_CFG_CTSEN_Msk : 0)
                    ;
            break;

        case UART_OPCODE_SET_BREAK:                     /* TXD break condition */
            if (pConfig->enableBreak) {
#if (__CORTEX_M == 4)
                /* Clean TX disable to allow current character to be sent completely */
                LPCLIB_BITBAND(&uart->CTL, UART_CTL_TXDIS_Pos) = 1;
                while (!LPCLIB_BITBAND(&uart->STAT, UART_STAT_TXDISSTAT_Pos))
                    ;

                /* Enter TXD break */
                LPCLIB_BITBAND(&uart->CTL, UART_CTL_TXBRKEN_Pos) = 1;
#else
                /* Clean TX disable to allow current character to be sent completely */
                uart->CTL |= UART_CTL_TXDIS_Msk;
                while (!(uart->STAT & UART_STAT_TXDISSTAT_Msk))
                    ;

                /* Enter TXD break */
                uart->CTL |= UART_CTL_TXBRKEN_Msk;
#endif
            }
            else {
#if (__CORTEX_M == 4)
                /* Back to normal operation */
                LPCLIB_BITBAND(&uart->CTL, UART_CTL_TXBRKEN_Pos) = 0;
                /* Re-enable TX */
                LPCLIB_BITBAND(&uart->CTL, UART_CTL_TXDIS_Pos) = 0;
#else
                /* Back to normal operation */
                uart->CTL &= ~UART_CTL_TXBRKEN_Msk;
                /* Re-enable TX */
                uart->CTL &= ~UART_CTL_TXDIS_Msk;
#endif
            }
            break;

        case UART_OPCODE_INVALID:
        default:
            /* ignore */
            break;
        }

        ++pConfig;
    }
}


static bool UART_handleTxChar (UART_Handle handle, LPCLIB_Event *pEvent)
{
    (void)pEvent;

    LPC_UART_Type * const uart = uartPtr[handle->bus];

    if (handle->txWriteIndex != handle->txReadIndex) {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
        uart->TXDAT = handle->pTxBuffer[handle->txReadIndex];
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
        uart->FIFOWR = handle->pTxBuffer[handle->txReadIndex];
#endif

        ++(handle->txReadIndex);
        if (handle->txReadIndex >= handle->txBufferSize) {
            handle->txReadIndex = 0;
        }

        return true;                                    /* New character sent */
    }

    return false;                                       /* No character sent */
}


/** Handle a receive character.
 *
 *  If an RX buffer exists, the character is put into that buffer. In case the buffer is full,
 *  the RX character is ignored, and an error counter is incremented.
 *  If no RX buffer exists, the callback handler is informed about the character.
 */
static bool UART_handleRxChar (UART_Handle handle, LPCLIB_Event *pEvent)
{
    LPC_UART_Type * const uart = uartPtr[handle->bus];
    uint8_t c;
    uint32_t temp;

    /* Take character from FIFO */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    c = uart->RXDAT;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    c = uart->FIFORD;
#endif

    /* If we have no buffer, we must call the callback handler. */
    if (handle->pRxBuffer == NULL) {
        pEvent->parameter = (void *)(0 + c);
        if (handle->callback) {
            pEvent->opcode = UART_EVENT_RX;
            handle->callback(*pEvent);
        }
    }

    /* If we have a buffer, we try to store the character there. */
    if (handle->pRxBuffer != NULL) {
        /* Calculate coming write index */
        temp = handle->rxWriteIndex + 1;
        if (temp >= handle->rxBufferSize) {
            temp = 0;
        }

        /* If we reach the buffer tail, this is an overflow condition.
        * If not, store the character.
        */
        if (temp != handle->rxReadIndex) {
            handle->pRxBuffer[handle->rxWriteIndex] = c;
            handle->rxWriteIndex = temp;

            return true;
        }
    }

    return false;
}



/** Common UART interrupt handling.
 *
 *  \param[in] uartNum UART number
 */
static void UART_commonIRQHandler (UART_Name uartNum)
{
    LPC_UART_Type * const uart = uartPtr[uartNum];
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    uint32_t stat;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    uint32_t fifointstat;
#endif
//    uint8_t n;
    LPCLIB_Event event;
    UART_Handle handle = &uartContext[uartNum];
//    volatile uint8_t dummy;


    /* Preset some event fields */
    event.id = LPCLIB_EVENTID_UART;
    event.block = uartNum;

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    stat = uart->STAT;

    if (stat & UART_STAT_RXRDY_Msk) {
        UART_handleRxChar(handle, &event);
    }

    if (stat & UART_STAT_TXRDY_Msk) {
        if (!UART_handleTxChar(handle, &event)) {
            uart->INTENCLR = UART_INTENCLR_TXRDYCLR_Msk;
        }
    }
#endif

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    fifointstat = uart->FIFOINTSTAT;

    if (fifointstat & UART_FIFOINTSTAT_RXLVL_Msk) {
        UART_handleRxChar(handle, &event);
    }

    if (fifointstat & UART_FIFOINTSTAT_TXLVL_Msk) {
        if (!UART_handleTxChar(handle, &event)) {
            uart->FIFOINTENCLR = UART_FIFOINTENCLR_TXLVL_Msk;
        }
    }
#endif
}


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
/** UART0 interrupt handler.
 */
void UART0_IRQHandler (void)
{
    UART_commonIRQHandler(UART0);
}

/** UART1 interrupt handler.
 */
void UART1_IRQHandler (void)
{
    UART_commonIRQHandler(UART1);
}

/** UART2 interrupt handler.
 */
void UART2_IRQHandler (void)
{
    UART_commonIRQHandler(UART2);
}

/** UART3 interrupt handler.
 */
void UART3_IRQHandler (void)
{
    UART_commonIRQHandler(UART3);
}
#endif

/** @} */

/** @} */

