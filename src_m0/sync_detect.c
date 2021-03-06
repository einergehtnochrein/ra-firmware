
#include <string.h>

#include "lpclib.h"
#include "bsp.h"
#include "sync_detect.h"


#define MAX_PATTERNS        4

struct _SyncDetectorContext {
    uint64_t rxShiftReg[2];
    const SYNC_Config *config;

    SYNC_State state;
    int activeBuffer;
    int detectedType;           /* 0=RS92, 1=RS41, 2=DFM06, 3=DFM09 */
    int frameLengthBits;
    int rxCounterBits;
    int bitCounter;
    int writeIndex;
    int uartBitCounter;
    bool inverted;
    int lastBit;
    PostProcessFunc postProcess;
    int symbolPhase;
    int nSubBlockBits;
    int nSubBlockBytes;
} syncContext;



__SECTION(".ipc")
volatile IPC_S2M ipc_s2m[IPC_S2M_NUM_BUFFERS];

void PIN_INT3_IRQHandler (void)
{
    SYNC_Handle handle = &syncContext;
    int i, j;
    int nDifferences;
    int bit;
    bool skipBit;

    LPC_PINT->IST = (1u << 3);

    if (handle->config && GPIO_readBit(GPIO_RX_CLK)) {
        /* Read data bit */
        bit = GPIO_readBit(GPIO_RX_DATA);

        switch (handle->state) {
            case SYNC_STATE_HUNT:
                /* Take RX bit and put into 128-bit shift register */
                handle->rxShiftReg[1] <<= 1;
                if (handle->rxShiftReg[0] & (1ull << 63)) {
                    handle->rxShiftReg[1] |= 1;
                }
                handle->rxShiftReg[0] <<= 1;
                handle->rxShiftReg[0] |= bit;

                /* Compare received bits with expected sync patterns */
                for (i = 0; i < handle->config->nPatterns; i++) {
                    /* Compare with expected pattern. (Use fast functions to calculate Hamming weight.) */
                    nDifferences = 0
                        + __builtin_popcountll((handle->rxShiftReg[0] ^ handle->config->conf[i].pattern[0]) & handle->config->conf[i].patternMask[0])
                        + __builtin_popcountll((handle->rxShiftReg[1] ^ handle->config->conf[i].pattern[1]) & handle->config->conf[i].patternMask[1])
                        ;

                    if (nDifferences <= handle->config->conf[i].nMaxDifference) {
                        /* SYNC! Start frame reception */

                        /* Find available buffer */
                        for (j = 0; j < IPC_S2M_NUM_BUFFERS; j++) {
                            if (!ipc_s2m[j].valid) {
                                break;  /* found */
                            }
                        }
                        if (j < IPC_S2M_NUM_BUFFERS) {
                            handle->activeBuffer = j;
                            handle->frameLengthBits = handle->config->conf[i].frameLengthBits;
                            handle->rxCounterBits = handle->config->conf[i].frameLengthBits;
                            handle->bitCounter = 0;
                            handle->state = handle->config->conf[i].dataState;
                            handle->inverted = handle->config->conf[i].inverted;
                            handle->uartBitCounter = 0;
                            handle->writeIndex = handle->config->conf[i].startOffset;
                            handle->postProcess = handle->config->conf[i].postProcess;
                            handle->symbolPhase = 0;
                            handle->nSubBlockBits = handle->config->conf[i].nSubBlockBits;
                            handle->nSubBlockBytes = handle->config->conf[i].nSubBlockBytes;
                            ipc_s2m[handle->activeBuffer].opcode = 0;
                            ipc_s2m[handle->activeBuffer].numBits = handle->frameLengthBits;
                            ipc_s2m[handle->activeBuffer].param = handle->config->conf[i].id;
                            ipc_s2m[handle->activeBuffer].rxTime = os_time;
                        }
                    }
                }
                break;

            case SYNC_STATE_DATA_RAW:
                if (handle->inverted) {
                    bit = bit ^ 1;
                }

                ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] =
                    (ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] << 1) | bit;

                if (++handle->bitCounter >= 8) {
                    handle->bitCounter = 0;
                    ++handle->writeIndex;
                }

                if (--handle->rxCounterBits <= 0) {
                    handle->state = SYNC_STATE_HUNT;
                    handle->writeIndex = 0;
                    if (handle->postProcess) {
                        handle->postProcess(&ipc_s2m[handle->activeBuffer]);
                    }
                    ipc_s2m[handle->activeBuffer].valid = 1;

                    LPC_MAILBOX->IRQ1SET = (1u << 0);
                }
                else if (handle->rxCounterBits == handle->frameLengthBits / 2) {
                    /* Tell M4 to keep current RSSI value */
                    LPC_MAILBOX->IRQ1SET = (1u << 1);
                }
                break;

            case SYNC_STATE_DATA_UART_8N1:
                skipBit = false;

                /* Strip start and stop bit */
                if (handle->uartBitCounter == 0) {
                    skipBit = true;
                }
                if (handle->uartBitCounter == 9) {
                    skipBit = true;
                }

                /* 8N1: 10 bits per character */
                ++handle->uartBitCounter;
                if (handle->uartBitCounter >= 10) {
                    handle->uartBitCounter = 0;
                }

                if (!skipBit) {
                    if (handle->inverted) {
                        bit = bit ^ 1;
                    }

                    ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] =
                        (ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] >> 1) | (bit << 7);

                    if (++handle->bitCounter >= 8) {
                        handle->bitCounter = 0;
                        ++handle->writeIndex;
                    }

                    if (--handle->rxCounterBits <= 0) {
                        handle->state = SYNC_STATE_HUNT;
                        handle->writeIndex = 0;
                        if (handle->postProcess) {
                            handle->postProcess(&ipc_s2m[handle->activeBuffer]);
                        }
                        ipc_s2m[handle->activeBuffer].valid = 1;

                        LPC_MAILBOX->IRQ1SET = (1u << 0);
                    }
                    else if (handle->rxCounterBits == handle->frameLengthBits / 2) {
                        /* Tell M4 to keep current RSSI value */
                        LPC_MAILBOX->IRQ1SET = (1u << 1);
                    }
                }
                break;

            case SYNC_STATE_DATA_BIPHASE_S:
                if (handle->symbolPhase == 0) {
                    /* First bit in a Manchester symbol: Remember for later evaluation */
                    handle->lastBit = bit;
                    ++handle->symbolPhase;
                }
                else {
                    if ((handle->bitCounter % 8) == 0) {
                        ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] = 0;
                    }

                    /* Second bit in a Manchester symbol: Determine and save data bit */
                    int offset = handle->bitCounter % 8;
                    bit = (bit == handle->lastBit) ? 1 : 0;
                    ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] |= (bit << offset);

                    handle->symbolPhase = 0;

                    ++handle->bitCounter;
                    if ((handle->bitCounter % 8) == 0) {
                        ++handle->writeIndex;
                    }
                    if (handle->nSubBlockBits != 0) {
                        if (handle->bitCounter >= handle->nSubBlockBits) {
                            handle->bitCounter = 0;
                            int bits2skip = 8 * handle->nSubBlockBytes - handle->nSubBlockBits;
                            if (bits2skip > 0) {
                                handle->writeIndex += 1 + (bits2skip - 1) / 8;
                            }
                        }
                    }

                    if (--handle->rxCounterBits <= 0) {
                        handle->state = SYNC_STATE_HUNT;
                        handle->writeIndex = 0;
                        if (handle->postProcess) {
                            handle->postProcess(&ipc_s2m[handle->activeBuffer]);
                        }
                        ipc_s2m[handle->activeBuffer].valid = 1;

                        LPC_MAILBOX->IRQ1SET = (1u << 0);
                    }
                    else if (handle->rxCounterBits == handle->frameLengthBits / 2) {
                        /* Tell M4 to keep current RSSI value */
                        LPC_MAILBOX->IRQ1SET = (1u << 1);
                    }
                }
                break;

            case SYNC_STATE_DATA_MANCHESTER:
                /* Synchronize Manchester decoder */
                if (bit == handle->lastBit) {
                    handle->symbolPhase = 0;
                }
                handle->lastBit = bit;

                if (handle->symbolPhase == 0) {
                    handle->symbolPhase = 1;
                }
                else {
                    handle->symbolPhase = 0;

                    if ((handle->bitCounter % 8) == 0) {
                        ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] = 0;
                    }

                    if (handle->inverted) {
                        bit = bit ^ 1;
                    }
                    ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] |= (bit << (7 - (handle->bitCounter % 8)));

                    ++handle->bitCounter;
                    if ((handle->bitCounter % 8) == 0) {
                        ++handle->writeIndex;
                    }
                    if (--handle->rxCounterBits <= 0) {
                        handle->state = SYNC_STATE_HUNT;
                        handle->writeIndex = 0;
                        if (handle->postProcess) {
                            handle->postProcess(&ipc_s2m[handle->activeBuffer]);
                        }
                        ipc_s2m[handle->activeBuffer].valid = 1;

                        LPC_MAILBOX->IRQ1SET = (1u << 0);
                    }
                    else if (handle->rxCounterBits == handle->frameLengthBits / 2) {
                        /* Tell M4 to keep current RSSI value */
                        LPC_MAILBOX->IRQ1SET = (1u << 1);
                    }
                }
                break;

            default:    /* Should never come here! */
                handle->state = SYNC_STATE_HUNT;
                break;
        }
    }
}


void SYNC_open (SYNC_Handle *pHandle)
{
    SYNC_Handle handle = &syncContext;

    //TODO
    /* Configure pin interrupt for RX clock */
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_INPUTMUX);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_PINT);
    LPC_INPUTMUX->PINTSEL3 =        // RX clock
        32 * GPIO_getPortNumberFromPin(GPIO_RX_CLK) + GPIO_getBitNumberFromPin(GPIO_RX_CLK);
    LPC_PINT->ISEL &= ~(1u << 3);   // edge triggered
    LPC_PINT->CIENF = (1u << 3);    // disable falling edge
    LPC_PINT->SIENR = (1u << 3);    // enable rising edge

    *pHandle = handle;
}


void SYNC_configure (SYNC_Handle handle, const SYNC_Config *pConfig)
{
    if (pConfig == NULL) {
        memset(&syncContext, 0, sizeof(syncContext));
    }
    else {
        handle->config = pConfig;
    }
}

