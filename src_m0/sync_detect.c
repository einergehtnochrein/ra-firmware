
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
    ProcessFunc postProcess;
    ProcessFunc byteProcess;
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
                            handle->byteProcess = handle->config->conf[i].byteProcess;
                            handle->symbolPhase = 0;
                            handle->nSubBlockBits = handle->config->conf[i].nSubBlockBits;
                            handle->nSubBlockBytes = handle->config->conf[i].nSubBlockBytes;
                            ipc_s2m[handle->activeBuffer].opcode = 0;
                            ipc_s2m[handle->activeBuffer].numBits = handle->frameLengthBits + handle->config->conf[i].startOffset * 8;
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
                    if (handle->byteProcess) {
                        handle->byteProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                    }
                }

                if (--handle->rxCounterBits <= 0) {
                    handle->state = SYNC_STATE_HUNT;
                    if (handle->postProcess) {
                        handle->postProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                    }
                    handle->writeIndex = 0;
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
                        if (handle->byteProcess) {
                            handle->byteProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                        }
                    }

                    if (--handle->rxCounterBits <= 0) {
                        handle->state = SYNC_STATE_HUNT;
                        if (handle->postProcess) {
                            handle->postProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                        }
                        handle->writeIndex = 0;
                        ipc_s2m[handle->activeBuffer].valid = 1;

                        LPC_MAILBOX->IRQ1SET = (1u << 0);
                    }
                    else if (handle->rxCounterBits == handle->frameLengthBits / 2) {
                        /* Tell M4 to keep current RSSI value */
                        LPC_MAILBOX->IRQ1SET = (1u << 1);
                    }
                }
                break;

            case SYNC_STATE_DATA_BIPHASE_M:
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
                    int offset = 7 - (handle->bitCounter % 8);  //TODO reverse, make this a config option!
                    bit = (bit == handle->lastBit) ? 0 : 1;
                    ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] |= (bit << offset);

                    handle->symbolPhase = 0;

                    ++handle->bitCounter;
                    if ((handle->bitCounter % 8) == 0) {
                        ++handle->writeIndex;
                        if (handle->byteProcess) {
                            handle->byteProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                        }
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
                        ipc_s2m[handle->activeBuffer].numBits = 8*handle->writeIndex; //TODO

                        handle->state = SYNC_STATE_HUNT;
                        if (handle->postProcess) {
                            handle->postProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                        }
                        handle->writeIndex = 0;
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
                        if (handle->byteProcess) {
                            handle->byteProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                        }
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
                        ipc_s2m[handle->activeBuffer].numBits = 8*handle->writeIndex; //TODO

                        handle->state = SYNC_STATE_HUNT;
                        if (handle->postProcess) {
                            handle->postProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                        }
                        handle->writeIndex = 0;
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
                        if (handle->byteProcess) {
                            handle->byteProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                        }
                    }
                    if (--handle->rxCounterBits <= 0) {
                        handle->state = SYNC_STATE_HUNT;
                        if (handle->postProcess) {
                            handle->postProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                        }
                        handle->writeIndex = 0;
                        ipc_s2m[handle->activeBuffer].valid = 1;

                        LPC_MAILBOX->IRQ1SET = (1u << 0);
                    }
                    else if (handle->rxCounterBits == handle->frameLengthBits / 2) {
                        /* Tell M4 to keep current RSSI value */
                        LPC_MAILBOX->IRQ1SET = (1u << 1);
                    }
                }
                break;

            case SYNC_STATE_DATA_MANCHESTER_UART_8N1:
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

                    /* Inner UART decoder */
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
                            if (handle->byteProcess) {
                                handle->byteProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                            }
                        }

                        if (--handle->rxCounterBits <= 0) {
                            handle->state = SYNC_STATE_HUNT;
                            if (handle->postProcess) {
                                handle->postProcess(handle, &ipc_s2m[handle->activeBuffer], handle->writeIndex);
                            }
                            handle->writeIndex = 0;
                            ipc_s2m[handle->activeBuffer].valid = 1;

                            LPC_MAILBOX->IRQ1SET = (1u << 0);
                        }
                        else if (handle->rxCounterBits == handle->frameLengthBits / 2) {
                            /* Tell M4 to keep current RSSI value */
                            LPC_MAILBOX->IRQ1SET = (1u << 1);
                        }
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


/* Meteomodem M10 and M20 use the same 32-bit frame sync word.
 * The byte following the sync pattern indicates the payload length. This length can vary
 * if external (XDATA) instruments are connected. This hook function updates the frame receiver
 * with the correct length once the first byte is received.
 *
 * The second byte (first byte of the payload) indicates the sonde model.
 * We process this in the hook function as well, although it should better be done in a general
 * Meteomodem sonde payload parser (on the M4 core).
 */
void _SYNC_modemByteProcess (SYNC_Handle handle, volatile IPC_S2M *buffer, int writeIndex)
{
    int nBits;
    uint8_t typeCode;

    /* Update frame length */
    if (writeIndex == 1) {
        nBits = (1 + buffer->data8[0]) * 8; /* 1 length byte + n bytes */

        handle->rxCounterBits = nBits;
        handle->frameLengthBits = nBits;
    }

    /* Update sonde type */ //TODO: move this functionality to M4
    if (writeIndex == 2) {
        typeCode = buffer->data8[1];
        if (typeCode == 0x9F) {
            ipc_s2m[handle->activeBuffer].param = IPC_PACKET_TYPE_MODEM_M10;
        }
        if (typeCode == 0x20) {
            ipc_s2m[handle->activeBuffer].param = IPC_PACKET_TYPE_MODEM_M20;
        }
    }
}

