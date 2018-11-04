
#include <string.h>

#include "lpclib.h"
#include "bsp.h"
#include "sync_detect.h"


#define MAX_PATTERNS        4

struct _SyncDetectorContext {
    uint64_t rxShiftReg[2];
    const SYNC_Config *config;

    int state;                  /* 0=sync hunt, 1=data reception */
    int activeBuffer;
    int detectedType;           /* 0=RS92, 1=RS41, 2=DFM06, 3=DFM09 */
    int frameLength;
    int rxCounter;
    int bitCounter;
    int writeIndex;
    int uartBitCounter;
    SYNC_BitFormat bitFormat;
    bool inverted;
    PostProcessFunc postProcess;
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
            case 0:           /* Sync hunt */
                /* Take RX bit and put into 128-bit shift register */
                handle->rxShiftReg[1] <<= 1;
                if (handle->rxShiftReg[0] & (1ull << 63)) {
                    handle->rxShiftReg[1] |= 1;
                }
                handle->rxShiftReg[0] <<= 1;
                handle->rxShiftReg[0] |= bit;

                /* Compare received bits with expected sync patterns */
                for (i = 0; i < handle->config->nPatterns; i++) {
                    uint64_t compare;
                    nDifferences = 0;
                    compare = handle->rxShiftReg[0] ^ handle->config->conf[i].pattern[0];
                    for (j = 0; (j < 64) && (j < handle->config->conf[i].nSyncLen); j++) {
                        if (compare & (1ull << j)) {
                            ++nDifferences;
                        }
                    }
                    compare = handle->rxShiftReg[1] ^ handle->config->conf[i].pattern[1];
                    for (j = 64; j < handle->config->conf[i].nSyncLen; j++) {
                        if (compare & (1ull << (j - 64))) {
                            ++nDifferences;
                        }
                    }

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
                            handle->frameLength = handle->config->conf[i].frameLength;
                            handle->rxCounter = handle->config->conf[i].frameLength;
                            handle->bitCounter = 0;
                            handle->bitFormat = handle->config->conf[i].bitFormat;
                            handle->inverted = handle->config->conf[i].inverted;
                            handle->uartBitCounter = 0;
                            handle->writeIndex = handle->config->conf[i].startOffset;
                            handle->postProcess = handle->config->conf[i].postProcess;
                            handle->state = 1;
                            ipc_s2m[handle->activeBuffer].opcode = 0;
                            ipc_s2m[handle->activeBuffer].param = handle->config->conf[i].id;
                        }
                    }
                }
                break;

            case 1:           /* Data reception */
                skipBit = false;

                if (handle->bitFormat == SYNC_BITFORMAT_UART_8N1) {
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
                }

                if (!skipBit) {
                    if (handle->inverted) {
                        bit = bit ^ 1;
                    }

                    if (handle->bitFormat == SYNC_BITFORMAT_UART_8N1) {
                        ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] =
                            (ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] >> 1) | (bit << 7);
                    }
                    else {
                        ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] =
                            (ipc_s2m[handle->activeBuffer].data8[handle->writeIndex] << 1) | bit;
                    }

                    if (++handle->bitCounter >= 8) {
                        handle->bitCounter = 0;
                        ++handle->writeIndex;
                        if (--handle->rxCounter <= 0) {
                            handle->state = 0;
                            handle->writeIndex = 0;
                            if (handle->postProcess) {
                                handle->postProcess(&ipc_s2m[handle->activeBuffer]);
                            }
                            ipc_s2m[handle->activeBuffer].valid = 1;

                            LPC_MAILBOX->IRQ1SET = (1u << 0);
                        }
                        else if (handle->rxCounter == handle->frameLength / 2) {
                            /* Tell M4 to keep current RSSI value */
                            LPC_MAILBOX->IRQ1SET = (1u << 1);
                        }
                    }
                }
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

