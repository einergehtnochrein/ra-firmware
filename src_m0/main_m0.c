
//#include <string.h>

#include "lpclib.h"
#include "sync_detect.h"

int currentMode;
volatile int newMode;
SYNC_Handle sync;


static const SYNC_Config configVaisala = {
    .nPatterns = 2,
    .conf = {
        {
            .id = 0,
            .nSyncLen = 116,
            .pattern = {0x9A6669A6669AA9A9LL, 0x0006669A6669A666LL},
            .nMaxDifference = 5,
            .frameLength = (234 * 10 * 2) / 8,
            .startOffset = 0,
        },
        {
            .id = 1,
            .nSyncLen = 40,
            .pattern = {0x000000884469481FLL, 0},
            .nMaxDifference = 3,
            .frameLength = 510,
            .startOffset = 0,
        },
    },
};


static const SYNC_Config configGraw = {
    .nPatterns = 2,
    .conf = {
        {
            .id = 2,
            .nSyncLen = 32,
            .pattern = {0x000000006566A5AALL, 0},
            .nMaxDifference = 2,
            .frameLength = 66,
            .startOffset = 0,
        },
        {
            .id = 3,
            .nSyncLen = 32,
            .pattern = {0x000000009A995A55LL, 0},
            .nMaxDifference = 2,
            .frameLength = 66,
            .startOffset = 0,
        },
    },
};


/* Add missing bytes (frame length) after frame reception */
static void _m10PostProcess100Normal (volatile IPC_S2M *buffer)
{
    buffer->data8[0] = 0xD4;
    buffer->data8[1] = 0xD3;
}
static void _m10PostProcess100Inverse (volatile IPC_S2M *buffer)
{
    buffer->data8[0] = 0x2B;
    buffer->data8[1] = 0x2C;
}


static const SYNC_Config configModem = {
    .nPatterns = 2,
    .conf = {
        /* The first byte after the sync word in an M10 frame contains the (remaining) frame length,
         * in this case 100. This byte is included in the sync pattern to increase the confidence in
         * the detection. The payload is therefore reduced to 99 bytes, and the M10 driver will add
         * the length byte before evaluating the checksum.
         *
         * Two patterns are checked: Normal and inverted.
         */
        {
            .id = 4,
            .nSyncLen = 48,
            .pattern = {0x0000CCCCA64CD4D3LL, 0},
            .nMaxDifference = 1,
            .frameLength = 100 * 2,
            .startOffset = 2,
            .postProcess = _m10PostProcess100Normal,
        },
        {
            .id = 4,
            .nSyncLen = 48,
            .pattern = {0x0000333359B32B2CLL, 0},
            .nMaxDifference = 1,
            .frameLength = 100 * 2,
            .startOffset = 2,
            .postProcess = _m10PostProcess100Inverse,
        },
    },
};


void MAILBOX_IRQHandler (void)
{
    uint32_t requests = LPC_MAILBOX->IRQ0;

    if (requests & 1) {
        newMode = 1;
        LPC_MAILBOX->IRQ0CLR = 1;
    }
    else if (requests & 2) {
        newMode = 2;
        LPC_MAILBOX->IRQ0CLR = 2;
    }
    else if (requests & 4) {
        newMode = 3;
        LPC_MAILBOX->IRQ0CLR = 4;
        NVIC_DisableIRQ(PIN_INT3_IRQn);
    }
    else if (requests & 8) {
        newMode = 4;
        LPC_MAILBOX->IRQ0CLR = 8;
    }
    else {
        /* No supported request. Clear them all */
        LPC_MAILBOX->IRQ0CLR = requests;
    }
}


int main (void)
{
    SystemCoreClock = 48000000ul;

    osKernelInitialize();
    osKernelStart();

    SYNC_open(&sync);

    NVIC_SetPriority(PIN_INT3_IRQn, 3);
    NVIC_EnableIRQ(MAILBOX_IRQn);
    NVIC_EnableIRQ(PIN_INT3_IRQn);

    while (1) {
        __WFI();

        if (newMode != currentMode) {
            //TODO configure detector
            switch (newMode) {
                case 1:
                    SYNC_configure(sync, &configVaisala);
                    NVIC_EnableIRQ(PIN_INT3_IRQn);
                    break;
                case 2:
                    SYNC_configure(sync, &configGraw);
                    NVIC_EnableIRQ(PIN_INT3_IRQn);
                    break;
                case 3:
                    break;
                case 4:
                    SYNC_configure(sync, &configModem);
                    NVIC_EnableIRQ(PIN_INT3_IRQn);
                    break;
            }

            currentMode = newMode;
        }
    }
}

