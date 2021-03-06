
#include "lpclib.h"
#include "sync_detect.h"


typedef enum {
    MODE_VAISALA = 1,
    MODE_GRAW = 2,
    MODE_AFSK = 3,  /* FSK sync detector inactive in this mode */
    MODE_MODEM_M10 = 4,
    MODE_MODEM_PILOTSONDE = 5,
    MODE_MEISEI = 6,
    MODE_JINYANG = 7,
    MODE_MRZ = 9,
} SYNC_Mode;


SYNC_Mode currentMode;
volatile SYNC_Mode newMode;
volatile bool resetSync;
SYNC_Handle sync;


static const SYNC_Config configVaisala = {
    .nPatterns = 2,
    .conf = {
        {
            .id = IPC_PACKET_TYPE_VAISALA_RS92,
            .pattern     = {0x9A6669A6669AA9A9LL, 0x0006669A6669A666LL},
            .patternMask = {0xFFFFFFFFFFFFFFFFLL, 0x000FFFFFFFFFFFFFLL},
            .nMaxDifference = 5,
            .frameLengthBits = 234 * 10 * 2,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = false,
        },
        {
            .id = IPC_PACKET_TYPE_VAISALA_RS41,
            .pattern     = {0x000000884469481FLL, 0},
            .patternMask = {0x000000FFFFFFFFFFLL, 0},
            .nMaxDifference = 3,
            .frameLengthBits = 510 * 8,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = false,
        },
    },
};


static const SYNC_Config configGraw = {
    .nPatterns = 2,
    .conf = {
        {
            .id = IPC_PACKET_TYPE_GRAW_INVERTED,
            .pattern     = {0x000000009A995A55LL, 0},
            .patternMask = {0x00000000FFFFFFFFLL, 0},
            .nMaxDifference = 2,
            .frameLengthBits = 66 * 8,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = true,
        },
        {
            .id = IPC_PACKET_TYPE_GRAW_NORMAL,
            .pattern     = {0x000000006566A5AALL, 0},
            .patternMask = {0x00000000FFFFFFFFLL, 0},
            .nMaxDifference = 2,
            .frameLengthBits = 66 * 8,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = false,
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
static void _m20PostProcess69Normal (volatile IPC_S2M *buffer)
{
    buffer->data8[0] = 0xD3;
    buffer->data8[1] = 0x2D;
}
static void _m20PostProcess69Inverse (volatile IPC_S2M *buffer)
{
    buffer->data8[0] = 0x2C;
    buffer->data8[1] = 0xD2;
}


static const SYNC_Config configModem = {
    .nPatterns = 4,
    .conf = {
        /* The first byte after the sync word in an M10 frame contains the (remaining) frame length,
         * in this case N=100 or N=69. This byte is included in the sync pattern to increase the confidence in
         * the detection. The payload is therefore reduced to N-1 bytes, and the M10 driver will add
         * the length byte before evaluating the checksum.
         *
         * Two patterns are checked: Normal and inverted.
         */
        {
            .id = IPC_PACKET_TYPE_MODEM_M10,
            .pattern     = {0x0000CCCCA64CD4D3LL, 0},
            .patternMask = {0x0000FFFFFFFFFFFFLL, 0},
            .nMaxDifference = 1,
            .frameLengthBits = 100 * 2 * 8,
            .startOffset = 2,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = false,
            .postProcess = _m10PostProcess100Normal,
        },
        {
            .id = IPC_PACKET_TYPE_MODEM_M10,
            .pattern     = {0x0000333359B32B2CLL, 0},
            .patternMask = {0x0000FFFFFFFFFFFFLL, 0},
            .nMaxDifference = 1,
            .frameLengthBits = 100 * 2 * 8,
            .startOffset = 2,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = false,
            .postProcess = _m10PostProcess100Inverse,
        },
        {
            .id = IPC_PACKET_TYPE_MODEM_M20,
            .pattern     = {0x0000CCCCA64CD32DLL, 0},
            .patternMask = {0x0000FFFFFFFFFFFFLL, 0},
            .nMaxDifference = 0,
            .frameLengthBits = 69 * 2 * 8,
            .startOffset = 2,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = false,
            .postProcess = _m20PostProcess69Normal,
        },
        {
            .id = IPC_PACKET_TYPE_MODEM_M20,
            .pattern     = {0x0000333359B32CD2LL, 0},
            .patternMask = {0x0000FFFFFFFFFFFFLL, 0},
            .nMaxDifference = 0,
            .frameLengthBits = 69 * 2 * 8,
            .startOffset = 2,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = false,
            .postProcess = _m20PostProcess69Inverse,
        },
    },
};


static const SYNC_Config configModemPilot = {
    .nPatterns = 2,
    .conf = {
        {
            .id = IPC_PACKET_TYPE_MODEM_PILOT,
            .pattern     = {0x00000000354D52FELL, 0},
            .patternMask = {0x000000003FFFFFFFLL, 0},
            .nMaxDifference = 1,
            .frameLengthBits = (50 - 4) * 8,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_UART_8N1,
            .inverted = true,
            .postProcess = NULL,
        },
        {
            .id = IPC_PACKET_TYPE_MODEM_PILOT,
            .pattern     = {0x000000000AB2AD01LL, 0},
            .patternMask = {0x000000003FFFFFFFLL, 0},
            .nMaxDifference = 1,
            .frameLengthBits = (50 - 4) * 8,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_UART_8N1,
            .inverted = false,
            .postProcess = NULL,
        },
    },
};


static const SYNC_Config configMeisei = {
    .nPatterns = 4,
    .conf = {
        {
            .id = IPC_PACKET_TYPE_MEISEI_CONFIG,
            .pattern     = {0x0000AAB52B34CACDLL, 0},
            .patternMask = {0x0000FFFFFFFFFFFFLL, 0},
            .nMaxDifference = 0,
            .frameLengthBits = 6 * 46,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_BIPHASE_S,
            .inverted = false,
            .nSubBlockBits = 46,
            .nSubBlockBytes = 8,
        },
        {
            .id = IPC_PACKET_TYPE_MEISEI_CONFIG,
            .pattern     = {0x0000554AD4CB3532LL, 0},
            .patternMask = {0x0000FFFFFFFFFFFFLL, 0},
            .nMaxDifference = 0,
            .frameLengthBits = 6 * 46,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_BIPHASE_S,
            .inverted = false,
            .nSubBlockBits = 46,
            .nSubBlockBytes = 8,
        },
        {
            .id = IPC_PACKET_TYPE_MEISEI_GPS,
            .pattern     = {0x0000CCD34D52ACAALL, 0},
            .patternMask = {0x0000FFFFFFFFFFFFLL, 0},
            .nMaxDifference = 2,
            .frameLengthBits = 6 * 46,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_BIPHASE_S,
            .inverted = false,
            .nSubBlockBits = 46,
            .nSubBlockBytes = 8,
        },
        {
            .id = IPC_PACKET_TYPE_MEISEI_GPS,
            .pattern     = {0x0000332CB2AD5355LL, 0},
            .patternMask = {0x0000FFFFFFFFFFFFLL, 0},
            .nMaxDifference = 2,
            .frameLengthBits = 6 * 46,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_BIPHASE_S,
            .inverted = false,
            .nSubBlockBits = 46,
            .nSubBlockBytes = 8,
        },
    },
};


static const SYNC_Config configJinyang = {
    .nPatterns = 2,
    .conf = {
        {
            .id = IPC_PACKET_TYPE_JINYANG_RSG20,
            .pattern     = {0x00AAAAAA88888888LL, 0},
            .patternMask = {0x00FFFFFFFFFFFFFFLL, 0},
            .nMaxDifference = 0,
            .frameLengthBits = 320,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = false,
        },
        {
            .id = IPC_PACKET_TYPE_JINYANG_RSG20,
            .pattern     = {0x0055555577777777LL, 0},
            .patternMask = {0x00FFFFFFFFFFFFFFLL, 0},
            .nMaxDifference = 0,
            .frameLengthBits = 320,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = true,
        },
    },
};


static const SYNC_Config configMrz = {
    .nPatterns = 2,
    .conf = {
        {
            .id = IPC_PACKET_TYPE_MRZ,
            .pattern     = {0x000066666555A599LL, 0},
            .patternMask = {0x0000FFFFFFFFFFFFLL, 0},
            .nMaxDifference = 1,
            .frameLengthBits = 376,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_MANCHESTER,
            .inverted = false,
        },
        {
            .id = IPC_PACKET_TYPE_MRZ,
            .pattern     = {0x000099999AAA5A66LL, 0},
            .patternMask = {0x0000FFFFFFFFFFFFLL, 0},
            .nMaxDifference = 1,
            .frameLengthBits = 376,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_MANCHESTER,
            .inverted = true,
        },
    },
};



void MAILBOX_IRQHandler (void)
{
    uint32_t requests = LPC_MAILBOX->IRQ0;

    if (requests & (1u << 0)) {
        newMode = MODE_VAISALA;
        LPC_MAILBOX->IRQ0CLR = 1;
    }
    else if (requests & (1u << 1)) {
        newMode = MODE_GRAW;
        LPC_MAILBOX->IRQ0CLR = 2;
    }
    else if (requests & (1u << 2)) {
        newMode = MODE_AFSK;
        LPC_MAILBOX->IRQ0CLR = 4;
        NVIC_DisableIRQ(PIN_INT3_IRQn);
    }
    else if (requests & (1u << 3)) {
        newMode = MODE_MODEM_M10;
        LPC_MAILBOX->IRQ0CLR = 8;
    }
    else if (requests & (1u << 4)) {
        newMode = MODE_MODEM_PILOTSONDE;
        LPC_MAILBOX->IRQ0CLR = (1u << 4);
    }
    else if (requests & (1u << 5)) {
        newMode = MODE_MEISEI;
        LPC_MAILBOX->IRQ0CLR = (1u << 5);
    }
    else if (requests & (1u << 6)) {
        newMode = MODE_JINYANG;
        LPC_MAILBOX->IRQ0CLR = (1u << 6);
    }
    else if (requests & (1u << 8)) {
        newMode = MODE_MRZ;
        LPC_MAILBOX->IRQ0CLR = (1u << 8);
    }
    else if (requests & (1u << 30)) {
        resetSync = true;
        LPC_MAILBOX->IRQ0CLR = (1u << 30);
    }
    else if (requests & (1u << 31)) {
        resetSync = false;
        LPC_MAILBOX->IRQ0CLR = (1u << 31);
    }
    else {
        /* No supported request. Clear them all */
        LPC_MAILBOX->IRQ0CLR = requests;
    }
}


int main (void)
{
    const SYNC_Config *syncConfig = NULL;
    
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
                case MODE_VAISALA:
                    syncConfig = &configVaisala;
                    break;
                case MODE_GRAW:
                    syncConfig = &configGraw;
                    break;
                case MODE_MODEM_M10:
                    syncConfig = &configModem;
                    break;
                case MODE_MODEM_PILOTSONDE:
                    syncConfig = &configModemPilot;
                    break;
                case MODE_MEISEI:
                    syncConfig = &configMeisei;
                    break;
                case MODE_JINYANG:
                    syncConfig = &configJinyang;
                    break;
                case MODE_MRZ:
                    syncConfig = &configMrz;
                    break;
                case MODE_AFSK:
                default:
                    syncConfig = NULL;
                    break;
            }

            currentMode = newMode;
        }

        if (resetSync) {
            NVIC_DisableIRQ(PIN_INT3_IRQn);
            SYNC_configure(sync, NULL);
        }
        else {
            SYNC_configure(sync, syncConfig);
            NVIC_EnableIRQ(PIN_INT3_IRQn);
        }
    }
}

