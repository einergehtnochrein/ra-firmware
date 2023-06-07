
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
    MODE_WINDSOND = 8,
    MODE_MRZ = 9,
    MODE_ASIA1 = 10,    /* Various sondes (mostly Asia) with 2FSK 2400 sym/s */
    MODE_PSB3 = 11,
    MODE_IMET54 = 12,
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
            .frameLengthBits = 234 * 8,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_MANCHESTER_UART_8N1,
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
            .frameLengthBits = 33 * 8,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_MANCHESTER,
            .inverted = false,
        },
        {
            .id = IPC_PACKET_TYPE_GRAW_NORMAL,
            .pattern     = {0x000000006566A5AALL, 0},
            .patternMask = {0x00000000FFFFFFFFLL, 0},
            .nMaxDifference = 2,
            .frameLengthBits = 33 * 8,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_MANCHESTER,
            .inverted = true,
        },
    },
};


static const SYNC_Config configModem = {
    .nPatterns = 1,
    .conf = {
        {
            .id = IPC_PACKET_TYPE_MODEM,
            .pattern     = {0x00000000CCCCA64CLL, 0},
            .patternMask = {0x00000000FFFFFFFFLL, 0},
            .nMaxDifference = 0,
            .frameLengthBits = 70 * 8,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_BIPHASE_M,
            .inverted = false,
            .byteProcess = _SYNC_modemByteProcess,
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
    .nPatterns = 1,
    .conf = {
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


static void _gth3_cf06_postProcess (SYNC_Handle handle, volatile IPC_S2M *buffer, int writeIndex)
{
    (void)handle;
    (void)writeIndex;

    // Restore 1st payload byte which was used to extend the sync word
    buffer->data8[0] = 0x63;
}


static const SYNC_Config configAsia1 = {
    .nPatterns = 4,
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
            .id = IPC_PACKET_TYPE_HT03G_CF06AH,
            .pattern     = {0x00005555B42BC6LL, 0},
            .patternMask = {0x0000FFFFFFFFFFLL, 0},
            .nMaxDifference = 0,
            .frameLengthBits = 102 * 8,
            .startOffset = 1,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = false,
            .lsbFirst = true,
            .postProcess = _gth3_cf06_postProcess,
        },
    },
};


static const SYNC_Config configPSB3 = {
    .nPatterns = 1,
    .conf = {
        {
            .id = IPC_PACKET_TYPE_VIKRAM_PSB3,
            .pattern     = {0xAA969965599A9A56LL, 0},
            .patternMask = {0x0FFFFFFFFFFFFFFFLL, 0},
            .nMaxDifference = 1,
            .frameLengthBits = 8*44,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_MANCHESTER,
            .inverted = true,
        },
    },
};


static const SYNC_Config configIMET54 = {
    .nPatterns = 1,
    .conf = {
        {
            .id = IPC_PACKET_TYPE_IMET54,
            .pattern     = {0x0005551244912449LL, 0},
            .patternMask = {0x0FFFFFFFFFFFFFFFLL, 0},
            .nMaxDifference = 1,
            .frameLengthBits = 8*(1+432),
            .dataState = SYNC_STATE_DATA_UART_8N1,
            .inverted = false,
        },
    },
};


static const SYNC_Config configWindsond = {
    .nPatterns = 1,
    .conf = {
        {
            .id = IPC_PACKET_TYPE_WINDSOND_S1,
            .pattern     = {0x000000005555552DLL, 0},
            .patternMask = {0x00000000FFFFFFFFLL, 0},
            .nMaxDifference = 0,
            .frameLengthBits = 512,
            .startOffset = 0,
            .dataState = SYNC_STATE_DATA_RAW,
            .inverted = false,
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
    else if (requests & (1u << 7)) {
        newMode = MODE_WINDSOND;
        LPC_MAILBOX->IRQ0CLR = (1u << 7);
    }
    else if (requests & (1u << 8)) {
        newMode = MODE_MRZ;
        LPC_MAILBOX->IRQ0CLR = (1u << 8);
    }
    else if (requests & (1u << 9)) {
        newMode = MODE_ASIA1;
        LPC_MAILBOX->IRQ0CLR = (1u << 9);
    }
    else if (requests & (1u << 10)) {
        newMode = MODE_PSB3;
        LPC_MAILBOX->IRQ0CLR = (1u << 10);
    }
    else if (requests & (1u << 11)) {
        newMode = MODE_IMET54;
        LPC_MAILBOX->IRQ0CLR = (1u << 11);
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
                case MODE_WINDSOND:
                    syncConfig = &configWindsond;
                    break;
                case MODE_MRZ:
                    syncConfig = &configMrz;
                    break;
                case MODE_ASIA1:
                    syncConfig = &configAsia1;
                    break;
                case MODE_PSB3:
                    syncConfig = &configPSB3;
                    break;
                case MODE_IMET54:
                    syncConfig = &configIMET54;
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

