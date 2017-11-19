/* Copyright (c) 2016, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 * Neither the name of the author nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"

#include "adf7021.h"
#include "app.h"
#include "dfm.h"
#include "imet.h"
#include "m10.h"
#include "pdm.h"
#include "rinex.h"
#include "rs41.h"
#include "rs92.h"
#include "srsc.h"
#include "sys.h"
#include "config.h"


/** \file
 *  \brief System management task.
 */


#define SYS_QUEUE_LENGTH                        (10)

typedef struct {
    uint8_t opcode;
    union {
        LPCLIB_Event event;
        int bufferIndex;
    };
} SYS_Message;


/** Message opcodes for SYS task. */
enum {
    SYS_OPCODE_EVENT,
    SYS_OPCODE_BUFFER_COMPLETE,
    SYS_OPCODE_GET_RSSI,
    SYS_OPCODE_GET_OFFSET,
};


/** Identifiers for OS timers. */
enum {
    SYS_TIMERMAGIC_RSSI,
    SYS_TIMERMAGIC_INACTIVITY,
};


/********** MUST BE DEFINED SAME AS IN M0 PROJECT ***********/
#define IPC_S2M_DATA_SIZE                   1024
#define IPC_S2M_NUM_BUFFERS                 4

typedef struct {
    volatile uint8_t valid;
    uint8_t opcode;
    uint16_t param;

    uint8_t data8[IPC_S2M_DATA_SIZE];
} IPC_S2M;

__SECTION(".ipc")
static IPC_S2M ipc[IPC_S2M_NUM_BUFFERS];
/************************************************************/



#define COMMAND_LINE_SIZE   400

#define INACTIVITY_TIMEOUT  20000


struct SYS_Context {
    struct pt pt;

    osMailQId queue;                                    /**< Task message queue */
    osEvent rtosEvent;
    osTimerId rssiTick;
    osTimerId inactivityTimeout;

#if SEMIHOSTING
    FILE *fpLog;
#endif

    RS41_Handle rs41;
    RS92_Handle rs92;
    DFM_Handle dfm;
    IMET_Handle imet;
    M10_Handle m10;
    SRSC_Handle srsc;
    PDM_Handle pdm;

    _Bool sleeping;                                     /**< Low-power mode activated */
    LPCLIB_Callback callback;                           /**< Callback for system events */

    SONDE_Type sondeType;
    SONDE_Detector sondeDetector;
    uint32_t currentFrequencyHz;
    float lastInPacketRssi;                             /**< Last RSSI measurement while data reception was still active */
    float packetOffsetKhz;                              /**< Frequency offset at end of sync word */
    float vbat;

    char commandLine[COMMAND_LINE_SIZE];

    int securityResponse;                               /* Response expected to security challenge */
} sysContext;



#if (BOARD_RA == 1)
static const GPIO_Config buttonInit[] = {
    {.opcode = GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT,
        {.pinInterrupt = {
            .pin = GPIO_BUTTON,
            .enable = LPCLIB_YES,
            .mode = GPIO_INT_FALLING_EDGE,
            .interruptLine = GPIO_PIN_INT_0,
            .callback = SYS_handleEvent, }}},

    GPIO_CONFIG_END
};

/* UART RXD line for wakeup */
static const GPIO_Config uartRxdWakeupInit[] = {
    {.opcode = GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT,
        {.pinInterrupt = {
            .pin = GPIO_BLE_RXD,
            .enable = LPCLIB_YES,
            .mode = GPIO_INT_FALLING_EDGE,
            .interruptLine = GPIO_PIN_INT_4,
            .callback = SYS_handleEvent, }}},

    GPIO_CONFIG_END
};
#endif


#if (BOARD_RA == 2)
/* UART RXD line for wakeup */
static const GPIO_Config uartRxdWakeupInit[] = {
    {.opcode = GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT,
        {.pinInterrupt = {
            .pin = GPIO_BLE_RXD,
            .enable = LPCLIB_YES,
            .mode = GPIO_INT_FALLING_EDGE,
            .interruptLine = GPIO_PIN_INT_4,
            .callback = SYS_handleEvent, }}},

    GPIO_CONFIG_END
};
#endif



static uint32_t _SYS_getSondeBufferLength (SONDE_Type type)
{
    uint32_t length = 2;

    switch (type) {
        case SONDE_RS41:
            length = 510;
            break;
        case SONDE_RS92:
            length = 585;
            break;
        case SONDE_DFM06:
        case SONDE_DFM09:
        case SONDE_PS15:
            length = 66;
            break;
        case SONDE_M10:
            length = (100+1) * 2;
            break;
        case SONDE_C34:
        case SONDE_C50:
        case SONDE_IMET_RSB:
            /* Doesn't use SPI */
            break;
        case SONDE_UNDEFINED:
            /* Nothing to do */
            break;
    }

    return length;
}



//TODO need a mailbox driver
//TODO do all the sonde type modifications on process level!
void MAILBOX_IRQHandler (void)
{
    SYS_Message *pMessage;
    uint32_t requests = LPC_MAILBOX->IRQ1;

    if (requests & 1) {
        /* Find valid mailbox */
        int i;
        for (i = 0; i < IPC_S2M_NUM_BUFFERS; i++) {
            if (ipc[i].valid) {
                if (ipc[i].opcode == 0) {
                    pMessage = osMailAlloc(sysContext.queue, 0);
                    if (pMessage) {
                        pMessage->opcode = SYS_OPCODE_BUFFER_COMPLETE;
                        pMessage->bufferIndex = i;
                    }
                }
            }
        }
        LPC_MAILBOX->IRQ1CLR = (1u << 0);
    }
    else {
        /* No supported request. Clear them all */
        LPC_MAILBOX->IRQ1CLR = requests;
    }
}



static void _SYS_reportRadioFrequency (SYS_Handle handle)
{
    char s[20];
    sprintf(s, "1,%.3f", handle->currentFrequencyHz / 1e6f);
    SYS_send2Host(HOST_CHANNEL_GUI, s);
}



static void _SYS_setRadioFrequency (SYS_Handle handle, uint32_t frequencyHz)
{
    if (frequencyHz < 400000000) {
        frequencyHz = 400000000;
    }
    if (frequencyHz > 406100000) {
        frequencyHz = 406100000;
    }

    ADF7021_setPLL(radio, frequencyHz - 100000);

    handle->currentFrequencyHz = frequencyHz;
}



/* Get current RX frequency */
uint32_t SYS_getCurrentFrequencyHz (SYS_Handle handle)
{
    return handle->currentFrequencyHz;
}



/* Enable the receiver, select a frequency and a sonde decoder type.
 */
LPCLIB_Result SYS_enableDetector (SYS_Handle handle, uint32_t frequencyHz, SONDE_Detector detector)
{
    if ((detector != handle->sondeDetector) || (frequencyHz != handle->currentFrequencyHz)) {
        PDM_stop(handle->pdm);

        handle->sondeDetector = detector;

        ADF7021_calibrateIF(radio, 1);  //TODO coarse/fine

        switch (detector) {
        case SONDE_DETECTOR_C34_C50:
            ADF7021_write(radio, ADF7021_REGISTER_0, 0
                            | (1u << 28)        /* UART/SPI mode */
                            | (1u << 27)        /* Receive mode */
                            );
            ADF7021_write(radio, ADF7021_REGISTER_1, 0
                            | (1u << 25)        /* External L */
                            | (3u << 19)        /* VCO bias = 0.75 mA */
                            | (1u << 17)        /* VCO on */
                            | (1u << 12)        /* XOSC on */
                            | (0u << 7)         /* CLKOUT off */
                            | (1u << 4)         /* R = 1 */
                            );
            osDelay(10);
            ADF7021_write(radio, ADF7021_REGISTER_3, 0
                            | (12u << 26)       /* AGC_CLK_DIVIDE = 12 --> AGCUpdateRate = 8,3 kHz (8 kHz) */
                            | (130u << 18)      /* SEQ_CLK_DIVIDE = 130 --> SEQCLK = 100 kHz (100 kHz) */
#if (BOARD_RA == 1)
                            | (1u << 10)        /* CDR_CLK_DIVIDE = 1 --> CDRCLK = 2.1667 MHz (=DEMODCLK) */
                            | (6u << 6)         /* DEMOD_CLK_DIVIDE = 6 --> DEMODCLK = 2.1667 MHz (2...15 MHz) */
#endif
#if (BOARD_RA == 2)
                            | (1u << 10)        /* CDR_CLK_DIVIDE = 1 --> CDRCLK = 3.2 MHz (=DEMODCLK) */
                            | (4u << 6)         /* DEMOD_CLK_DIVIDE = 4 --> DEMODCLK = 3.2 MHz (2...15 MHz) */
#endif
                            | (1u << 4)         /* BBOS_CLK_DIVIDE = 8 --> BBOSCLK = 1.625 MHz (1...2 MHz) */
                            );
            _SYS_setRadioFrequency(handle, frequencyHz);
            _SYS_reportRadioFrequency(handle);  /* Inform host */
            // K=42
            ADF7021_write(radio, ADF7021_REGISTER_4, 0
                            | (2u << 30)        /* IF_FILTER_BW = 18.5 kHz */
                            | (30u << 20)       /* POST_DEMOD_BW, assuming fcutoff = 10000 Hz */
                                                //TODO: post demod filter seems to have half that value (5k)
                            | (226u << 10)      /* DISCRIMINATOR_BW, assuming K = 42, fdev = 2.4 kHz */
                            | (2u << 8)         /* INVERT DATA */
                            | (0u << 7)         /* CROSS_PRODUCT */
                            | (0u << 4)         /* 2FSK linear demodulator */
                            );
            ADF7021_write(radio, ADF7021_REGISTER_10, 0
                            | (0u << 4)         /* AFC off */
                            );
            ADF7021_write(radio, ADF7021_REGISTER_15, 0
                            | (2u << 17)        /* CLKOUT pin carries CDR CLK */
                            | (0u << 11)        /* 3rd order sigma-delta, no dither */
                            | (9u << 4)         /* Enable REG14 modes */
                            );
            ADF7021_write(radio, ADF7021_REGISTER_14, 0
                            | (4 << 21)         /* Test DAC gain = ? dB */
#if (BOARD_RA == 1)
                            | (12098 << 5)      /* Test DAC offset (0...65535) */
#endif
#if (BOARD_RA == 2)
                            | (8192 << 5)       /* Test DAC offset (0...65535) */
#endif
                            | (1 << 4)          /* Enable Test DAC */
                            );

#if (BOARD_RA == 1)
            PDM_run(handle->pdm, 134, SRSC_handleAudioCallback);
#endif
#if (BOARD_RA == 2)
            PDM_run(handle->pdm, 200, SRSC_handleAudioCallback);
#endif
            LPC_MAILBOX->IRQ0SET = (1u << 2); //TODO
            break;

        case SONDE_DETECTOR_IMET:
            ADF7021_write(radio, ADF7021_REGISTER_0, 0
                            | (1u << 28)        /* UART/SPI mode */
                            | (1u << 27)        /* Receive mode */
                            );
            ADF7021_write(radio, ADF7021_REGISTER_1, 0
                            | (1u << 25)        /* External L */
                            | (3u << 19)        /* VCO bias = 0.75 mA */
                            | (1u << 17)        /* VCO on */
                            | (1u << 12)        /* XOSC on */
                            | (0u << 7)         /* CLKOUT off */
                            | (1u << 4)         /* R = 1 */
                            );
            osDelay(10);
            ADF7021_write(radio, ADF7021_REGISTER_3, 0
                            | (12u << 26)       /* AGC_CLK_DIVIDE = 12 --> AGCUpdateRate = 8,3 kHz (8 kHz) */
                            | (130u << 18)      /* SEQ_CLK_DIVIDE = 130 --> SEQCLK = 100 kHz (100 kHz) */
#if (BOARD_RA == 1)
                            | (1u << 10)        /* CDR_CLK_DIVIDE = 1 --> CDRCLK = 2.1667 MHz (=DEMODCLK) */
                            | (6u << 6)         /* DEMOD_CLK_DIVIDE = 6 --> DEMODCLK = 2.1667 MHz (2...15 MHz) */
#endif
#if (BOARD_RA == 2)
                            | (1u << 10)        /* CDR_CLK_DIVIDE = 1 --> CDRCLK = 3.2 MHz (=DEMODCLK) */
                            | (4u << 6)         /* DEMOD_CLK_DIVIDE = 4 --> DEMODCLK = 3.2 MHz (2...15 MHz) */
#endif
                            | (1u << 4)         /* BBOS_CLK_DIVIDE = 8 --> BBOSCLK = 1.625 MHz (1...2 MHz) */
                            );
            _SYS_setRadioFrequency(handle, frequencyHz);
            _SYS_reportRadioFrequency(handle);  /* Inform host */
            // K=42
            ADF7021_write(radio, ADF7021_REGISTER_4, 0
                            | (2u << 30)        /* IF_FILTER_BW = 18.5 kHz */
                            | (30u << 20)       /* POST_DEMOD_BW, assuming fcutoff = 10000 Hz */
                                                //TODO: post demod filter seems to have half that value (5k)
                            | (226u << 10)      /* DISCRIMINATOR_BW, assuming K = 42, fdev = 2.4 kHz */
                            | (2u << 8)         /* INVERT DATA */
                            | (0u << 7)         /* CROSS_PRODUCT */
                            | (0u << 4)         /* 2FSK linear demodulator */
                            );
            ADF7021_write(radio, ADF7021_REGISTER_10, 0
                            | (0u << 4)         /* AFC off */
                            );
            ADF7021_write(radio, ADF7021_REGISTER_15, 0
                            | (2u << 17)        /* CLKOUT pin carries CDR CLK */
                            | (0u << 11)        /* 3rd order sigma-delta, no dither */
                            | (9u << 4)         /* Enable REG14 modes */
                            );
            ADF7021_write(radio, ADF7021_REGISTER_14, 0
                            | (4 << 21)         /* Test DAC gain = ? dB */
#if (BOARD_RA == 1)
                            | (12098 << 5)      /* Test DAC offset (0...65535) */
#endif
#if (BOARD_RA == 2)
                            | (8192 << 5)       /* Test DAC offset (0...65535) */
#endif
                            | (1 << 4)          /* Enable Test DAC */
                            );

#if (BOARD_RA == 1)
            PDM_run(handle->pdm, 134, IMET_handleAudioCallback);
#endif
#if (BOARD_RA == 2)
            PDM_run(handle->pdm, 200, IMET_handleAudioCallback);
#endif
            LPC_MAILBOX->IRQ0SET = (1u << 2); //TODO
            break;

        case SONDE_DETECTOR_DFM:
            ADF7021_write(radio, ADF7021_REGISTER_0,  (1u << 28) | (1u << 27)); /* UART/SPI mode (see also register 15), RX */
            ADF7021_write(radio, ADF7021_REGISTER_14, (0 << 30) | (0 << 27) | (0 << 25) | (0 << 21) | (0 << 5) | (0 << 4)); /* ED_PEAK_RESPONSE=0, ED_LEAK_FACTOR=0 */
            ADF7021_write(radio, ADF7021_REGISTER_15, (7u << 17) | (9u << 4)); /* CLKOUT=TxRxCLK, Rx_TEST_MODES=9 */
            ADF7021_write(radio, ADF7021_REGISTER_1,  (1u << 25) | (3u << 19) | (1u << 17) | (1u << 12) | (1u << 4)); /* External L, VCObias=0.75mA, VCO=on, XOSC=on, CLKOUT=off, R=1 */
            osDelay(10);
#if (BOARD_RA == 1)
            ADF7021_write(radio, ADF7021_REGISTER_3, 0
                            | (12u << 26)   /* AGC_CLK_DIVIDE=12 --> AGC_rate=8.333 kHz */
                            | (130u << 18)  /* SEQ_CLK_DIVIDE=130 --> SEQ_CLK=100 kHz */
                            | (54u << 10)   /* CDR_CLK_DIVIDE=54 --> CDR_CLK=80.2469 kHz (+0.3%) */
                            | (3u << 6)     /* DEMOD_CLK_DIVIDE=3 --> DEMOD_CLK=4.3333 MHz */
                            | (1u << 4)     /* BBOSdivide=8 --> BBOS_CLK=1.625 MHz */
                            );
            // K=42
            ADF7021_write(radio, ADF7021_REGISTER_4,  (1u << 30) | (4u << 20) | (342u << 10) | (2u << 8) | (0u << 7) | (1u << 4)); /* IF=13.5kHz, 2FSK correlator */
#endif
#if (BOARD_RA == 2)
            ADF7021_write(radio, ADF7021_REGISTER_3, 0
                            | (12u << 26)   /* AGC_CLK_DIVIDE=12 --> AGC_rate=8.333 kHz */
                            | (128u << 18)  /* SEQ_CLK_DIVIDE=128 --> SEQ_CLK=100 kHz */
                            | (40u << 10)   /* CDR_CLK_DIVIDE=40 --> CDR_CLK=80.0000 kHz (0%) */
                            | (4u << 6)     /* DEMOD_CLK_DIVIDE=4 --> DEMOD_CLK=3.2 MHz */
                            | (1u << 4)     /* BBOSdivide=8 --> BBOS_CLK=1.6 MHz */
                            );
            // K=42
            // TODO Data NOT inverted. Does data polarity depend on DEMOD_CLK_DIVIDE?
            ADF7021_write(radio, ADF7021_REGISTER_4,  (1u << 30) | (4u << 20) | (342u << 10) | (0u << 8) | (0u << 7) | (1u << 4)); /* IF=13.5kHz, 2FSK correlator */
#endif
            _SYS_setRadioFrequency(handle, frequencyHz);
            _SYS_reportRadioFrequency(handle);  /* Inform host */
            ADF7021_write(radio, ADF7021_REGISTER_10, (20u << 24) | (2u << 21) | (11u << 17) | (645u << 5) | (1u << 4)); /* MAX_AFC_RANGE=20 (+/-5 kHz), KP=2, KI=11, AFC_SCALING_FACTOR=645, AFC_EN=1 */

            LPC_MAILBOX->IRQ0SET = (1u << 1); //TODO
            break;

        case SONDE_DETECTOR_RS41_RS92:
            ADF7021_write(radio, ADF7021_REGISTER_0, 0
                            | (1u << 28)
                            | (1u << 27)
                            ); /* UART/SPI mode (see also register 15), RX */
            ADF7021_write(radio, ADF7021_REGISTER_14, 0
                            | (0 << 30)
                            | (0 << 27)
                            | (0 << 25)
                            | (0 << 21)
                            | (0 << 5)
                            | (0 << 4)
                            ); /* ED_PEAK_RESPONSE=0, ED_LEAK_FACTOR=0 */
            ADF7021_write(radio, ADF7021_REGISTER_15, 0
                            | (7u << 17)
                            | (9u << 4)
                            ); /* CLKOUT=TxRxCLK, Rx_TEST_MODES=9 */
            ADF7021_write(radio, ADF7021_REGISTER_1, 0
                            | (1u << 25)
                            | (3u << 19)
                            | (1u << 17)
                            | (1u << 12)
                            | (1u << 4)
                            ); /* External L, VCObias=0.75mA, VCO=on, XOSC=on, CLKOUT=off, R=1 */
            osDelay(10);
#if (BOARD_RA == 1)
            ADF7021_write(radio, ADF7021_REGISTER_3, 0
                            | (12u << 26)   /* AGC_CLK_DIVIDE=12 --> AGC_rate=8.333 kHz */
                            | (130u << 18)  /* SEQ_CLK_DIVIDE=130 --> SEQ_CLK=100 kHz */
                            | (21u << 10)   /* CDR_CLK_DIVIDE=21 --> CDR_CLK=154.7619 kHz (+0.75%) */
                            | (4u << 6)     /* DEMOD_CLK_DIVIDE=4 --> DEMOD_CLK=3.25 MHz */
                            | (1u << 4)     /* BBOSdivide=8 --> BBOS_CLK=1.625 MHz */
                            );
#endif
#if (BOARD_RA == 2)
            ADF7021_write(radio, ADF7021_REGISTER_3, 0
                            | (12u << 26)   /* AGC_CLK_DIVIDE=12 --> AGC_rate=8.333 kHz */
                            | (128u << 18)  /* SEQ_CLK_DIVIDE=128 --> SEQ_CLK=100 kHz */
                            | (21u << 10)   /* CDR_CLK_DIVIDE=21 --> CDR_CLK=152.3810 kHz (-0.8%) */
                            | (4u << 6)     /* DEMOD_CLK_DIVIDE=4 --> DEMOD_CLK=3.2 MHz */
                            | (1u << 4)     /* BBOSdivide=8 --> BBOS_CLK=1.6 MHz */
                            );
#endif
            _SYS_setRadioFrequency(handle, frequencyHz);
            _SYS_reportRadioFrequency(handle);  /* Inform host */
            // K=42
            ADF7021_write(radio, ADF7021_REGISTER_4, 0
                            | (1u << 30)    //0=9.5k, 1=13.5k, 2=18.5k
                            | (8u << 20)
                            | (342u << 10)
                            | (2u << 8)
                            | (0u << 7)
                            | (1u << 4)
                            ); /* IF=18.5kHz, 2FSK correlator */
            ADF7021_write(radio, ADF7021_REGISTER_10, 0
                            | (20u << 24)
                            | (4u << 21)    // =2?
                            | (11u << 17)
                            | (645u << 5)
                            | (1u << 4)
                            ); /* MAX_AFC_RANGE=20 (+/-10 kHz), KP=4, KI=11, AFC_SCALING_FACTOR=645, AFC_EN=1 */

            LPC_MAILBOX->IRQ0SET = (1u << 0); //TODO
            break;

        case SONDE_DETECTOR_MODEM:
            ADF7021_write(radio, ADF7021_REGISTER_0, 0
                            | (1u << 28)
                            | (1u << 27)
                            ); /* UART/SPI mode (see also register 15), RX */
            ADF7021_write(radio, ADF7021_REGISTER_14, 0
                            | (0 << 30)
                            | (0 << 27)
                            | (0 << 25)
                            | (0 << 21)
                            | (0 << 5)
                            | (0 << 4)
                            ); /* ED_PEAK_RESPONSE=0, ED_LEAK_FACTOR=0 */
            ADF7021_write(radio, ADF7021_REGISTER_15, 0
                            | (7u << 17)
                            | (9u << 4)
                            ); /* CLKOUT=TxRxCLK, Rx_TEST_MODES=9 */
            ADF7021_write(radio, ADF7021_REGISTER_1, 0
                            | (1u << 25)
                            | (3u << 19)
                            | (1u << 17)
                            | (1u << 12)
                            | (1u << 4)
                            ); /* External L, VCObias=0.75mA, VCO=on, XOSC=on, CLKOUT=off, R=1 */
            osDelay(10);
#if (BOARD_RA == 1)
            ADF7021_write(radio, ADF7021_REGISTER_3, 0
                            | (12u << 26)   /* AGC_CLK_DIVIDE=12 --> AGC_rate=8.333 kHz */
                            | (130u << 18)  /* SEQ_CLK_DIVIDE=130 --> SEQ_CLK=100 kHz */
                            | (14u << 10)   /* CDR_CLK_DIVIDE=14 --> CDR_CLK=309.5238 kHz (+0.75%) */
                            | (3u << 6)     /* DEMOD_CLK_DIVIDE=3 --> DEMOD_CLK=4.3333 MHz */
                            | (1u << 4)     /* BBOSdivide=8 --> BBOS_CLK=1.625 MHz */
                            );
#endif
#if (BOARD_RA == 2)
            ADF7021_write(radio, ADF7021_REGISTER_3, 0
                            | (12u << 26)   /* AGC_CLK_DIVIDE=12 --> AGC_rate=8.333 kHz */
                            | (128u << 18)  /* SEQ_CLK_DIVIDE=128 --> SEQ_CLK=100 kHz */
                            | (14u << 10)   /* CDR_CLK_DIVIDE=14 --> CDR_CLK=304.7619 kHz (-0.8%) */
                            | (3u << 6)     /* DEMOD_CLK_DIVIDE=3 --> DEMOD_CLK=4.2667 MHz */
                            | (1u << 4)     /* BBOSdivide=8 --> BBOS_CLK=1.6 MHz */
                            );
#endif
            _SYS_setRadioFrequency(handle, frequencyHz);
            _SYS_reportRadioFrequency(handle);  /* Inform host */
            // K=42
            ADF7021_write(radio, ADF7021_REGISTER_4, 0
                            | (2u << 30)    //0=9.5k, 1=13.5k, 2=18.5k
                            | (11u << 20)
                            | (448u << 10)
                            | (2u << 8)
                            | (0u << 7)
//                            | (1u << 4)
                            ); /* IF=18.5kHz, 2FSK correlator */  //TODO uses linear demod as workaround
            ADF7021_write(radio, ADF7021_REGISTER_10, 0
                            | (20u << 24)
                            | (4u << 21)    // =2?
                            | (11u << 17)
                            | (645u << 5)
                            | (1u << 4)
                            ); /* MAX_AFC_RANGE=20 (+/-10 kHz), KP=4, KI=11, AFC_SCALING_FACTOR=645, AFC_EN=1 */

            LPC_MAILBOX->IRQ0SET = (1u << 3); //TODO
            break;

        default:
            // Disable sync detector
            //TODO
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


/* Send a message to the host.
 * Formatter adds channel ID and line feed to the message string.
 */
LPCLIB_Result SYS_send2Host (int channel, const char *message)
{
    char s[20];
    int checksum = 0;
    int i;

    //TODO
    /* Check for enough room in TX FIFO */
    if (UART_getTxFree(blePort) < (int)strlen(message) + 10) {
        return LPCLIB_BUSY;
    }

    sprintf(s, "#%d,", channel);
    for (i = 0; i < (int)strlen(s); i++) {
        checksum += s[i];
    }
    UART_write(blePort, s, strlen(s));

    for (i = 0; i < (int)strlen(message); i++) {
        checksum += message[i];
    }
    UART_write(blePort, (uint8_t *)message, strlen(message));

    checksum += ',';
    sprintf(s, ",%d\r", checksum % 100);
    UART_write(blePort, s, strlen(s));

    return LPCLIB_SUCCESS;
}



/** Enter low-power mode */
static void SYS_sleep (SYS_Handle handle)
{
    uint32_t OldSystemCoreClock;


    SCANNER_setScannerMode(scanner, false);

    handle->sleeping = true;

    /* Enable BLE RXD line interrupt (allow pending interrupt to happen now) */
    NVIC_EnableIRQ(PIN_INT4_IRQn);

    /* All GPIO pins to lowest power mode. */
    BSP_prepareSleep();

    /* Remember system clock frequency */
    OldSystemCoreClock = SystemCoreClock;

#if (BOARD_RA == 1)
    CLKPWR_setCpuClock(12000000ul, CLKPWR_CLOCK_IRC);

    /* Wakeup source: UART RXD line (pin interrupt 4) */
    LPC_SYSCON->STARTERSET1 = (1u << 1);    // PINT4 wakeup

    /* Disable BOD reset */
    LPC_SYSCONEXTRA->BODCTRL = (LPC_SYSCONEXTRA->BODCTRL & ~(1u << 2)) | (1u << 6);
#endif
#if (BOARD_RA == 2)
    CLKPWR_setCpuClock(12000000ul, CLKPWR_CLOCK_FRO12);

    /* Wakeup source: UART RXD line (pin interrupt 4) */
    LPC_SYSCON->STARTERSET1 = (1u << 0);    // PINT4 wakeup

    /* Disable BOD reset */
    LPC_SYSCONEXTRA->BODCTRL = (LPC_SYSCONEXTRA->BODCTRL & ~(1u << 2)) | (1u << 6);
#endif

    /* Enter Power Down mode, and wait for wake-up. */
    //NOTE: Requires privileges! That's why RTX must run with privileged tasks for now
    CLKPWR_enterPowerSaving(CLKPWR_POWERSAVING_DEEPSLEEP);

    NVIC_DisableIRQ(PIN_INT4_IRQn);

    /* Re-enable the PLL */
#if (BOARD_RA == 1)
    CLKPWR_setCpuClock(OldSystemCoreClock, CLKPWR_CLOCK_IRC);
#endif
#if (BOARD_RA == 2)
    CLKPWR_setCpuClock(OldSystemCoreClock, CLKPWR_CLOCK_FRO12);
#endif

    /* Restore GPIO state. */
    BSP_wakeup();

    handle->sleeping = false;

    /* Wait some time before accessing the radio. */
    osDelay(10);

    /* Restore radio mode */
    SONDE_Detector detector = handle->sondeDetector;
    handle->sondeDetector = _SONDE_DETECTOR_UNDEFINED_;
    SYS_enableDetector(handle, handle->currentFrequencyHz, detector);
}



/* Read a new RSSI value in dBm. */
LPCLIB_Result SYS_readRssi (SYS_Handle handle, float *rssi)
{
    (void)handle;

    int32_t rawRssiTenthDb;

    /* Get a new RSSI value from radio (value comes as integer in tenth of a dB */
    ADF7021_readRSSI(radio, &rawRssiTenthDb);
    float level = rawRssiTenthDb / 10.0f;

    /* Correct for LNA gain */
    if (GPIO_readBit(GPIO_LNA_GAIN) == 1) {
        level += config_g->rssiCorrectionLnaOn;
    }
    else {
        level += config_g->rssiCorrectionLnaOff;
    }

    *rssi = level;

    return LPCLIB_SUCCESS;
}



/* Read a new RSSI value and filter it. Return filtered RSSI in dBm. */
static float _SYS_getFilteredRssi (SYS_Handle handle)
{
    (void)handle;

    /* Inertial damping to emulate response of a mechanical level meter.
     * ADF7021 RSSI readings are stable for high levels, but quite unstable
     * for lower input levels.
     */
#define RSSI_INERTIAL_HI 0.6f           /* Inertial for high levels */
#define RSSI_INERTIAL_HI_ABOVE -100.0f   /* Threshold for hi inertial */
#define RSSI_INERTIAL_LO 0.1f           /* Inertial for low levels */
#define RSSI_INERTIAL_LO_BELOW -115.0f   /* Threshold for lo inertial */

    int32_t rawRssiTenthDb;
    static float level;
    float adjustedLevel;

    /* Get a new RSSI value from radio (value comes as integer in tenth of a dB */
    ADF7021_readRSSI(radio, &rawRssiTenthDb);
    float newLevel = rawRssiTenthDb / 10.0f;

    /* Apply inertial damping to raw level (before attenuator correction) */
    float inertial = RSSI_INERTIAL_LO;
    if (level >= RSSI_INERTIAL_HI_ABOVE) {
        inertial = RSSI_INERTIAL_HI;
    }
    else if (level >= RSSI_INERTIAL_LO_BELOW) {
        inertial = RSSI_INERTIAL_LO + (level - RSSI_INERTIAL_LO_BELOW)
                * ((RSSI_INERTIAL_HI - RSSI_INERTIAL_LO) / (RSSI_INERTIAL_HI_ABOVE - RSSI_INERTIAL_LO_BELOW));
    }
    level -= (level - newLevel) * inertial;

    /* Correct for LNA gain */
    adjustedLevel = level;
    if (GPIO_readBit(GPIO_LNA_GAIN) == 1) {
        adjustedLevel += config_g->rssiCorrectionLnaOn;
    }
    else {
        adjustedLevel += config_g->rssiCorrectionLnaOff;
    }

    return adjustedLevel;
}


/* Measure the battery voltage */
//TODO in the future use an ADC driver...
static float _SYS_measureVbat (SYS_Handle handle)
{
    (void)handle;
    float vbat = NAN;

#if (BOARD_RA == 2)
    uint32_t vbatAdcDat;

    /* Enable voltage divider for VBAT */
    GPIO_writeBit(GPIO_VBAT_ADC_ENABLE, 0);

    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_ADC0);
    CLKPWR_unitPowerUp(CLKPWR_UNIT_ADC0);
    CLKPWR_unitPowerUp(CLKPWR_UNIT_VDDA);
    CLKPWR_unitPowerUp(CLKPWR_UNIT_VREFP);

    //TODO need at least 10µs delay here before ADC can be enabled */
    volatile int delay = SystemCoreClock / 100000;
    while (delay--);

    LPC_ADC0->STARTUP = 0
        | (1 << 0)                          /* Enable */
        ;
    LPC_ADC0->CALIB = 0
        | (1 << 0)                          /* Start a calibration */
        ;
    while (LPC_ADC0->CALIB & (1 << 0))      /* Wait until calibration is over */
        ;
    LPC_ADC0->CTRL = 0
        | ((2-1) << 0)                      /* Divide system clock by 2 (--> ADC clock = 24 MHz */
        | (0 << 1)                          /* Synchronous mode */
        | (3 << 9)                          /* 12-bit resolution */
        | (0 << 11)                         /* Use calibration result */
        | (7 << 12)                         /* Maximum sample time (9.5 ADC clocks) */
        ;

    LPC_ADC0->SEQA_CTRL &= ~(1u << 31);     /* Seq A: Disable */
    LPC_ADC0->SEQB_CTRL &= ~(1u << 31);     /* Seq B: Disable */
    LPC_ADC0->SEQA_CTRL = (LPC_ADC0->SEQA_CTRL & ~((0x3F << 12) | (0xFFF << 0)))
        | (1 << 8)                          /* Seq A: Select channel 8 */
        | (3 << 12)                         /* Seq A: Trigger source 3 (unused source) */
        ;
    LPC_ADC0->SEQA_CTRL |= (1u << 18);      /* Seq A: Positive edge trigger */
    LPC_ADC0->SEQA_CTRL |= (1u << 31);      /* Seq A: Enable */
    LPC_ADC0->SEQA_CTRL |= (1u << 26);      /* Seq A: Start single sequence */

    do {
        vbatAdcDat = LPC_ADC0->DAT8;
    } while (!(vbatAdcDat & (1u << 31)));   /* Wait until there is a valid result */

    /* Convert to real battery voltage */
    vbat = (float)(vbatAdcDat & 0xFFFF) / 65536.0f * 3.0f * 2;

    CLKPWR_unitPowerDown(CLKPWR_UNIT_VREFP);
    CLKPWR_unitPowerDown(CLKPWR_UNIT_VDDA);
    CLKPWR_unitPowerDown(CLKPWR_UNIT_ADC0);
    CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_ADC0);

    /* Disable voltage divider for VBAT */
    GPIO_writeBit(GPIO_VBAT_ADC_ENABLE, 1);
#endif

    return vbat;
}


/* Control the attenuator (LNA) */
void SYS_setAttenuator (SYS_Handle handle, bool enable)
{
    (void)handle;

    GPIO_writeBit(GPIO_LNA_GAIN, enable ? 0 : 1);
}


/* Return last RSSI measurement in an RX frame */
float SYS_getFrameRssi (SYS_Handle handle)
{
    return handle->lastInPacketRssi;
}


/* Return frequency offset for this frame */
float SYS_getFrameOffsetKhz (SYS_Handle handle)
{
    return handle->packetOffsetKhz;
}


/* Submit a job for the system handler. */
LPCLIB_Result SYS_handleEvent (LPCLIB_Event event)
{
    SYS_Message *pMessage = NULL;


    if (!sysContext.queue) {
        return LPCLIB_ERROR;
    }

    if ((event.id == LPCLIB_EVENTID_GPIO) && (event.opcode == GPIO_EVENT_GROUP_INTERRUPT)) {
        pMessage = osMailAlloc(sysContext.queue, 0);
        if (pMessage) {
            pMessage->opcode = SYS_OPCODE_GET_OFFSET;
        }
    }

    if (event.id == LPCLIB_EVENTID_APPLICATION) {
        /* Handle things that can be done immediately without too much effort. */
        switch (event.opcode) {
            case APP_EVENT_HEARD_SONDE:
                {
                    float frequencyMHz = (uint32_t)event.parameter / 1e6f;
                    SCANNER_addListenFrequency(
                            scanner,
                            frequencyMHz,
                            (SONDE_Type)event.block);
                }
                break;

            default:
                /* All remaining event require task action. */
                pMessage = osMailAlloc(sysContext.queue, 0);
                if (pMessage) {
                    pMessage->opcode = SYS_OPCODE_EVENT;
                    pMessage->event = event;
                }

                switch (event.opcode) {
                default:
                    break;
                }
        }
    }

    /* Defer more complicated jobs to the SYS handler task. */
    if (pMessage) {
        osMailPut(sysContext.queue, pMessage);
    }

    return LPCLIB_SUCCESS;
}


static void _SYS_osalCallback (void const *pArgument)
{
    SYS_Message *pMessage;

    if (sysContext.queue == NULL) {
        return;
    }

    switch ((int)pArgument) {
        case SYS_TIMERMAGIC_RSSI:
            pMessage = osMailAlloc(sysContext.queue, 0);
            if (pMessage == NULL) {
                return;
            }

            pMessage->opcode = SYS_OPCODE_GET_RSSI;
            osMailPut(sysContext.queue, pMessage);
            break;

        case SYS_TIMERMAGIC_INACTIVITY:
            pMessage = osMailAlloc(sysContext.queue, 0);
            if (pMessage == NULL) {
                return;
            }

            pMessage->opcode = SYS_OPCODE_EVENT;
            pMessage->event.opcode = APP_EVENT_SUSPEND;
            osMailPut(sysContext.queue, pMessage);
            break;
    }
}


/* Install a callback to become informed about system events. */
void SYS_installCallback (struct SYS_ConfigCallback configCallback)
{
//TODO: critical section!
    if (configCallback.pOldCallback) {
        *(configCallback.pOldCallback) = sysContext.callback;
    }
    sysContext.callback = configCallback.callback;
}


osMailQDef(sysQueueDef, SYS_QUEUE_LENGTH, SYS_Message);
osTimerDef(rssiTimer, _SYS_osalCallback);
osTimerDef(inactivityTimer, _SYS_osalCallback);


LPCLIB_Result SYS_open (SYS_Handle *pHandle)
{
    *pHandle = &sysContext;
    sysContext.sondeType = SONDE_UNDEFINED;

    return LPCLIB_SUCCESS;
}



static void _SYS_handleBleCommand (SYS_Handle handle) {
    char *cl = handle->commandLine;

    /* Valid command line starts with command number, a comma, and a non-zero length payload */
    int channel;
    if (sscanf(cl, "#%d", &channel) == 1) {
        switch (channel) {
        case HOST_CHANNEL_PING:
            {
                /* If host sends a time stamp, respond by sending all current settings. */
                long timestamp = 0;
                if (sscanf(cl, "#%*d,%ld", &timestamp) == 1) {
                    /* Send application and version */
                    char s[80];
                    snprintf(s, sizeof(s), "1,%d,%d,%d,%s",
                            FIRMWARE_VERSION_MAJOR,
#if (BOARD_RA == 1)
                            1,
#elif (BOARD_RA == 2)
                            2,
#else
                            0,
#endif
                            FIRMWARE_VERSION_MINOR,
                            FIRMWARE_NAME);
                    SYS_send2Host(HOST_CHANNEL_PING, s);

                    /* Send status */
                    _SYS_reportRadioFrequency(handle);
                    SYS_send2Host(HOST_CHANNEL_GUI, SCANNER_getManualMode(scanner) ? "2,1" : "2,0");
                    SONDE_Detector sondeDetector = SCANNER_getManualSondeDetector(scanner);
                    snprintf(s, sizeof(s), "5,%d", (int)sondeDetector);
                    SYS_send2Host(HOST_CHANNEL_GUI, s);
                    SYS_send2Host(HOST_CHANNEL_GUI, SCANNER_getManualAttenuator(scanner) ? "6,1" : "6,0");
                    SYS_send2Host(HOST_CHANNEL_GUI, SCANNER_getScannerMode(scanner) ? "7,1" : "7,0");

                    //TODO send only if ping parameter asks for it
                    RS41_resendLastPositions(handle->rs41);
                    RS92_resendLastPositions(handle->rs92);
                    DFM_resendLastPositions(handle->dfm);
                    SRSC_resendLastPositions(handle->srsc);
                    IMET_resendLastPositions(handle->imet);
                    M10_resendLastPositions(handle->m10);
                }
            }
            break;

        case 1:     /* Set frequency */
            {
                float frequency;

                if (sscanf(cl, "#%*d,%f", &frequency) == 1) {
                    SCANNER_setManualFrequency(scanner, 1000 * lround(frequency * 1e3));    // round to 1 kHz
                }
            }
            break;

        case 3:     /* Set manual sonde detector */
            {
                int selector;
                SONDE_Detector detector;

                if (sscanf(cl, "#%*d,%d", &selector) == 1) {
                    detector = SONDE_DETECTOR_RS41_RS92;
                    switch (selector) {
                        case 0:     detector = SONDE_DETECTOR_RS41_RS92; break;
                        case 1:     detector = SONDE_DETECTOR_DFM; break;
                        case 2:     detector = SONDE_DETECTOR_C34_C50; break;
                        case 3:     detector = SONDE_DETECTOR_IMET; break;
                        case 4:     detector = SONDE_DETECTOR_MODEM; break;
                        case 5:     detector = SONDE_DETECTOR_RS41_RS92_DFM; break;
                    }
                    SCANNER_setManualSondeDetector(scanner, detector);
                }
            }
            break;

        case HOST_CHANNEL_EPHEMUPDATE:
            EPHEMUPDATE_processCommand(euTask, cl);
            break;

        case 6:     /* Scanner list control */ //TODO
            {
                int list, action;

                if (sscanf(cl, "#%*d,%d,%d", &list, &action) == 2) {
                    if (list == 0) {        /* Scanner */
                        ; //TODO
//                            SCANNER_addListenFrequency(scanner, frequencyMHz, SONDE_DECODER_RS41_RS92);  //TODO TODO
//                        SCANNER_setManualMode(scanner, true);
                    }

                    if (list == 1) {        /* Sonde lists in decoders */
                        if (action == 0) {  /* Remove an entry */
                            float frequencyMHz;
                            int decoderCode;
                            if (sscanf(cl, "#%*d,%*d,%*d,%f,%d", &frequencyMHz, &decoderCode) == 2) {
                                SONDE_Decoder decoder = (SONDE_Decoder)decoderCode;
                                switch (decoder) {
                                    case SONDE_DECODER_MODEM:
                                        M10_removeFromList(handle->m10, frequencyMHz);
                                        break;
                                    case SONDE_DECODER_RS41:
                                        RS41_removeFromList(handle->rs41, frequencyMHz);
                                        break;
                                    case SONDE_DECODER_RS92:
                                        RS92_removeFromList(handle->rs92, frequencyMHz);
                                        break;
                                    case SONDE_DECODER_DFM:
                                        DFM_removeFromList(handle->dfm, frequencyMHz);
                                        break;
                                    case SONDE_DECODER_C34_C50:
                                        SRSC_removeFromList(handle->srsc, frequencyMHz);
                                        break;
                                    default:
                                        /* ignore */
                                        break;
                                }
                                SCANNER_removeListenFrequency(scanner, frequencyMHz);
                            }
                        }
                    }
                }
            }
            break;

        case HOST_CHANNEL_SWITCHES:
            {
                int command;
                int enableValue;
                bool enable;

                if (sscanf(cl, "#%*d,%d,%d", &command, &enableValue) == 2) {
                    enable = (enableValue != 0);
                    switch (command) {
                        case 1:
                            SCANNER_setManualAttenuator(scanner, enable);
                            break;

                        case 2:
                            SCANNER_setScannerMode(scanner, enable);
                            if (enable) {
                                handle->currentFrequencyHz = 0;
                                _SYS_reportRadioFrequency(handle);
                            }
                            break;

                        case 3:
                            SCANNER_setManualMode(scanner, enable);
                            osTimerStart(handle->rssiTick, 20);
                            break;
                    }
                }
            }
            break;

        case HOST_CHANNEL_FIRMWAREUPDATE:
            {
                /* We accept the following commands:
                 * 1) A ping
                 * 2) Request for a challenge
                 * 3) Firmware erase (requires correct response to challenge)
                 */
                int command;
                int securityResponse;
                uint32_t sectorNumber;
                uint32_t pageNumber;
                LPCLIB_Result iapResult;
                char s[20];

                if (sscanf(cl, "#%*d,%d", &command) == 1) {
                    switch (command) {
                        case 0:
                            sprintf(s, "%d,0", command);
                            SYS_send2Host(HOST_CHANNEL_FIRMWAREUPDATE, s);
                            break;
                        case 42:
                            securityResponse = -1;
                            if (sscanf(cl, "#%*d,%*d,%d", &securityResponse) == 1) {
                                /* Is this the right security code to erase the firmware? */
                                if (securityResponse == handle->securityResponse) {
                                    /* Yes! Invalidate firmware, then restart */

                                    /* Send response, and wait some time to ensure it's been sent. */
                                    sprintf(s, "%d,0", command);
                                    SYS_send2Host(HOST_CHANNEL_FIRMWAREUPDATE, s);
                                    osDelay(1000);

                                    /* Disable interrupts. Stop M0+ core. Stop PDM engine. */
                                    __disable_irq();
                                    LPC_SYSCON->CPUCTRL = (LPC_SYSCON->CPUCTRL | 0xC0C40000) | (1 << 3) | (1 << 5);
                                    PDM_stop(handle->pdm);

                                    IAP_address2SectorNumber(FIRMWARE_END_ADDRESS, &sectorNumber, NULL);
                                    IAP_address2PageNumber(FIRMWARE_END_ADDRESS, &pageNumber, NULL);

                                    iapResult = IAP_prepareSectorsForWriteOperation(
                                            sectorNumber,
                                            sectorNumber,
                                            NULL);
                                    if (iapResult == LPCLIB_SUCCESS) {
                                        iapResult = IAP_erasePages(
                                                pageNumber,
                                                pageNumber,
                                                NULL);
                                    }

                                    NVIC_SystemReset();
                                }
                            }

                            /* Incorrect security response or (worse...) flash failure */
                            sprintf(s, "%d,1", command);
                            SYS_send2Host(HOST_CHANNEL_FIRMWAREUPDATE, s);
                            handle->securityResponse = 0;
                            break;

                        case 99:
                            /* Produce two random numbers that add up to a three-digit decimal number */
                            srand(os_time);
                            int rand1 = rand() % 1000;
                            int rand2 = rand() % 1000;
                            if (rand1 < 100) {
                                rand1 += 100;
                            }
                            if (rand2 < 100) {
                                rand2 += 100;
                            }
                            while (rand1 + rand2 > 1000) {
                                if (rand1 > 500) {
                                    rand1 -= 100;
                                }
                                else if (rand2 > 500) {
                                    rand2 -= 100;
                                }
                            }
                            handle->securityResponse = rand1 + rand2;

                            sprintf(s, "%d,%d,%d", command, rand1, rand2);
                            SYS_send2Host(HOST_CHANNEL_FIRMWAREUPDATE, s);
                            break;

                        default:
                            //TODO
                            break;
                    }
                }
            }
            break;

        case 99:        /* Keep-alive message */
            /* No action required. Packet reception itself has retriggered the keep-alive timer. */
            break;
        }
    }
}


static bool _SYS_checkEvent (SYS_Handle handle)
{
    bool haveEvent = false;

    /* Something in the mailbox? */
    handle->rtosEvent = osMailGet(handle->queue, 0);
    haveEvent |= handle->rtosEvent.status == osEventMail;
#if 1
    /* Data from Bluetooth link? */
    if (UART_readLine(blePort, handle->commandLine, sizeof(handle->commandLine)) > 0) {
        haveEvent = true;
    }
    else {
        /* There may have been partial data already copied into commandLine buffer (no complete line yet).
         * Make sure this won't be misinterpreted as a command.
         */
        handle->commandLine[0] = 0;
    }
#endif
    return haveEvent;
}



PT_THREAD(SYS_thread (SYS_Handle handle))
{
    static SYS_Message *pMessage;
    static uint32_t n;


    PT_BEGIN(&handle->pt);

    sysContext.queue = osMailCreate(osMailQ(sysQueueDef), NULL);
//    GPIO_ioctl(buttonInit);
    GPIO_ioctl(uartRxdWakeupInit);

    EPHEMERIS_init();

    RS41_open(&handle->rs41);
    RS92_open(&handle->rs92);
    DFM_open(&handle->dfm);
    SRSC_open(&handle->srsc);
    IMET_open(&handle->imet);
    M10_open(&handle->m10);
    PDM_open(0, &handle->pdm);

#if SEMIHOSTING
    handle->fpLog = fopen("sys.csv", "w");
#endif

    handle->rssiTick = osTimerCreate(osTimer(rssiTimer), osTimerPeriodic, (void *)SYS_TIMERMAGIC_RSSI);
    osTimerStart(handle->rssiTick, 20);

    handle->inactivityTimeout = osTimerCreate(osTimer(inactivityTimer), osTimerOnce, (void *)SYS_TIMERMAGIC_INACTIVITY);
    osTimerStart(handle->inactivityTimeout, INACTIVITY_TIMEOUT);

    while (1) {
        /* Wait for an event */
        PT_WAIT_UNTIL(&handle->pt, _SYS_checkEvent(handle));

        /* Message via Bluetooth? */
        if (strlen(handle->commandLine) > 0) {
            osTimerStart(handle->inactivityTimeout, INACTIVITY_TIMEOUT);

            _SYS_handleBleCommand(handle);
            handle->commandLine[0] = 0;
        }

        /* Is there a new message? */
        if (handle->rtosEvent.status == osEventMail) {
            pMessage = (SYS_Message *)handle->rtosEvent.value.p;
            switch (pMessage->opcode) {
            case SYS_OPCODE_EVENT:
                switch(pMessage->event.opcode) {
                case APP_EVENT_SUSPEND:
                    SYS_sleep(handle);
                    break;

                case APP_EVENT_RAW_FRAME:
                    /* Check which type of sonde this is for */
                    if ((SONDE_Type)pMessage->event.block == SONDE_C34) {
                        SRSC_processBlock(
                                    handle->srsc,
                                    (uint8_t *)pMessage->event.parameter,
                                    7,
                                    handle->currentFrequencyHz * 1.0f);
                    }
                    else if ((SONDE_Type)pMessage->event.block == SONDE_IMET_RSB) {
                        IMET_processBlock(
                                    handle->imet,
                                    (uint8_t *)pMessage->event.parameter,
                                    0,  /* variable packet length */
                                    handle->currentFrequencyHz * 1.0f);
                    }
                    break;
                }
                break;

            case SYS_OPCODE_BUFFER_COMPLETE:
                {
                    /* Find out which buffer to use */
                    int bufferIndex = pMessage->bufferIndex;

                    /* Ignore glitches (packets arriving while switching to/from scanner mode) */
                    if (handle->currentFrequencyHz != 0) {
                        SONDE_Type sondeType = SONDE_UNDEFINED;
                        switch (ipc[bufferIndex].param) {
                            case 0: sondeType = SONDE_RS92; break;
                            case 1: sondeType = SONDE_RS41; break;
                            case 2: sondeType = SONDE_DFM09; break;
                            case 3: sondeType = SONDE_DFM06; break;
                            case 4: sondeType = SONDE_M10; break;
                        }

                        /* Process buffer */
                        if (sondeType == SONDE_M10) {
                            M10_processBlock(
                                    handle->m10,
                                    ipc[bufferIndex].data8,
                                    _SYS_getSondeBufferLength(SONDE_M10),
                                    handle->currentFrequencyHz * 1.0f);

                            /* Let scanner prepare for next frequency */
                            SCANNER_notifyValidFrame(scanner);
                        }
                        else if (sondeType == SONDE_RS41) {
                            RS41_processBlock(
                                    handle->rs41,
                                    ipc[bufferIndex].data8,
                                    _SYS_getSondeBufferLength(SONDE_RS41),
                                    handle->currentFrequencyHz * 1.0f);

                            /* Let scanner prepare for next frequency */
                            SCANNER_notifyValidFrame(scanner);
                        }
                        else if (sondeType == SONDE_RS92) {
                            RS92_processBlock(
                                    handle->rs92,
                                    ipc[bufferIndex].data8,
                                    _SYS_getSondeBufferLength(SONDE_RS92),
                                    handle->currentFrequencyHz * 1.0f);

                            /* Let scanner prepare for next frequency */
                            SCANNER_notifyValidFrame(scanner);
                        }
                        else if (sondeType == SONDE_DFM06) {
                            // DFM-06 uses inverted data (or inverted FSK modulation)
                            for (n = 0; n < _SYS_getSondeBufferLength(SONDE_DFM06); n++) {
                                ipc[bufferIndex].data8[n] ^= 0xFF;
                            }
                            if (DFM_processBlock(
                                    handle->dfm,
                                    sondeType,
                                    ipc[bufferIndex].data8,
                                    _SYS_getSondeBufferLength(SONDE_DFM06),
                                    handle->currentFrequencyHz * 1.0f) == LPCLIB_SUCCESS) {
                                /* Frame complete. Let scanner prepare for next frequency */
                                SCANNER_notifyValidFrame(scanner);
                            }
                        }
                        else if (sondeType == SONDE_DFM09) {
                            if (DFM_processBlock(
                                    handle->dfm,
                                    sondeType,
                                    ipc[bufferIndex].data8,
                                    _SYS_getSondeBufferLength(SONDE_DFM09),
                                    handle->currentFrequencyHz * 1.0f) == LPCLIB_SUCCESS) {
                                /* Frame complete. Let scanner prepare for next frequency */
                                SCANNER_notifyValidFrame(scanner);
                            }
                        }
                    }

                    /* Buffer may now be reused */
                    ipc[bufferIndex].valid = 0;
                }
                break;

            case SYS_OPCODE_GET_RSSI:
                {
                    static int rate;
                    static bool lastScannerMode;

                    if (SCANNER_getScannerMode(scanner) != lastScannerMode) {
                        lastScannerMode = SCANNER_getScannerMode(scanner);
                        rate = 0;
                    }

                    /* Send RSSI only when not in scanner mode */
                    if (!SCANNER_getScannerMode(scanner)) {
                        if (++rate >= 2) {
                            float rssi = _SYS_getFilteredRssi(handle);
                            char s[30];
                            sprintf(s, "3,%.1f", rssi);
                            SYS_send2Host(HOST_CHANNEL_GUI, s);
//TODO need to store this near the end of the frame
handle->lastInPacketRssi = rssi;

                            rate = 0;
                        }
                    }
                    else {
                        /* Send dummy value at a low rate to ensure S-meter shows minimum */
                        if (rate == 0) {
                            SYS_send2Host(HOST_CHANNEL_GUI, "3,");
                        }

                        if (++rate >= 100) {
                            rate = 0;
                        }
                    }

#if (BOARD_RA == 2)
                    //TODO
                    /* Get frequency offset from PDM DC bias (only AFSK modes) */
                    SRSC_setRxOffset(handle->srsc, PDM_getDcOffset(handle->pdm) / 1.32f); //TODO factor
#endif
handle->vbat = _SYS_measureVbat(handle);
                }
                break;

            case SYS_OPCODE_GET_OFFSET:
                {
                    int32_t offset;

                    ADF7021_readOffset(radio, &offset);
                    handle->packetOffsetKhz = offset / 1000.0f;
                }
                break;
            }

            osMailFree(sysContext.queue, pMessage);
        }
    }

    PT_END(&handle->pt);
}


