/* Copyright (c) 2016-2021, DF9DQ
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

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"

#include "adf7021.h"
#include "app.h"
#include "beacon.h"
#include "bl652.h"
#include "bootloader.h"
#include "cf06.h"
#include "china1.h"
#include "dfm.h"
#include "gth3.h"
#include "imet.h"
#include "imet54.h"
#include "jinyang.h"
#include "m10.h"
#include "m20.h"
#include "meisei.h"
#include "mon.h"
#include "mrz.h"
#include "mts01.h"
#include "pdm.h"
#include "pilot.h"
#include "psb3.h"
#include "rinex.h"
#include "rs41.h"
#include "rs92.h"
#include "srsc.h"
#include "sys.h"
#include "windsond.h"
#include "config.h"
#if (BOARD_RA == 2)
#include "usbuser_config.h"
#endif


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
};


/** Identifiers for OS timers. */
enum {
    SYS_TIMERMAGIC_RSSI,
    SYS_TIMERMAGIC_INACTIVITY,
    SYS_TIMERMAGIC_STARTUP_DELAY,
};


/********** MUST BE DEFINED SAME AS IN M0 PROJECT ***********/
#define IPC_S2M_DATA_SIZE                   1024
#define IPC_S2M_NUM_BUFFERS                 4

typedef struct {
    volatile uint8_t valid;
    uint8_t opcode;
    uint8_t param;
    uint8_t reserved;
    uint16_t numBits;
    uint16_t rxTime;

    uint8_t data8[IPC_S2M_DATA_SIZE];
} IPC_S2M;

__SECTION(".ipc")
static IPC_S2M ipc[IPC_S2M_NUM_BUFFERS];
/************************************************************/



#define COMMAND_LINE_SIZE           400

#define STARTUP_POWERCONTROL_DELAY  60000
#define INACTIVITY_TIMEOUT          40000

#define VBAT_FILTER_LENGTH          50


struct SYS_Context {
    struct pt pt;

    osMailQId queue;                                    /**< Task message queue */
    osEvent rtosEvent;
    osTimerId rssiTick;
    osTimerId inactivityTimeout;
    osTimerId startupPowerControlTimeout;

    RS41_Handle rs41;
    RS92_Handle rs92;
    BEACON_Handle beacon;
    CF06_Handle cf06;
    DFM_Handle dfm;
    GTH3_Handle gth3;
    IMET_Handle imet;
    IMET54_Handle imet54;
    JINYANG_Handle jinyang;
    M10_Handle m10;
    M20_Handle m20;
    MRZ_Handle mrz;
    PILOT_Handle pilot;
    PSB3_Handle psb3;
    SRSC_Handle srsc;
    MEISEI_Handle meisei;
    WINDSOND_Handle windsond;
    MTS01_Handle mts01;
    PDM_Handle pdm;

    _Bool sleeping;                                     /**< Low-power mode activated */
    LPCLIB_Callback callback;                           /**< Callback for system events */

    SONDE_Type sondeType;
    SONDE_Detector sondeDetector;
    _Bool monitor;
    _Bool monitorUpdate;
    float currentFrequency;
    float currentRssi;
    float lastInPacketRssi;                             /**< Last RSSI measurement while data reception was still active */
    uint64_t realTime;                                  /**< Real-time as sent by host */
    uint32_t last_os_time;

    float vbatFilter[VBAT_FILTER_LENGTH];               /**< Taps for VBAT filter */
    int vbatFilterIndex;                                /**< Index for writing to VBAT filter */
    float vbat;                                         /**< Consolidated VBAT measurement */

    bool attenuatorActive;

    char commandLine[COMMAND_LINE_SIZE];
    int securityResponse;                               /* Response expected to security challenge */
    bool blePortSetBreak;                               /* Set/clear UART break on BLE port */
    int blePortBreakTime;                               /* Duration of break signalling on BLE port */
    bool linkEstablished;                               /* Set when host has connected */

    bool hasPowerScript;                                /* smartBASIC script loaded to BL652 */
} sysContext;



#if (BOARD_RA == 1)
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


static const ADF7021_Config radioModeVaisala[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_13k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = ENABLE,
            .KI = 11,
            .KP = 4,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_2FSK_CORR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .deviation = 2400,
            .postDemodBandwidth = 3600, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeGraw[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_13k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = ENABLE,
            .KI = 11,
            .KP = 2,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_2FSK_CORR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .deviation = 2400,
            .postDemodBandwidth = 1875, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeJinyang[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_13k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = ENABLE,
            .KI = 11,
            .KP = 2,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_2FSK_CORR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .deviation = 2400,
            .postDemodBandwidth = 1875, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeMrz[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_13k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = ENABLE,
            .KI = 11,
            .KP = 2,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_2FSK_CORR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .deviation = 2400,
            .postDemodBandwidth = 1875, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeAsia1[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_13k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = ENABLE,
//            .KI = 14,
//            .KP = 5,
.KI = 11,
.KP = 2,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_2FSK_CORR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .deviation = 2400,
            .postDemodBandwidth = 1875, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModePSB3[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_18k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = ENABLE,
            .KI = 11,
            .KP = 2,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_2FSK_CORR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .deviation = 5000,
            .postDemodBandwidth = 1875, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeIMET54[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_18k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = ENABLE,
            .KI = 11,
            .KP = 4,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_2FSK_CORR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .deviation = 2400,
            .postDemodBandwidth = 3600, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeModem[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_18k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = ENABLE,
            .KI = 11,
            .KP = 4,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
//TODO fails! use linear demod as workaround        {.demodType = ADF7021_DEMODULATORTYPE_2FSK_CORR, }},
        {.demodType = ADF7021_DEMODULATORTYPE_LINEAR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .deviation = 2400,
            .postDemodBandwidth = 7200, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModePilot[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
//        {.bandwidth = ADF7021_BANDWIDTH_13k5, }}, /* Old PCB variant <=9 */
        {.bandwidth = ADF7021_BANDWIDTH_18k, }},   /* PCB variant 10 */
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = ENABLE,
            .KI = 11,
            .KP = 4,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_2FSK_CORR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
//            .deviation = 2400,    /* Old PCB variant <=9 */
            .deviation = 6200,      /* PCB variant 10 */
            .postDemodBandwidth = 3600, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeMeisei[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_13k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = ENABLE,
            .KI = 11,
            .KP = 2,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_2FSK_CORR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .deviation = 2400,
            .postDemodBandwidth = 1800, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeMTS01[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_18k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = ENABLE,
            .KI = 11,
            .KP = 2,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_2FSK_CORR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .deviation = 5000,      //TODO
            .postDemodBandwidth = 938, }}}, //TODO
    {.opcode = ADF7021_OPCODE_SET_AGC_CLOCK,
        {.agcClockFrequency = 8e3f, }},
    /* Hardware sync detect is used to freeze the detector threshold
     * for the duration of the packet
     */
    {.opcode = ADF7021_OPCODE_SET_SYNCWORD,
        {.sync = {
            .pattern = 0xB42B80,
            .enable = ENABLE,
            .maxErrors = 0,
            .packetLength = 130, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeWindsond[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_FSK, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_25k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = DISABLE,
            .KI = 11,
            .KP = 2,
            .maxRange = 20, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_LINEAR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .deviation = 34000,
            .postDemodBandwidth = 1875, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeC34C50[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_AFSK_GAIN6, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_18k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = DISABLE, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_LINEAR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .postDemodBandwidth = 7500, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeImet[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_AFSK_GAIN5, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_18k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = DISABLE, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_LINEAR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .postDemodBandwidth = 3500, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeBeacon[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_AFSK_GAIN6, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_9k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = DISABLE, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_LINEAR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .postDemodBandwidth = 3000, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};

static const ADF7021_Config radioModeMONITOR[] = {
    {.opcode = ADF7021_OPCODE_POWER_ON, },
    {.opcode = ADF7021_OPCODE_SET_INTERFACE_MODE,
        {.interfaceMode = ADF7021_INTERFACEMODE_AFSK_GAIN6, }},
    {.opcode = ADF7021_OPCODE_SET_BANDWIDTH,
        {.bandwidth = ADF7021_BANDWIDTH_25k, }},
    {.opcode = ADF7021_OPCODE_SET_AFC,
        {.afc = {
            .enable = DISABLE, }}},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR,
        {.demodType = ADF7021_DEMODULATORTYPE_LINEAR, }},
    {.opcode = ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
        {.demodParams = {
            .postDemodBandwidth = 7500, }}},
    {.opcode = ADF7021_OPCODE_CONFIGURE, },

    ADF7021_CONFIG_END
};


static const UART_Config _uartFlushTx[] = {
    {.opcode = UART_OPCODE_FLUSH_TX_BUFFER, },

    UART_CONFIG_END
};


#if (BOARD_RA == 2)
static LPCLIB_Result _SYS_handleBleLinkDown (LPCLIB_Event event) {
    SYS_Message *pMessage;
    (void)event;

    pMessage = osMailAlloc(sysContext.queue, 0);
    if (pMessage == NULL) {
        return LPCLIB_OUT_OF_MEMORY;
    }

    pMessage->opcode = SYS_OPCODE_EVENT;
    pMessage->event.opcode = APP_EVENT_SUSPEND;
    osMailPut(sysContext.queue, pMessage);

    return LPCLIB_SUCCESS;
}

static const GPIO_Config bleLinkDownHandlerEnable[] = {
    {.opcode = GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT,
        {.pinInterrupt = {
            .pin = GPIO_BLE_EXTRA1,
            .enable = LPCLIB_YES,
            .mode = GPIO_INT_LOW_LEVEL_ONCE,
            .interruptLine = GPIO_PIN_INT_0,
            .callback = _SYS_handleBleLinkDown
            }}},

    GPIO_CONFIG_END
};

/* Handler for delayed link power control.
 * Called by a one-shot timer after reset. This is to make sure we have some time
 * to take control by a debugger!
 * Note that after reset (e.g. changing batteries) the receiver remains active
 * for one minute before beginning to react to BL652 link indication via BLE_EXTRA1.
 */
static void _SYS_handleStartupTimer (void const *pArgument)
{
    if (sysContext.queue == NULL) {
        return;
    }

    switch ((int)pArgument) {
        case SYS_TIMERMAGIC_STARTUP_DELAY:
            GPIO_ioctl(bleLinkDownHandlerEnable);
            break;
    }
}
#endif



//TODO need a mailbox driver
void MAILBOX_IRQHandler (void)
{
    SYS_Message *pMessage = NULL;
    SYS_Handle handle = &sysContext;
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
    else if (requests & 2) {
        /* Sample RSSI value for the current RX packet */
        handle->lastInPacketRssi = handle->currentRssi;
        LPC_MAILBOX->IRQ1CLR = (1u << 1);
    }
    else {
        /* No supported request. Clear them all */
        LPC_MAILBOX->IRQ1CLR = requests;
    }

    if (pMessage) {
        osMailPut(sysContext.queue, pMessage);
    }
}



static void _SYS_reportRadioFrequency (SYS_Handle handle)
{
    char s[20];
    sprintf(s, "1,%.3f", handle->currentFrequency / 1e6f);
    SYS_send2Host(HOST_CHANNEL_GUI, s);
}


static void _SYS_reportVbat (SYS_Handle handle)
{
    char s[20];
    sprintf(s, "6,%.2f", handle->vbat);
    SYS_send2Host(HOST_CHANNEL_GUI, s);
}


static void _SYS_reportControls (SYS_Handle handle)
{
    (void)handle;

    char s[20];
    snprintf(s, sizeof(s), "2,%d", SCANNER_getMode(scanner));
    SYS_send2Host(HOST_CHANNEL_GUI, s);
    SONDE_Detector sondeDetector = SCANNER_getManualSondeDetector(scanner);
    snprintf(s, sizeof(s), "5,%d", (int)sondeDetector);
    SYS_send2Host(HOST_CHANNEL_GUI, s);
    snprintf(s, sizeof(s), "9,%d", handle->monitor ? 1 : 0);
    SYS_send2Host(HOST_CHANNEL_GUI, s);
}


static void _SYS_reportEphemerisAge (SYS_Handle handle)
{
    (void)handle;

    const char *ephemAge;
    if (EPHEMERIS_findOldestEntry(&ephemAge) == LPCLIB_SUCCESS) {
        char s[32];
        snprintf(s, sizeof(s), "8,%s", ephemAge);
        SYS_send2Host(HOST_CHANNEL_GUI, s);
    }
}



static void _SYS_setRadioFrequency (SYS_Handle handle, float frequency)
{
    if (frequency < 400e6f) {
        frequency = 400e6f;
    }
    if (frequency > 406.1e6f) {
        frequency = 406.1e6f;
    }

    LPC_MAILBOX->IRQ0SET = (1u << 30);
    SRSC_pauseResume(handle->srsc, ENABLE);
    IMET_pauseResume(handle->imet, ENABLE);
    MON_DSP_reset();

    ADF7021_setPLL(radio, frequency - 100000);
    handle->currentFrequency = frequency;

    LPC_MAILBOX->IRQ0SET = (1u << 31);
    SRSC_pauseResume(handle->srsc, DISABLE);
    IMET_pauseResume(handle->imet, DISABLE);
    MON_DSP_reset();
}



/* Get current RX frequency */
float SYS_getCurrentFrequency (SYS_Handle handle)
{
    return handle->currentFrequency;
}



/* Enable the receiver, select a frequency and a sonde decoder type.
 */
LPCLIB_Result SYS_enableDetector (SYS_Handle handle, float frequency, SONDE_Detector detector)
{
#if (BOARD_RA == 2)
    float demodClock;
#endif

    if ((detector != handle->sondeDetector) || (frequency != handle->currentFrequency) || handle->monitorUpdate) {
        PDM_stop(handle->pdm);

        handle->sondeDetector = detector;

        ADF7021_calibrateIF(radio, 1);  //TODO coarse/fine

        handle->monitorUpdate = false;
        if (handle->monitor) {
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(0));
            ADF7021_ioctl(radio, radioModeMONITOR);

            _SYS_setRadioFrequency(handle, frequency);
            _SYS_reportRadioFrequency(handle);  /* Inform host */

#if (BOARD_RA == 1)
            PDM_run(handle->pdm, 202, MON_handleAudioCallback);
#endif
#if (BOARD_RA == 2)
            ADF7021_getDemodClock(radio, &demodClock);
            PDM_run(handle->pdm, lrintf(demodClock / 16000.0f), MON_handleAudioCallback);
#endif
        } else {
            switch (detector) {
            case SONDE_DETECTOR_C34_C50:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(0));
                ADF7021_ioctl(radio, radioModeC34C50);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

#if (BOARD_RA == 1)
                PDM_run(handle->pdm, 202, SRSC_handleAudioCallback);
#endif
#if (BOARD_RA == 2)
                ADF7021_getDemodClock(radio, &demodClock);
                PDM_run(handle->pdm, lrintf(demodClock / 16000.0f), SRSC_handleAudioCallback);
#endif
                LPC_MAILBOX->IRQ0SET = (1u << 2); //TODO
                break;

            case SONDE_DETECTOR_IMET:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(0));
                ADF7021_ioctl(radio, radioModeImet);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

#if (BOARD_RA == 1)
                PDM_run(handle->pdm, 202, IMET_handleAudioCallback);
#endif
#if (BOARD_RA == 2)
                ADF7021_getDemodClock(radio, &demodClock);
                PDM_run(handle->pdm, lrintf(demodClock / 16000.0f), IMET_handleAudioCallback);
#endif
                LPC_MAILBOX->IRQ0SET = (1u << 2); //TODO
                break;

            case SONDE_DETECTOR_BEACON:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(0));
                ADF7021_ioctl(radio, radioModeBeacon);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

#if (BOARD_RA == 1)
                PDM_run(handle->pdm, 202, BEACON_handleAudioCallback);
#endif
#if (BOARD_RA == 2)
                ADF7021_getDemodClock(radio, &demodClock);
                PDM_run(handle->pdm, lrintf(demodClock / 16000.0f), BEACON_handleAudioCallback);
#endif
                LPC_MAILBOX->IRQ0SET = (1u << 2); //TODO
                break;

            case SONDE_DETECTOR_DFM:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(2500));
                ADF7021_setBitRate(radio, 2500);
                ADF7021_ioctl(radio, radioModeGraw);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 1); //TODO
                break;

            case SONDE_DETECTOR_JINYANG:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(2400));
                ADF7021_setBitRate(radio, 2400);
                ADF7021_ioctl(radio, radioModeJinyang);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 6); //TODO
                break;

            case SONDE_DETECTOR_MRZ:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(2400));
                ADF7021_setBitRate(radio, 2400);
                ADF7021_ioctl(radio, radioModeMrz);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 8); //TODO
                break;

            case SONDE_DETECTOR_MEISEI:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(2400));
                ADF7021_setBitRate(radio, 2400);
                ADF7021_ioctl(radio, radioModeMeisei);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 5); //TODO
                break;

            case SONDE_DETECTOR_WINDSOND:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(2400));
                ADF7021_setBitRate(radio, 2400);
                ADF7021_ioctl(radio, radioModeWindsond);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 7); //TODO
                break;

            case SONDE_DETECTOR_RS41_RS92:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(4800));
                ADF7021_setBitRate(radio, 4800);
                ADF7021_ioctl(radio, radioModeVaisala);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 0); //TODO
                break;

            case SONDE_DETECTOR_MODEM:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(9600));
                ADF7021_setBitRate(radio, 9600);
                ADF7021_ioctl(radio, radioModeModem);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 3); //TODO
                break;

            case SONDE_DETECTOR_PILOT:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(4800));
                ADF7021_setBitRate(radio, 4800);
                ADF7021_ioctl(radio, radioModePilot);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 4); //TODO
                break;

            case SONDE_DETECTOR_ASIA1: /* 2FSK 2.4k deviation, 2400 sym/s */
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(2400));
                ADF7021_setBitRate(radio, 2400);
                ADF7021_ioctl(radio, radioModeAsia1);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 9); //TODO
                break;

            case SONDE_DETECTOR_PSB3:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(768));
                ADF7021_setBitRate(radio, 768);
                ADF7021_ioctl(radio, radioModePSB3);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 10); //TODO
                break;

            case SONDE_DETECTOR_IMET54:
                ADF7021_setDemodClockDivider(radio, CONFIG_getDemodClockDivider(4800));
                ADF7021_setBitRate(radio, 4800);
                ADF7021_ioctl(radio, radioModeIMET54);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 11); //TODO
                break;

            case SONDE_DETECTOR_MTS01:
                ADF7021_setDemodClockDivider(radio, 4);
                ADF7021_setBitRate(radio, 1202);
                ADF7021_ioctl(radio, radioModeMTS01);

                _SYS_setRadioFrequency(handle, frequency);
                _SYS_reportRadioFrequency(handle);  /* Inform host */

                LPC_MAILBOX->IRQ0SET = (1u << 12); //TODO
                break;

            default:
                // Disable sync detector
                //TODO
                break;
            }
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

    snprintf(s, sizeof(s), "#%d,", channel);
    for (i = 0; i < (int)strlen(s); i++) {
        checksum += s[i];
    }
    UART_write(blePort, s, strlen(s));
#if (BOARD_RA == 2)
    USBSerial_write(s, strlen(s));
#endif

    for (i = 0; i < (int)strlen(message); i++) {
        checksum += message[i];
    }
    UART_write(blePort, (uint8_t *)message, strlen(message));
#if (BOARD_RA == 2)
    USBSerial_write(message, strlen(message));
#endif

    checksum += ',';
    snprintf(s, sizeof(s), ",%d\r", checksum % 100);
    UART_write(blePort, s, strlen(s));
#if (BOARD_RA == 2)
    USBSerial_write(s, strlen(s));
#endif

    return LPCLIB_SUCCESS;
}


//TODO: duration=0: off, duration=-1: on, duration>0: on for specified #milliseconds
LPCLIB_Result SYS_sendBreak (int durationMilliseconds)
{
    sysContext.blePortBreakTime = durationMilliseconds;
    sysContext.blePortSetBreak = true;

    return LPCLIB_SUCCESS;
}



/** Enter low-power mode */
static void SYS_sleep (SYS_Handle handle)
{
    uint32_t OldSystemCoreClock;


    /* Reset M0 (keep it there) */
    extern uint32_t M0IMAGE_start;
    LPC_SYSCON->CPUCTRL = (LPC_SYSCON->CPUCTRL | 0xC0C40000) | (1 << 3) | (1 << 5);

    handle->linkEstablished = false;

    SCANNER_setMode(scanner, 1);

    /* Stop BLE UART transmission.
     * NOTE: If we didn't stop the UART cleanly, it could be stopped by deep sleep in the middle
     * of a character. A zero bit would then be interpreted as break by the BL652, and in this case
     * the BL652 might stop advertising, a situation from which we would not be able to recover.
     */
    UART_ioctl(blePort, _uartFlushTx);

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

    //TODO
    LPC_SYSCON->PDSLEEPCFG0 |= 0
            | (1u << 4)         /* FRO off */
            | (1u << 7)         /* BOD Reset off */
            | (1u << 8)         /* BOD IRQ off */
            | (1u << 17)        /* ROM off */
            ;
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
    handle->monitor = false;
    SYS_enableDetector(handle, handle->currentFrequency, detector);

    /* Start M0 */
    LPC_SYSCON->CPSTACK = ((volatile uint32_t *)&M0IMAGE_start)[0];
    LPC_SYSCON->CPBOOT = ((volatile uint32_t *)&M0IMAGE_start)[1];
    LPC_SYSCON->CPUCTRL = ((LPC_SYSCON->CPUCTRL | 0xC0C40000) | (1 << 3)) & ~(1 << 5);

#if (BOARD_RA == 2)
    /* Install handler for link-down signal from Bluetooth module */
    GPIO_ioctl(bleLinkDownHandlerEnable);
#endif
}



/* Read a new RSSI value in dBm. */
LPCLIB_Result SYS_readRssi (SYS_Handle handle, float *rssi)
{
    (void)handle;

    float dBm;

    /* Get a new RSSI value from radio */
    ADF7021_readRSSI(radio, &dBm);

    /* Correct for LNA gain */
    if (handle->attenuatorActive) {
        dBm += config_g->rssiCorrectionLnaOff;
    }
    else {
        dBm += config_g->rssiCorrectionLnaOn;
    }

    *rssi = dBm;

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

    static float level;
    float adjustedLevel;
    float newLevel;

    /* Get a new RSSI value from radio (value comes as integer in tenth of a dB */
    ADF7021_readRSSI(radio, &newLevel);

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
    if (handle->attenuatorActive) {
        adjustedLevel += config_g->rssiCorrectionLnaOff;
    }
    else {
        adjustedLevel += config_g->rssiCorrectionLnaOn;
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

    /* PIO1_5: switch to analog mode */
    IOCON_configurePin(PIN_P1_5, IOCON_makeConfigA(PIN_FUNCTION_0,              /* ADC0_8       VBAT_P_ADC */
                                                   PIN_PULL_NONE,
                                                   PIN_INPUT_NOT_INVERTED,
                                                   PIN_ADMODE_ANALOG,
                                                   PIN_FILTER_OFF,
                                                   PIN_OPENDRAIN_OFF));

    //TODO need at least 10µs delay here before ADC can be enabled */
    volatile int delay = SystemCoreClock / 100000;
    while (delay--);

    LPC_ADC0->STARTUP = 0
        | (1 << 0)                          /* Enable */
        ;
    while (LPC_ADC0->STARTUP & (1u << 1))   /* Wait for end of initialization */
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
    if (GPIO_readBit(GPIO_0_19) == 0) {     /* Board Ra2fix */
        LPC_ADC0->SEQA_CTRL |= (1u << 7);   /* ADC0_7 probes negative battery node */
    }
    LPC_ADC0->SEQA_CTRL |= (1u << 18);      /* Seq A: Positive edge trigger */
    LPC_ADC0->SEQA_CTRL |= (1u << 31);      /* Seq A: Enable */
    LPC_ADC0->SEQA_CTRL |= (1u << 26);      /* Seq A: Start single sequence */

    do {
        vbatAdcDat = LPC_ADC0->DAT8;
    } while (!(vbatAdcDat & (1u << 31)));   /* Wait until there is a valid result */

    /* Convert to real battery voltage */
    if (GPIO_readBit(GPIO_0_19) == 0) {     /* Board Ra2fix */
        vbat = ((float)(vbatAdcDat & 0xFFFF) - (float)(LPC_ADC0->DAT7 & 0xFFFF)) / 65536.0f * 3.0f * 2;
    }
    else {
        vbat = (float)(vbatAdcDat & 0xFFFF) / 65536.0f * 3.0f * 2;
    }

    /* PIO1_5: back to digital mode (analog mode pins draw extra input curent when VDDA is off! */
    IOCON_configurePinDefault(PIN_P1_5,  PIN_FUNCTION_0, PIN_PULL_NONE);

    CLKPWR_unitPowerDown(CLKPWR_UNIT_VREFP);
    CLKPWR_unitPowerDown(CLKPWR_UNIT_VDDA);
    CLKPWR_unitPowerDown(CLKPWR_UNIT_ADC0);
    CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_ADC0);

    /* Disable voltage divider for VBAT */
    GPIO_writeBit(GPIO_VBAT_ADC_ENABLE, 1);
#endif

    return vbat * CONFIG_getVbatTrim();
}


static const ADF7021_Config _SYS_radioConfigBeforeAttenuatorOn[] = {
    {.opcode = ADF7021_OPCODE_SET_AGC,
        {.agc = {
            .mode = ADF7021_AGCMODE_MANUAL,
            .lnaGain = 1,
            .filterGain = 1, }}},

    ADF7021_CONFIG_END
};

static const ADF7021_Config _SYS_radioConfigAfterAttenuatorOn[] = {
    {.opcode = ADF7021_OPCODE_SET_AGC,
        {.agc = {
            .mode = ADF7021_AGCMODE_AUTO,
            .lnaGain = 1,
            .filterGain = 1, }}},

    ADF7021_CONFIG_END
};

static const ADF7021_Config _SYS_radioConfigBeforeAttenuatorOff[] = {
    {.opcode = ADF7021_OPCODE_SET_AGC,
        {.agc = {
            .mode = ADF7021_AGCMODE_MANUAL,
            .lnaGain = 0,
            .filterGain = 0, }}},

    ADF7021_CONFIG_END
};

static const ADF7021_Config _SYS_radioConfigAfterAttenuatorOff[] = {
    {.opcode = ADF7021_OPCODE_SET_AGC,
        {.agc = {
            .mode = ADF7021_AGCMODE_AUTO,
            .lnaGain = 0,
            .filterGain = 0, }}},

    ADF7021_CONFIG_END
};


/* Control the attenuator (LNA) */
static void SYS_controlAutoAttenuator (SYS_Handle handle, float dBm)
{
    if (GPIO_readBit(GPIO_LNA_GAIN) == 0) {
        /* Disable attenuator if level falls below -80 dBm */
        if (dBm <= -80.0f) {
            /* Set AGC state (filter/LNA gain stages) in ADF7021 manually to the values
             * expected after switching the attenuator.
             * Then reenable automatic mode afterwards, so the AGC will ideally not have
             * to perform any control step.
             */
            ADF7021_ioctl(radio, _SYS_radioConfigBeforeAttenuatorOff);
            GPIO_writeBit(GPIO_LNA_GAIN, 1);
            ADF7021_ioctl(radio, _SYS_radioConfigAfterAttenuatorOff);
        }
    }
    else {
        /* Enable attenuator if level reaches -70 dBm */
        if (dBm >= -70.0f) {
            /* Set AGC state (filter/LNA gain stages) in ADF7021 manually to the values
             * expected after switching the attenuator.
             * Then reenable automatic mode afterwards, so the AGC will ideally not have
             * to perform any control step.
             */
            ADF7021_ioctl(radio, _SYS_radioConfigBeforeAttenuatorOn);
            GPIO_writeBit(GPIO_LNA_GAIN, 0);
            ADF7021_ioctl(radio, _SYS_radioConfigAfterAttenuatorOn);
        }
    }

    /* Remember what state the attenuator is in */
    handle->attenuatorActive = (GPIO_readBit(GPIO_LNA_GAIN) == 0);
}



/* Return last RSSI measurement in an RX frame */
float SYS_getFrameRssi (SYS_Handle handle)
{
    return handle->lastInPacketRssi;
}


/* Submit a job for the system handler. */
LPCLIB_Result SYS_handleEvent (LPCLIB_Event event)
{
    SYS_Message *pMessage = NULL;


    if (!sysContext.queue) {
        return LPCLIB_ERROR;
    }

    if (event.id == LPCLIB_EVENTID_APPLICATION) {
        /* Handle things that can be done immediately without too much effort. */
        switch (event.opcode) {
            case APP_EVENT_HEARD_SONDE:
                {
                    float frequency = (uint32_t)event.parameter;
                    SCANNER_addListenFrequency(
                            scanner,
                            frequency,
                            (SONDE_Detector)event.block);
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
#if (BOARD_RA == 2)
osTimerDef(startupPowerControlTimer, _SYS_handleStartupTimer);
#endif


LPCLIB_Result SYS_open (SYS_Handle *pHandle)
{
    SYS_Handle handle = &sysContext;

    *pHandle = handle;
    handle->sondeType = SONDE_UNDEFINED;

    handle->hasPowerScript = false;
    BL652_hasPowerScript(ble, &handle->hasPowerScript);

    return LPCLIB_SUCCESS;
}



static void _SYS_handleBleCommand (SYS_Handle handle) {
    char *cl = handle->commandLine;

//TODO: Waking up from deep sleep causes the first character to be destorted.
//Future version of the app will send dummy lines to overcome this. For the time being
//we just accept the first distorted character anyway.
//TODO Remove in the future.
if (cl[0] != 0) {
    cl[0] = '#';
}
    /* Valid command line starts with command number, a comma, and a non-zero length payload */
    int channel;
    if (sscanf(cl, "#%d", &channel) == 1) {
        switch (channel) {
        case HOST_CHANNEL_PING:
            {
                /* If host sends a time stamp, respond by sending all current settings. */
                long timestamp = 0;
                if (sscanf(cl, "#%*d,%ld", &timestamp) == 1) {
                    handle->realTime = timestamp * 100ull;
                    handle->last_os_time = os_time;

                    /* Reset special functions */
                    handle->monitor = false;
                    handle->monitorUpdate = false;
                    SCANNER_setMode(scanner, 1);

                    /* Flush UART */
                    UART_ioctl(blePort, _uartFlushTx);

                    /* Send some dummy lines */
                    UART_write(blePort, "\r\r\r", 3);

                    /* Send application and version */
                    char s[140];
                    int hardwareVersion = 0;
#if (BOARD_RA == 1)
                    hardwareVersion = 1;        /* Ra1 */
#endif
#if (BOARD_RA == 2)
                    hardwareVersion = 2;        /* Ra2 */
                    if (GPIO_readBit(GPIO_DETECT_RA2FIX) == 0) {
                        hardwareVersion = 3;    /* Ra2fix */

                        _Bool isWideband = false;
                        if (ADF7021_getWideband(radio, &isWideband) == LPCLIB_SUCCESS) {
                            if (isWideband) {
                                hardwareVersion = 4;    /* ADF7021, not ADF7021-N */
                            }
                        }
                    }
#endif
                    uint32_t bleFirmwareVersion = 0;
                    BL652_getFirmwareVersion(ble, &bleFirmwareVersion);

                    snprintf(s, sizeof(s), "1,%d,%d,%d,%s,%d,%"PRIu32",%d,%d",
                            FIRMWARE_VERSION_MAJOR,
                            hardwareVersion,
                            FIRMWARE_VERSION_MINOR,
                            FIRMWARE_NAME,
                            config_g->serialNumber,
                            bleFirmwareVersion,
                            handle->hasPowerScript ? 1 : 0,
                            BOOTLOADER_getVersion()
                            );
                    SYS_send2Host(HOST_CHANNEL_PING, s);

                    /* Send status */
                    _SYS_reportRadioFrequency(handle);
                    _SYS_reportControls(handle);
                    _SYS_reportEphemerisAge(handle);

                    //TODO send only if ping parameter asks for it
                    RS41_resendLastPositions(handle->rs41);
                    RS92_resendLastPositions(handle->rs92);
                    DFM_resendLastPositions(handle->dfm);
                    SRSC_resendLastPositions(handle->srsc);
                    IMET_resendLastPositions(handle->imet);
                    IMET54_resendLastPositions(handle->imet54);
                    JINYANG_resendLastPositions(handle->jinyang);
                    M10_resendLastPositions(handle->m10);
                    M20_resendLastPositions(handle->m20);
                    MRZ_resendLastPositions(handle->mrz);
                    BEACON_resendLastPositions(handle->beacon);
                    PILOT_resendLastPositions(handle->pilot);
                    MEISEI_resendLastPositions(handle->meisei);
                    CF06_resendLastPositions(handle->cf06);
                    GTH3_resendLastPositions(handle->gth3);
                    PSB3_resendLastPositions(handle->psb3);
                    WINDSOND_resendLastPositions(handle->windsond);
                    MTS01_resendLastPositions(handle->mts01);

                    handle->linkEstablished = true;
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
                    char s[40];

                    detector = SONDE_DETECTOR_RS41_RS92;
                    switch (selector) {
                        case 0:     detector = SONDE_DETECTOR_RS41_RS92; break;
                        case 1:     detector = SONDE_DETECTOR_DFM; break;
                        case 2:     detector = SONDE_DETECTOR_C34_C50; break;
                        case 3:     detector = SONDE_DETECTOR_IMET; break;
                        case 4:     detector = SONDE_DETECTOR_MODEM; break;
                        case 5:     detector = SONDE_DETECTOR_BEACON; break;
                        case 6:     detector = SONDE_DETECTOR_MEISEI; break;
                        case 7:     detector = SONDE_DETECTOR_PILOT; break;
                        case 8:     detector = SONDE_DETECTOR_JINYANG; break;
                        case 9:     detector = SONDE_DETECTOR_IMET54; break;
                        case 10:    detector = SONDE_DETECTOR_MRZ; break;
                        case 11:    detector = SONDE_DETECTOR_ASIA1; break;
                        case 12:    detector = SONDE_DETECTOR_PSB3; break;
                        case 13:    detector = SONDE_DETECTOR_WINDSOND; break;
                        case 14:    detector = SONDE_DETECTOR_MTS01; break;
                    }
                    SCANNER_setManualSondeDetector(scanner, detector);
                    SONDE_Detector sondeDetector = SCANNER_getManualSondeDetector(scanner);
                    snprintf(s, sizeof(s), "5,%d", (int)sondeDetector);
                    SYS_send2Host(HOST_CHANNEL_GUI, s);
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
//                            SCANNER_addListenFrequency(scanner, frequency, SONDE_DECODER_RS41_RS92);  //TODO TODO
//                        SCANNER_setManualMode(scanner, true);
                    }

                    if (list == 1) {        /* Sonde lists in decoders */
                        if (action == 0) {  /* Remove an entry */
                            int id;
                            int decoderCode;
                            if (sscanf(cl, "#%*d,%*d,%*d,%d,%d", &id, &decoderCode) == 2) {
                                SONDE_Decoder decoder = (SONDE_Decoder)decoderCode;
                                SONDE_Detector detector = _SONDE_DETECTOR_UNDEFINED_;
                                float frequency = NAN;
                                switch (decoder) {
                                    case SONDE_DECODER_MODEM:
                                        M10_removeFromList(handle->m10, id, &frequency);
                                        M20_removeFromList(handle->m20, id, &frequency);
                                        detector = SONDE_DETECTOR_MODEM;
                                        break;
                                    case SONDE_DECODER_RS41:
                                        RS41_removeFromList(handle->rs41, id, &frequency);
                                        detector = SONDE_DETECTOR_RS41_RS92;
                                        break;
                                    case SONDE_DECODER_RS92:
                                        RS92_removeFromList(handle->rs92, id, &frequency);
                                        detector = SONDE_DETECTOR_RS41_RS92;
                                        break;
                                    case SONDE_DECODER_DFM:
                                        DFM_removeFromList(handle->dfm, id, &frequency);
                                        detector = SONDE_DETECTOR_DFM;
                                        break;
                                    case SONDE_DECODER_C34_C50:
                                        SRSC_removeFromList(handle->srsc, id, &frequency);
                                        detector = SONDE_DETECTOR_C34_C50;
                                        break;
                                    case SONDE_DECODER_BEACON:
                                        BEACON_removeFromList(handle->beacon, id, &frequency);
                                        detector = SONDE_DETECTOR_BEACON;
                                        break;
                                    case SONDE_DECODER_PILOT:
                                        PILOT_removeFromList(handle->pilot, id, &frequency);
                                        detector = SONDE_DETECTOR_PILOT;
                                        break;
                                    case SONDE_DECODER_MEISEI:
                                        MEISEI_removeFromList(handle->meisei, id, &frequency);
                                        detector = SONDE_DETECTOR_MEISEI;
                                        break;
                                    case SONDE_DECODER_JINYANG:
                                        JINYANG_removeFromList(handle->jinyang, id, &frequency);
                                        detector = SONDE_DETECTOR_JINYANG;
                                        break;
                                    case SONDE_DECODER_WINDSOND:
                                        WINDSOND_removeFromList(handle->windsond, id, &frequency);
                                        detector = SONDE_DETECTOR_WINDSOND;
                                        break;
                                    case SONDE_DECODER_MRZ:
                                        MRZ_removeFromList(handle->mrz, id, &frequency);
                                        detector = SONDE_DETECTOR_MRZ;
                                        break;
                                    case SONDE_DECODER_CF06:
                                        CF06_removeFromList(handle->cf06, id, &frequency);
                                        detector = SONDE_DETECTOR_ASIA1;
                                        break;
                                    case SONDE_DECODER_GTH3:
                                        GTH3_removeFromList(handle->gth3, id, &frequency);
                                        detector = SONDE_DETECTOR_ASIA1;
                                        break;
                                    case SONDE_DECODER_PSB3:
                                        PSB3_removeFromList(handle->psb3, id, &frequency);
                                        detector = SONDE_DETECTOR_PSB3;
                                        break;
                                    case SONDE_DECODER_IMET54:
                                        IMET54_removeFromList(handle->imet54, id, &frequency);
                                        detector = SONDE_DETECTOR_IMET54;
                                        break;
                                    case SONDE_DECODER_MTS01:
                                        MTS01_removeFromList(handle->mts01, id, &frequency);
                                        detector = SONDE_DETECTOR_MTS01;
                                        break;
                                    default:
                                        /* ignore */
                                        break;
                                }

                                if (!isnan(frequency)) {
                                    SCANNER_removeListenFrequency(scanner, frequency, detector);
                                }
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
                int extra1;
                float floatExtra1;
                float floatExtra2;

                if (sscanf(cl, "#%*d,%d", &command) == 1) {
                    switch (command) {
                        case 3:
                            if (sscanf(cl, "#%*d,%*d,%d", &enableValue) == 1) {
                                SCANNER_setMode(scanner, enableValue);
                                if ((enableValue == 2) || (enableValue == 3)) {     /* Spectrum scan mode */
                                    handle->currentFrequency = 0;
                                    _SYS_reportRadioFrequency(handle);
                                }
                                osTimerStart(handle->rssiTick, 40);
                            }
                            break;

                        case 4:
                        {
                            int selector;
                            SONDE_Detector detector;

                            if (sscanf(cl, "#%*d,%*d,%d", &selector) == 1) {
                                char s[40];

                                detector = SONDE_DETECTOR_RS41_RS92;
                                switch (selector) {
                                    case 0:     detector = SONDE_DETECTOR_RS41_RS92; break;
                                    case 1:     detector = SONDE_DETECTOR_DFM; break;
                                    case 2:     detector = SONDE_DETECTOR_C34_C50; break;
                                    case 3:     detector = SONDE_DETECTOR_IMET; break;
                                    case 4:     detector = SONDE_DETECTOR_MODEM; break;
                                    case 5:     detector = SONDE_DETECTOR_BEACON; break;
                                    case 6:     detector = SONDE_DETECTOR_MEISEI; break;
                                    case 7:     detector = SONDE_DETECTOR_PILOT; break;
                                    case 8:     detector = SONDE_DETECTOR_JINYANG; break;
                                    case 9:     detector = SONDE_DETECTOR_IMET54; break;
                                    case 10:    detector = SONDE_DETECTOR_MRZ; break;
                                    case 11:    detector = SONDE_DETECTOR_ASIA1; break;
                                    case 12:    detector = SONDE_DETECTOR_PSB3; break;
                                    case 13:    detector = SONDE_DETECTOR_WINDSOND; break;
                                    case 14:    detector = SONDE_DETECTOR_MTS01; break;
                                }
                                SCANNER_setManualSondeDetector(scanner, detector);
                                SONDE_Detector sondeDetector = SCANNER_getManualSondeDetector(scanner);
                                snprintf(s, sizeof(s), "5,%d", (int)sondeDetector);
                                SYS_send2Host(HOST_CHANNEL_GUI, s);
                            }
                            break;
                        }

                        case 5:
                        {
                            if (sscanf(cl, "#%*d,%*d,%d,%d", &enableValue, &extra1) == 2) {
                                //TODO: Clean up this mess...
                                RS41_LogMode mode = RS41_LOGMODE_NONE;
                                if (enableValue == 1) {
                                    mode = RS41_LOGMODE_RAW;
                                }
                                RS41_setLogMode(handle->rs41, extra1, mode);

                                RS92_LogMode mode92 = RS92_LOGMODE_NONE;
                                if (enableValue == 1) {
                                    mode92 = RS92_LOGMODE_RAW;
                                }
                                RS92_setLogMode(handle->rs92, extra1, mode92);
                            }
                            break;
                        }

                        case 6:
                        {
                            if (sscanf(cl, "#%*d,%*d,%d,%f", &enableValue, &floatExtra2) == 2) {
                                RS92_setSatelliteSnrThreshold(handle->rs92, floatExtra2);
                            }
                            break;
                        }

                        case 7:
                        {
                            if (sscanf(cl, "#%*d,%*d,%f,%f", &floatExtra1, &floatExtra2) == 2) {
                                SCANNER_setSpectrumRange(scanner, floatExtra1, floatExtra2);
                            }
                            break;
                        }

                        case 8:
                        {
                            if (sscanf(cl, "#%*d,%*d,%d", &enableValue) == 1) {
                                handle->monitor = enableValue != 0;
                                handle->monitorUpdate = true;
                                SYS_enableDetector(sys, handle->currentFrequency, handle->sondeDetector);
                                _SYS_reportControls(handle);
                            }
                            break;
                        }
                    }
//                    _SYS_reportControls(handle);
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

        case HOST_CHANNEL_DEBUG:
            {
                int function;
                int mode;

                if (sscanf(cl, "#%*d,%d", &function) == 1) {
                    switch (function) {
                        case 13:
                            if (sscanf(cl, "#%*d,%*d,%d", &mode) == 1) {
                                /* Send USB audio test mode to all decoders */
                                BEACON_selectDebugAudio(mode);
                                IMET_selectDebugAudio(mode);
                                SRSC_selectDebugAudio(mode);
                            }
                            break;
                    }
                }
            }
            break;

        case 99:        /* Keep-alive message */
            handle->linkEstablished = true;
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

    /* Data from Bluetooth link? */
    if (UART_readLine(blePort, handle->commandLine, sizeof(handle->commandLine)) > 0) {
        haveEvent = true;
    }
#if (BOARD_RA == 2)
    else if (USBSERIAL_readLine(handle->commandLine, sizeof(handle->commandLine)) > 0) {
        haveEvent = true;
    }
#endif
    else {
        /* There may have been partial data already copied into commandLine buffer (no complete line yet).
         * Make sure this won't be misinterpreted as a command.
         */
        handle->commandLine[0] = 0;
    }

    return haveEvent;
}


static const UART_Config blePortConfigBreakEnable[] = {
    {.opcode = UART_OPCODE_SET_BREAK,
        {.enableBreak = LPCLIB_YES, }},

    UART_CONFIG_END
};

static const UART_Config blePortConfigBreakDisable[] = {
    {.opcode = UART_OPCODE_SET_BREAK,
        {.enableBreak = LPCLIB_NO, }},

    UART_CONFIG_END
};


PT_THREAD(SYS_thread (SYS_Handle handle))
{
    static SYS_Message *pMessage;


    PT_BEGIN(&handle->pt);

    sysContext.queue = osMailCreate(osMailQ(sysQueueDef), NULL);
    GPIO_ioctl(uartRxdWakeupInit);

    EPHEMERIS_init();

    RS41_open(&handle->rs41);
    RS92_open(&handle->rs92);
    DFM_open(&handle->dfm);
    BEACON_open(&handle->beacon);
    SRSC_open(&handle->srsc);
    IMET_open(&handle->imet);
    IMET54_open(&handle->imet54);
    JINYANG_open(&handle->jinyang);
    M10_open(&handle->m10);
    M20_open(&handle->m20);
    MEISEI_open(&handle->meisei);
    WINDSOND_open(&handle->windsond);
    MRZ_open(&handle->mrz);
    PILOT_open(&handle->pilot);
    CF06_open(&handle->cf06);
    GTH3_open(&handle->gth3);
    PSB3_open(&handle->psb3);
    MTS01_open(&handle->mts01);
    PDM_open(0, &handle->pdm);

    handle->rssiTick = osTimerCreate(osTimer(rssiTimer), osTimerPeriodic, (void *)SYS_TIMERMAGIC_RSSI);
    osTimerStart(handle->rssiTick, 40);

    /* Detector for Bluetooth link status.
     * With a script in the BL652, disconnection is signalled by the module via the
     * BLE_EXTRA1 line going low. With no such script, receiving no data from the
     * host for some time is treated as a disconnect event.
     */
    handle->inactivityTimeout = osTimerCreate(osTimer(inactivityTimer), osTimerOnce, (void *)SYS_TIMERMAGIC_INACTIVITY);
#if (BOARD_RA == 1)
    osTimerStart(handle->inactivityTimeout, INACTIVITY_TIMEOUT);
#else
    handle->startupPowerControlTimeout = osTimerCreate(osTimer(startupPowerControlTimer), osTimerOnce, (void *)SYS_TIMERMAGIC_STARTUP_DELAY);
    osTimerStart(handle->startupPowerControlTimeout, STARTUP_POWERCONTROL_DELAY);
    if (!handle->hasPowerScript) {
        osTimerStart(handle->inactivityTimeout, INACTIVITY_TIMEOUT);
    }
#endif

    while (1) {
        /* Wait for an event */
        PT_WAIT_UNTIL(&handle->pt, _SYS_checkEvent(handle));

        /* Change TX break for BLE port? */
        if (handle->blePortSetBreak) {
            handle->blePortSetBreak = false;

            if (handle->blePortBreakTime != 0) {
                UART_ioctl(blePort, blePortConfigBreakEnable);
            }
            else {
                UART_ioctl(blePort, blePortConfigBreakDisable);
            }
        }

        /* Message via Bluetooth? */
        if (strlen(handle->commandLine) > 0) {
            if (!handle->hasPowerScript) {
                osTimerStart(handle->inactivityTimeout, INACTIVITY_TIMEOUT);
            }

            _SYS_handleBleCommand(handle);
            handle->commandLine[0] = 0;
        }

        /* Update estimated real time */
        uint32_t delta_t = os_time - handle->last_os_time;
        if (delta_t > 0) {
            handle->realTime += delta_t;
            handle->last_os_time += delta_t;
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
                        float rxOffset = 0;
#if (BOARD_RA == 2)
                        rxOffset = PDM_getDcOffset(handle->pdm) / (-5.3f);     //TODO factor
#endif
                        SRSC_processBlock(
                                    handle->srsc,
                                    (uint8_t *)pMessage->event.parameter,
                                    handle->currentFrequency,
                                    rxOffset,
                                    SYS_getFrameRssi(handle),
                                    handle->realTime);
                    }
                    else if ((SONDE_Type)pMessage->event.block == SONDE_IMET_RSB) {
                        float rxOffset = 0;
#if (BOARD_RA == 2)
                        rxOffset = PDM_getDcOffset(handle->pdm) / (-2.65f);     //TODO factor
#endif
                        IMET_processBlock(
                                    handle->imet,
                                    (uint8_t *)pMessage->event.parameter,
                                    handle->currentFrequency,
                                    rxOffset,
                                    SYS_getFrameRssi(handle),
                                    handle->realTime);
                    }
                    else if ((SONDE_Type)pMessage->event.block == SONDE_BEACON) {
                        BEACON_processBlock(
                                    handle->beacon,
                                    (uint8_t *)pMessage->event.parameter,
                                    handle->currentFrequency,
                                    pMessage->event.channel,
                                    SYS_getFrameRssi(handle),
                                    handle->realTime);
                    }
                    break;
                }
                break;

            case SYS_OPCODE_BUFFER_COMPLETE:
                {
                    /* Find out which buffer to use */
                    int bufferIndex = pMessage->bufferIndex;

                    if (ipc[bufferIndex].valid) {
                        /* Ignore glitches (packets arriving while switching to/from scanner mode) */
                        if (handle->currentFrequency != 0) {
                            SONDE_Type sondeType = SONDE_UNDEFINED;
                            switch (ipc[bufferIndex].param) {
                                case 0:  sondeType = SONDE_RS92; break;
                                case 1:  sondeType = SONDE_RS41; break;
                                case 2:  sondeType = SONDE_DFM_INVERTED; break;
                                case 3:  sondeType = SONDE_DFM_NORMAL; break;
                                case 4:  sondeType = SONDE_M10; break;
                                case 5:  sondeType = SONDE_M20; break;
                                case 7:  sondeType = SONDE_PILOT; break;
                                case 8:  sondeType = SONDE_MEISEI_CONFIG; break;
                                case 9:  sondeType = SONDE_MEISEI_GPS; break;
                                case 10: sondeType = SONDE_RSG20; break;
                                case 11: sondeType = SONDE_WINDSOND_S1; break;
                                case 12: sondeType = SONDE_MRZ; break;
                                case 13: sondeType = SONDE_GTH3_CF06AH; break;
                                case 14: sondeType = SONDE_PSB3; break;
                                case 15: sondeType = SONDE_IMET54; break;
                                case 16: sondeType = SONDE_MTS01; break;
                            }

                            /* Process buffer */
                            if (sondeType == SONDE_M10) {
                                M10_processBlock(
                                        handle->m10,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime);

                                /* Let scanner prepare for next frequency */
                                SCANNER_notifyValidFrame(scanner);
                            }
                            else if (sondeType == SONDE_M20) {
                                M20_processBlock(
                                        handle->m20,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime);

                                /* Let scanner prepare for next frequency */
                                SCANNER_notifyValidFrame(scanner);
                            }
                            else if (sondeType == SONDE_PILOT) {
                                PILOT_processBlock(
                                        handle->pilot,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime);

                                /* Let scanner prepare for next frequency */
                                SCANNER_notifyValidFrame(scanner);
                            }
                            else if (sondeType == SONDE_RS41) {
                                RS41_processBlock(
                                        handle->rs41,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime);

                                /* Let scanner prepare for next frequency */
                                SCANNER_notifyValidFrame(scanner);
                            }
                            else if (sondeType == SONDE_RS92) {
                                RS92_processBlock(
                                        handle->rs92,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime);

                                /* Let scanner prepare for next frequency */
                                SCANNER_notifyValidFrame(scanner);
                            }
                            else if (sondeType == SONDE_DFM_NORMAL) {
                                if (DFM_processBlock(
                                        handle->dfm,
                                        sondeType,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        ipc[bufferIndex].rxTime,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime) == LPCLIB_SUCCESS) {
                                    /* Frame complete. Let scanner prepare for next frequency */
                                    SCANNER_notifyValidFrame(scanner);
                                }
                            }
                            else if (sondeType == SONDE_DFM_INVERTED) {
                                if (DFM_processBlock(
                                        handle->dfm,
                                        sondeType,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        ipc[bufferIndex].rxTime,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime) == LPCLIB_SUCCESS) {
                                    /* Frame complete. Let scanner prepare for next frequency */
                                    SCANNER_notifyValidFrame(scanner);
                                }
                            }
                            else if ((sondeType == SONDE_MEISEI_CONFIG) || (sondeType == SONDE_MEISEI_GPS)) {
                                if (MEISEI_processBlock(
                                        handle->meisei,
                                        sondeType,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime) == LPCLIB_SUCCESS) {
                                    /* Frame complete. Let scanner prepare for next frequency */
                                    SCANNER_notifyValidFrame(scanner);
                                }
                            }
                            else if (sondeType == SONDE_RSG20) {
                                if (JINYANG_processBlock(
                                        handle->jinyang,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime) == LPCLIB_SUCCESS) {
                                    /* Frame complete. Let scanner prepare for next frequency */
                                    SCANNER_notifyValidFrame(scanner);
                                }
                            }
                            else if (sondeType == SONDE_WINDSOND_S1) {
                                if (WINDSOND_processBlock(
                                        handle->windsond,
                                        sondeType,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime) == LPCLIB_SUCCESS) {
                                    /* Frame complete. Let scanner prepare for next frequency */
                                    SCANNER_notifyValidFrame(scanner);
                                }
                            }
                            else if (sondeType == SONDE_MRZ) {
                                if (MRZ_processBlock(
                                        handle->mrz,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime) == LPCLIB_SUCCESS) {
                                    /* Frame complete. Let scanner prepare for next frequency */
                                    SCANNER_notifyValidFrame(scanner);
                                }
                            }
                            else if (sondeType == SONDE_GTH3_CF06AH) {
                                if (CHINA1_processBlock(
                                        handle->cf06,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime) == LPCLIB_SUCCESS) {
                                    /* Frame complete. Let scanner prepare for next frequency */
                                    SCANNER_notifyValidFrame(scanner);
                                }
                            }
                            else if (sondeType == SONDE_PSB3) {
                                if (PSB3_processBlock(
                                        handle->psb3,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime) == LPCLIB_SUCCESS) {
                                    /* Frame complete. Let scanner prepare for next frequency */
                                    SCANNER_notifyValidFrame(scanner);
                                }
                            }
                            else if (sondeType == SONDE_IMET54) {
                                if (IMET54_processBlock(
                                        handle->imet54,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime) == LPCLIB_SUCCESS) {
                                    /* Frame complete. Let scanner prepare for next frequency */
                                    SCANNER_notifyValidFrame(scanner);
                                }
                            }
                            else if (sondeType == SONDE_MTS01) {
                                if (MTS01_processBlock(
                                        handle->mts01,
                                        ipc[bufferIndex].data8,
                                        ipc[bufferIndex].numBits,
                                        handle->currentFrequency,
                                        SYS_getFrameRssi(handle),
                                        handle->realTime) == LPCLIB_SUCCESS) {
                                    /* Frame complete. Let scanner prepare for next frequency */
                                    SCANNER_notifyValidFrame(scanner);
                                }
                            }
                        }

                        /* Buffer may now be reused */
                        ipc[bufferIndex].valid = 0;
                    }
                }
                break;

            case SYS_OPCODE_GET_RSSI:
                {
                    static int rate1;
                    static int rate2;

                    /* Send RSSI only when not in scanner mode */
                    if (SCANNER_getMode(scanner) != 2) {
                        if (++rate1 >= 2) {
                            handle->currentRssi = _SYS_getFilteredRssi(handle);

                            /* Do not fill TX queue before link established */
                            if (handle->linkEstablished) {
                                char s[30];
                                sprintf(s, "3,%.1f", handle->currentRssi);
                                SYS_send2Host(HOST_CHANNEL_GUI, s);
                            }

                            rate1 = 0;
                            SYS_controlAutoAttenuator(handle, handle->currentRssi);
                        }
                    }
                    else {
                        /* Send dummy value at a low rate to ensure S-meter shows minimum */
                        if (rate2 == 0) {
                            SYS_send2Host(HOST_CHANNEL_GUI, "3,");
                        }

                        if (++rate2 >= 100) {
                            rate2 = 0;
                        }
                    }

                    /* Low frequency VBAT reports */
                    static int rate3 = 0;
                    if (++rate3 >= 200) {
                        rate3 = 0;
                        _SYS_reportVbat(handle);
                    }

                    /* On Ra hardware prior to Ra2fix the VBAT measurement has a systematic error.
                     * The measurement can be too low if there is a voltage drop on a resistor
                     * in series with the battery.
                     * Therefore the following filter is implemented:
                     * Several consecutive raw VBAT measurements are stored in the 'vbatFilter'.
                     * Then the average of all these samples is calculated. This average is expected to be
                     * below the real value, because it includes samples taken where the systematic error
                     * caused the result to be too low.
                     * Now a second average value is taken including only those samples lying above the
                     * previously calculated average over all samples. This new average ideally uses only
                     * 'valid' samples, and the result is used as the consolidated VBAT result.
                     */
                    /* Store new sample */
                    handle->vbatFilter[handle->vbatFilterIndex] = _SYS_measureVbat(handle);
                    ++handle->vbatFilterIndex;
                    if (handle->vbatFilterIndex >= VBAT_FILTER_LENGTH) {
                        handle->vbatFilterIndex = 0;
                    }

                    /* Calculate first average (includes invalid samples) */
                    float f = 0;
                    int i;
                    for (i = 0; i < VBAT_FILTER_LENGTH; i++) {
                        f += handle->vbatFilter[i];
                    }
                    float favg = f / VBAT_FILTER_LENGTH;

                    /* Do it again, but now drop all samples below the average */
                    f = 0;
                    int nValid = 0;
                    for (i = 0; i < VBAT_FILTER_LENGTH; i++) {
                        if (handle->vbatFilter[i] >= favg) {
                            f += handle->vbatFilter[i];
                            ++nValid;
                        }
                    }

                    /* This is our consolidated result */
                    handle->vbat = f / nValid;
                }
                break;
            }

            osMailFree(sysContext.queue, pMessage);
        }
    }

    PT_END(&handle->pt);
}


