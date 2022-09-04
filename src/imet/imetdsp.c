
#include <math.h>
#include <stdio.h>

#include "lpclib.h"
#include "sys.h"
#include "imetprivate.h"
#include "bsp.h"
#if (BOARD_RA == 2)
#include "usbuser_config.h"
#endif


#define UART_BUFFER_SIZE        64
#define IMET_UART_BAUDRATE      1200
#define IMET_NUM_FRAMES         10
#define IMET_MAX_FRAME_LENGTH   50


enum {
    IMET_DEBUGAUDIO_NORMAL = 0,

    IMET_DEBUGAUDIO_1,

    __IMET_DEBUGAUDIO_MAX__
};


#if (BOARD_RA == 1)
/* Audio sample rate is 16169 Hz. Oscillator frequencies are:
 * OSC1: 1198 Hz (ideal: 1200 Hz)
 * OSC0: 2205 Hz (ideal: 2200 Hz)
 */

#define OSC1_N              27
static const float osc1_sin[OSC1_N] = {
     0.000000,  0.448799,  0.802123,  0.984808,  0.957990,
     0.727374,  0.342020, -0.116093, -0.549509, -0.866025,
    -0.998308, -0.918216, -0.642788, -0.230616,  0.230616,
     0.642788,  0.918216,  0.998308,  0.866025,  0.549509,
     0.116093, -0.342020, -0.727374, -0.957990, -0.984808,
    -0.802123, -0.448799,
};
static const float osc1_cos[OSC1_N] = {
     1.000000,  0.893633,  0.597159,  0.173648, -0.286803,
    -0.686242, -0.939693, -0.993238, -0.835488, -0.500000,
    -0.058145,  0.396080,  0.766044,  0.973045,  0.973045,
     0.766044,  0.396080, -0.058145, -0.500000, -0.835488,
    -0.993238, -0.939693, -0.686242, -0.286803,  0.173648,
     0.597159,  0.893633,
};
#define OSC0_N              22
static const float osc0_sin[OSC0_N] = {
     0.000000,  0.755750,  0.989821,  0.540641, -0.281733,
    -0.909632, -0.909632, -0.281733,  0.540641,  0.989821,
     0.755750,  0.000000, -0.755750, -0.989821, -0.540641,
     0.281733,  0.909632,  0.909632,  0.281733, -0.540641,
    -0.989821, -0.755750,
};
static const float osc0_cos[OSC0_N] = {
     1.000000,  0.654861, -0.142315, -0.841254, -0.959493,
    -0.415415,  0.415415,  0.959493,  0.841254,  0.142315,
    -0.654861, -1.000000, -0.654861,  0.142315,  0.841254,
     0.959493,  0.415415, -0.415415, -0.959493, -0.841254,
    -0.142315,  0.654861,
};
#endif

#if (BOARD_RA == 2)
/* Audio sample rate is 16000 Hz. Oscillator frequencies are:
 * OSC1: 1200 Hz (ideal: 1200 Hz)
 * OSC0: 2200 Hz (ideal: 2200 Hz)
 */

#define OSC1_N              40
static const float osc1_sin[OSC1_N] = {
     0.000000,  0.453990,  0.809017,  0.987688,  0.951057,
     0.707107,  0.309017, -0.156434, -0.587785, -0.891007,
    -1.000000, -0.891007, -0.587785, -0.156434,  0.309017,
     0.707107,  0.951057,  0.987688,  0.809017,  0.453990,
     0.000000, -0.453990, -0.809017, -0.987688, -0.951057,
    -0.707107, -0.309017,  0.156434,  0.587785,  0.891007,
     1.000000,  0.891007,  0.587785,  0.156434, -0.309017,
    -0.707107, -0.951057, -0.987688, -0.809017, -0.453990,
};
static const float osc1_cos[OSC1_N] = {
     1.000000,  0.891007,  0.587785,  0.156434, -0.309017,
    -0.707107, -0.951057, -0.987688, -0.809017, -0.453990,
    -0.000000,  0.453990,  0.809017,  0.987688,  0.951057,
     0.707107,  0.309017, -0.156434, -0.587785, -0.891007,
    -1.000000, -0.891007, -0.587785, -0.156434,  0.309017,
     0.707107,  0.951057,  0.987688,  0.809017,  0.453990,
     0.000000, -0.453990, -0.809017, -0.987688, -0.951057,
    -0.707107, -0.309017,  0.156434,  0.587785,  0.891007,
};
#define OSC0_N              80
static const float osc0_sin[OSC0_N] = {
     0.000000,  0.760406,  0.987688,  0.522499, -0.309017,
    -0.923880, -0.891007, -0.233445,  0.587785,  0.996917,
     0.707107, -0.078459, -0.809017, -0.972370, -0.453990,
     0.382683,  0.951057,  0.852640,  0.156434, -0.649448,
    -1.000000, -0.649448,  0.156434,  0.852640,  0.951057,
     0.382683, -0.453990, -0.972370, -0.809017, -0.078459,
     0.707107,  0.996917,  0.587785, -0.233445, -0.891007,
    -0.923880, -0.309017,  0.522499,  0.987688,  0.760406,
     0.000000, -0.760406, -0.987688, -0.522499,  0.309017,
     0.923880,  0.891007,  0.233445, -0.587785, -0.996917,
    -0.707107,  0.078459,  0.809017,  0.972370,  0.453990,
    -0.382683, -0.951057, -0.852640, -0.156434,  0.649448,
     1.000000,  0.649448, -0.156434, -0.852640, -0.951057,
    -0.382683,  0.453990,  0.972370,  0.809017,  0.078459,
    -0.707107, -0.996917, -0.587785,  0.233445,  0.891007,
     0.923880,  0.309017, -0.522499, -0.987688, -0.760406,
};
static const float osc0_cos[OSC0_N] = {
     1.000000,  0.649448, -0.156434, -0.852640, -0.951057,
    -0.382683,  0.453990,  0.972370,  0.809017,  0.078459,
    -0.707107, -0.996917, -0.587785,  0.233445,  0.891007,
     0.923880,  0.309017, -0.522499, -0.987688, -0.760406,
    -0.000000,  0.760406,  0.987688,  0.522499, -0.309017,
    -0.923880, -0.891007, -0.233445,  0.587785,  0.996917,
     0.707107, -0.078459, -0.809017, -0.972370, -0.453990,
     0.382683,  0.951057,  0.852640,  0.156434, -0.649448,
    -1.000000, -0.649448,  0.156434,  0.852640,  0.951057,
     0.382683, -0.453990, -0.972370, -0.809017, -0.078459,
     0.707107,  0.996917,  0.587785, -0.233445, -0.891007,
    -0.923880, -0.309017,  0.522499,  0.987688,  0.760406,
     0.000000, -0.760406, -0.987688, -0.522499,  0.309017,
     0.923880,  0.891007,  0.233445, -0.587785, -0.996917,
    -0.707107,  0.078459,  0.809017,  0.972370,  0.453990,
    -0.382683, -0.951057, -0.852640, -0.156434,  0.649448,
};
#endif


static struct _IMET_Audio {
    int osc0;
    int osc1;
    float osc[4];
    float filter[4][2];
    float filterOut[4];
    float fskOut[256];
    float fskOutFilter[2];

    struct {
        float fskIn[3];             /* Last three samples for filtering */
        int charState;              /* State machine for single character */
        uint32_t charSampleCount;   /* Counter for samples in character reception */
        uint8_t rxChar;
        uint8_t outBuf[UART_BUFFER_SIZE];
        int outWrIndex;
        uint32_t framingError;      /* Statistics... */
    } uart;

    struct {
        int state;                  /* State machine for frame */
        int length;
        int writeIndex;
        int uartRdIndex;
        int frameWrIndex;
        uint32_t lastTime;          /* Last time a character was received */
        uint8_t frames[IMET_NUM_FRAMES][IMET_MAX_FRAME_LENGTH];
    } rsb;

    int debugAudioChannel;

} _imetAudioContext;

static const float _B0 = 0.029402f;
static const float _B1 = 0.058803f;
static const float _B2 = 0.029402f;
static const float _A1 = -1.459787f;
static const float _A2 = 0.577394f;

/* Filter at FSK decoder output (1200 Hz) */
static const float _B0_FSK = 0.029402f;
static const float _B1_FSK = 0.058803f;
static const float _B2_FSK = 0.029402f;
static const float _A1_FSK = -1.459787f;
static const float _A2_FSK = 0.577394f;


/* Process last FM decoder output */
static void IMET_DSP_uart (void)
{
    int n;
    int sample;
    float f;
    struct _IMET_Audio *handle = &_imetAudioContext;

    /* Process all recently decoded samples */
    for (n = 0; n < 256; n++) {
        /* Base 0/1 decision on the average of the last three samples */
        handle->uart.fskIn[2] = handle->uart.fskIn[1];
        handle->uart.fskIn[1] = handle->uart.fskIn[0];
        handle->uart.fskIn[0] = handle->fskOut[n];
        f = (handle->uart.fskIn[2] + handle->uart.fskIn[1] + handle->uart.fskIn[0]) / 3.0f;
//f = handle->fskDecode[n];
        //TODO add hysteresis?
        sample = (f > 0) ? 1 : 0;

        /* Process UART state machine */
        switch (handle->uart.charState) {
            case 0:                             /* Waiting for start bit */
                handle->uart.charSampleCount = 0;
                if (sample == 0) {
                    handle->uart.charState = 1;
                    handle->uart.rxChar = 0;
                }
                break;

            case 1:                             /* Waiting for start bit verification */
                if (handle->uart.charSampleCount > 6) {
                    if (sample == 0) {
                        handle->uart.charState = 2;

                        /* Sample RSSI value */
                        LPC_MAILBOX->IRQ1SET = (1u << 1);
                    }
                    else {
                        handle->uart.charState = 0;
                    }
                }
                break;

            case 2:                             /* Waiting for next data bit */
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                if ((int)handle->uart.charSampleCount > 6 + (DSP_SAMPLERATE * (handle->uart.charState - 1)) / IMET_UART_BAUDRATE) {
                    handle->uart.rxChar = (handle->uart.rxChar >> 1) | (sample << 7);
                    ++handle->uart.charState;
                }
                break;

            case 10:                            /* Verify (1st) stop bit */
                if (handle->uart.charSampleCount > 6 + (DSP_SAMPLERATE * 9) / IMET_UART_BAUDRATE) {
                    if (sample == 1) {
                        handle->uart.outBuf[handle->uart.outWrIndex] = handle->uart.rxChar;
                        if (++handle->uart.outWrIndex >= UART_BUFFER_SIZE) {
                            handle->uart.outWrIndex = 0;
                        }
                    }
                    else {
                        ++handle->uart.framingError;
                    }
                    handle->uart.charState = 0;
                }
                break;

            default:                            /* Should never come here... */
                handle->uart.charState = 0;
                break;
        }

        ++handle->uart.charSampleCount;
    }
}


/* Process */
static void IMET_DSP_rsb (void)
{
    uint8_t c;
    struct _IMET_Audio *handle = &_imetAudioContext;

    /* Reset state machine after a long gap between characters */
    uint32_t gap = os_time - handle->rsb.lastTime;  //TODO get milliseconds
    handle->rsb.lastTime = os_time;
    if (gap > 100/10) {
        handle->rsb.state = 0;
    }

    /* Process all recently received characters */
    while (handle->rsb.uartRdIndex != handle->uart.outWrIndex) {
        c = handle->uart.outBuf[handle->rsb.uartRdIndex];
/*
if(c==0x01){
    printf("\r\n");
}
printf("%02X ",c);
*/
        if (++handle->rsb.uartRdIndex >= UART_BUFFER_SIZE) {
            handle->rsb.uartRdIndex = 0;
        }

        switch (handle->rsb.state) {
            case 0:
                if (c == 0x01) {
                    handle->rsb.state = 1;
                }
                break;

            case 1:
                handle->rsb.frames[handle->rsb.frameWrIndex][0] = c;
                handle->rsb.writeIndex = 1;
                switch (c) {
                    case 0x01:
                        handle->rsb.state = 3;
                        handle->rsb.length = 12;
                        break;
                    case 0x02:
                        handle->rsb.state = 3;
                        handle->rsb.length = 16;
                        break;
                    case 0x03:
                        handle->rsb.state = 2;
                        break;
                    case 0x04:
                        handle->rsb.state = 3;
                        handle->rsb.length = 18;
                        break;
                    case 0x05:
                        handle->rsb.state = 3;
                        handle->rsb.length = 28;
                        break;
                    default:
                        handle->rsb.state = 0;
                        break;
                }
                break;

            case 2:
                handle->rsb.frames[handle->rsb.frameWrIndex][1] = c;
                handle->rsb.writeIndex = 2;
                if ((c > 0) && (c <= IMET_MAX_FRAME_LENGTH - 4)) {
                    handle->rsb.state = 3;
                    handle->rsb.length = c;
                }
                else {
                    handle->rsb.state = 0;
                }
                break;

            case 3:
                handle->rsb.frames[handle->rsb.frameWrIndex][handle->rsb.writeIndex] = c;
                ++handle->rsb.writeIndex;
                if (--handle->rsb.length == 0) {
                    handle->rsb.state = 0;

                    LPCLIB_Event event;
                    LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
                    event.opcode = APP_EVENT_RAW_FRAME;
                    event.block = SONDE_IMET_RSB;
                    event.parameter = &handle->rsb.frames[handle->rsb.frameWrIndex];
                    SYS_handleEvent(event);

                    if (++handle->rsb.frameWrIndex >= IMET_NUM_FRAMES) {
                        handle->rsb.frameWrIndex = 0;
                    }
                }
                break;

            default:
                handle->rsb.state = 0;
                break;
        }
    }
}



/* Process a batch of 16-kHz audio samples */
void IMET_DSP_processAudio (const int32_t *rawAudio, float *cookedAudio, int nSamples)
{
    int n;
    int i;
    float sx, sy, stemp;
    struct _IMET_Audio *handle = &_imetAudioContext;


    /* The input signal is mixed with complex oscillators for the audio FSK
     * frequencies f1 and f2, then low-pass filtered in each I and Q branch
     * (resulting in bandpass functions for f1 and f2).
     * Output power of both filters is compared and delivered as decoded FSK.
     */
    for (n = 0; n < nSamples; n++) {
        sx = rawAudio[n] / 32768.0f;

        /* Table-based computation of the two complex oscillators */
        handle->osc[0] = osc1_cos[handle->osc1];
        handle->osc[1] = osc1_sin[handle->osc1];
        handle->osc1 = (handle->osc1 + 1) % OSC1_N;
        handle->osc[2] = osc0_cos[handle->osc0];
        handle->osc[3] = osc0_sin[handle->osc0];
        handle->osc0 = (handle->osc0 + 1) % OSC0_N;

        for (i = 0; i < 4; i++) {
            stemp = -_A2 * handle->filter[i][1] - _A1 * handle->filter[i][0] + sx * handle->osc[i];
            sy = _B2 * handle->filter[i][1] + _B1 * handle->filter[i][0] + _B0 * stemp;
            handle->filter[i][1] = handle->filter[i][0];
            handle->filter[i][0] = stemp;
            handle->filterOut[i] = sy * sy;
        }

        /* Decoded FSK */
        sy = (handle->filterOut[0] + handle->filterOut[1])
           - (handle->filterOut[2] + handle->filterOut[3]);
        sy = (sy < 0) ? -sqrtf(-sy) : sqrtf(sy);

#if 1
        /* Filter FSK decoder output */
        stemp = -_A2_FSK * handle->fskOutFilter[1] - _A1_FSK * handle->fskOutFilter[0] + sy;
        sy = _B2_FSK * handle->fskOutFilter[1] + _B1_FSK * handle->fskOutFilter[0] + _B0_FSK * stemp;
        handle->fskOutFilter[1] = handle->fskOutFilter[0];
        handle->fskOutFilter[0] = stemp;
#endif

        cookedAudio[n] = sy;
    }

    IMET_DSP_uart();
    IMET_DSP_rsb();
}


void IMET_DSP_initAudio (void)
{
    struct _IMET_Audio *handle = &_imetAudioContext;

    handle->osc0 = 0;
    handle->osc1 = 0;

    handle->osc[0] = handle->osc[1] = handle->osc[2] = handle->osc[3] = 0;
}



void IMET_handleAudioCallback (int32_t *samples, int nSamples)
{
    struct _IMET_Audio *handle = &_imetAudioContext;

    IMET_DSP_processAudio(samples, handle->fskOut, nSamples);
#if (BOARD_RA == 2)
    switch (handle->debugAudioChannel) {
        case IMET_DEBUGAUDIO_1:
            USBUSER_writeAudioStereo_i32_float(samples, handle->fskOut, nSamples);
            break;
        default:
            USBUSER_writeAudioStereo_i32_i32(samples, samples, nSamples);
            break;
    }
#endif
}


void IMET_selectDebugAudio (int debugAudioChannel)
{
    struct _IMET_Audio *handle = &_imetAudioContext;

    if (debugAudioChannel < __IMET_DEBUGAUDIO_MAX__) {
        handle->debugAudioChannel = debugAudioChannel;
    }
}


/* Discard partial decoder result */
void IMET_DSP_reset (void)
{
    // Can be done better, but...
    _imetAudioContext.rsb.state = 0;
}



