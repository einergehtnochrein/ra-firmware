#include <math.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <stdio.h>

#include "lpclib.h"
#include "sys.h"
#include "srscprivate.h"
#include "bsp.h"
#if (BOARD_RA == 2)
#include "usbuser_config.h"
#endif


#define UART_BUFFER_SIZE    32
#define C34_NUM_FRAMES      40


#if (BOARD_RA == 1)
/* Audio sample rate is 16169 Hz. Oscillator frequencies are:
 * OSC1: 2910 Hz (ideal: 2905 Hz)
 * OSC0: 4694 Hz (ideal: 4694 Hz)
 */

#define OSC1_N              50
static const float osc1_sin[OSC1_N] = {
     0.000000,  0.904827,  0.770513, -0.248690, -0.982287,
    -0.587785,  0.481754,  0.998027,  0.368125, -0.684547,
    -0.951057, -0.125333,  0.844328,  0.844328, -0.125333,
    -0.951057, -0.684547,  0.368125,  0.998027,  0.481754,
    -0.587785, -0.982287, -0.248690,  0.770513,  0.904827,
     0.000000, -0.904827, -0.770513,  0.248690,  0.982287,
     0.587785, -0.481754, -0.998027, -0.368125,  0.684547,
     0.951057,  0.125333, -0.844328, -0.844328,  0.125333,
     0.951057,  0.684547, -0.368125, -0.998027, -0.481754,
     0.587785,  0.982287,  0.248690, -0.770513, -0.904827,
};
static const float osc1_cos[OSC1_N] = {
     1.000000,  0.425779, -0.637424, -0.968583, -0.187381,
     0.809017,  0.876307, -0.062791, -0.929776, -0.728969,
     0.309017,  0.992115,  0.535827, -0.535827, -0.992115,
    -0.309017,  0.728969,  0.929776,  0.062791, -0.876307,
    -0.809017,  0.187381,  0.968583,  0.637424, -0.425779,
    -1.000000, -0.425779,  0.637424,  0.968583,  0.187381,
    -0.809017, -0.876307,  0.062791,  0.929776,  0.728969,
    -0.309017, -0.992115, -0.535827,  0.535827,  0.992115,
     0.309017, -0.728969, -0.929776, -0.062791,  0.876307,
     0.809017, -0.187381, -0.968583, -0.637424,  0.425779,
};
#define OSC0_N              31
static const float osc0_sin[OSC0_N] = {
     0.000000,  0.968077, -0.485302, -0.724793,  0.848644,
     0.299363, -0.998717,  0.201299,  0.897805, -0.651372,
    -0.571268,  0.937752,  0.101168, -0.988468,  0.394356,
     0.790776, -0.790776, -0.394356,  0.988468, -0.101168,
    -0.937752,  0.571268,  0.651372, -0.897805, -0.201299,
     0.998717, -0.299363, -0.848644,  0.724793,  0.485302,
    -0.968077,
};
static const float osc0_cos[OSC0_N] = {
     1.000000, -0.250653, -0.874347,  0.688967,  0.528964,
    -0.954139, -0.050649,  0.979530, -0.440394, -0.758758,
     0.820763,  0.347305, -0.994869,  0.151428,  0.918958,
    -0.612106, -0.612106,  0.918958,  0.151428, -0.994869,
     0.347305,  0.820763, -0.758758, -0.440394,  0.979530,
    -0.050649, -0.954139,  0.528964,  0.688967, -0.874347,
    -0.250653,
};
#endif

#if (BOARD_RA == 2)
/* Audio sample rate is 16000 Hz. Oscillator frequencies are:
 * OSC1: 2909 Hz (ideal: 2905 Hz)
 * OSC0: 4706 Hz (ideal: 4694 Hz)
 */

#define OSC1_N              11
static const float osc1_sin[OSC1_N] = {
     0.000000,  0.909632,  0.755750, -0.281733, -0.989821,
    -0.540641,  0.540641,  0.989821,  0.281733, -0.755750,
    -0.909632,
};
static const float osc1_cos[OSC1_N] = {
     1.000000,  0.415415, -0.654861, -0.959493, -0.142315,
     0.841254,  0.841254, -0.142315, -0.959493, -0.654861,
     0.415415,
};
#define OSC0_N              17
static const float osc0_sin[OSC0_N] = {
     0.000000,  0.961826, -0.526432, -0.673696,  0.895163,
     0.183750, -0.995734,  0.361242,  0.798017, -0.798017,
    -0.361242,  0.995734, -0.183750, -0.895163,  0.673696,
     0.526432, -0.961826,
};
static const float osc0_cos[OSC0_N] = {
     1.000000, -0.273663, -0.850217,  0.739009,  0.445738,
    -0.982973,  0.092268,  0.932472, -0.602635, -0.602635,
     0.932472,  0.092268, -0.982973,  0.445738,  0.739009,
    -0.850217, -0.273663,
};
#endif

static struct _SRSC_Audio {
    int osc0;
    int osc1;
    float osc[4];
    float filter[4][2];
    float filterOut[4];
#if SEMIHOSTING_C34
    FILE *fpFSK;
#endif
    float fskOut[256];

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
        int uartRdIndex;
        int frameWrIndex;
        uint8_t frames[C34_NUM_FRAMES][7];
    } c34;

} _srscAudioContext;


static const float _B0 = 0.029402f;
static const float _B1 = 0.058803f;
static const float _B2 = 0.029402f;
static const float _A1 = -1.459787f;
static const float _A2 = 0.577394f;


/* Process last FM decoder output */
static void SRSC_DSP_uart (void)
{
    int n;
    int sample;
    float f;
    struct _SRSC_Audio *handle = &_srscAudioContext;

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
                if (handle->uart.charSampleCount > 2) {
                    handle->uart.charState = (sample == 0) ? 2 : 0;
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
                if ((int)handle->uart.charSampleCount > 2 + (DSP_SAMPLERATE * (handle->uart.charState - 1)) / 2400) {
                    handle->uart.rxChar = (handle->uart.rxChar >> 1) | (sample << 7);
                    ++handle->uart.charState;
                }
                break;

            case 10:                            /* Verify (1st) stop bit */
                if (handle->uart.charSampleCount > 2 + (DSP_SAMPLERATE * 9) / 2400) {
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


/* Process C34 */
static void SRSC_DSP_c34 (void)
{
    uint8_t c;
    struct _SRSC_Audio *handle = &_srscAudioContext;

    /* Process all recently received characters */
    while (handle->c34.uartRdIndex != handle->uart.outWrIndex) {
        c = handle->uart.outBuf[handle->c34.uartRdIndex];

        if (++handle->c34.uartRdIndex >= UART_BUFFER_SIZE) {
            handle->c34.uartRdIndex = 0;
        }

        switch (handle->c34.state) {
            case 0:
                if (c == 0x00) {
                    handle->c34.state = 1;
                }
                break;

            case 1:
                if (c == 0xFF) {
                    handle->c34.state = 2;

                    /* Sample RSSI value */
                    LPC_MAILBOX->IRQ1SET = (1u << 1);
                }
                else {
                    handle->c34.state = 0;
                }
                break;

            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
                handle->c34.frames[handle->c34.frameWrIndex][handle->c34.state - 2] = c;
                ++handle->c34.state;
                break;

            case 8:
                handle->c34.frames[handle->c34.frameWrIndex][6] = c;
                handle->c34.state = 0;

                {
                    LPCLIB_Event event;
                    LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
                    event.opcode = APP_EVENT_RAW_FRAME;
                    event.block = SONDE_C34;
                    event.parameter = &handle->c34.frames[handle->c34.frameWrIndex];
                    SYS_handleEvent(event);
                }

if(((handle->c34.frames[handle->c34.frameWrIndex][0]
  +handle->c34.frames[handle->c34.frameWrIndex][1]
  +handle->c34.frames[handle->c34.frameWrIndex][2]
  +handle->c34.frames[handle->c34.frameWrIndex][3]
  +handle->c34.frames[handle->c34.frameWrIndex][4])&0xFF)
 ==handle->c34.frames[handle->c34.frameWrIndex][5]){
                if (++handle->c34.frameWrIndex >= C34_NUM_FRAMES) {
                    handle->c34.frameWrIndex = 0;
                }
}
                break;

            default:
                handle->c34.state = 0;
                break;
        }
    }
}


uint32_t timeDSP[8];


/* Process a batch of 16-kHz audio samples */
void SRSC_DSP_processAudio (const int32_t *rawAudio, float *cookedAudio, int nSamples)
{
    int n;
    int i;
    float sx, sy, stemp;
    struct _SRSC_Audio *handle = &_srscAudioContext;


    /* The input signal is mixed with complex oscillators for the audio FSK
     * frequencies f1 and f2, then low-pass filtered in each I and Q branch
     * (resulting in bandpass functions for f1 and f2).
     * Output power of both filters is compared and delivered as decoded FSK.
     */
    for (n = 0; n < nSamples; n++) {
        /* Table-based computation of the two complex oscillators */
        handle->osc[0] = osc1_cos[handle->osc1];
        handle->osc[1] = osc1_sin[handle->osc1];
        handle->osc1 = (handle->osc1 + 1) % OSC1_N;
        handle->osc[2] = osc0_cos[handle->osc0];
        handle->osc[3] = osc0_sin[handle->osc0];
        handle->osc0 = (handle->osc0 + 1) % OSC0_N;

        sx = rawAudio[n] / 32768.0f;

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

        cookedAudio[n] = sy;
    }

    SRSC_DSP_uart();
    SRSC_DSP_c34();
}


void SRSC_DSP_initAudio (void)
{
    struct _SRSC_Audio *handle = &_srscAudioContext;

    handle->osc0 = 0;
    handle->osc1 = 0;

    handle->osc[0] = handle->osc[1] = handle->osc[2] = handle->osc[3] = 0;
}



void SRSC_handleAudioCallback (int32_t *samples, int nSamples)
{
    struct _SRSC_Audio *handle = &_srscAudioContext;

    SRSC_DSP_processAudio(samples, handle->fskOut, nSamples);
#if (BOARD_RA == 2)
    USBUSER_writeAudioStereo_i32_float(samples, handle->fskOut, nSamples);
#endif
}


/* Discard partial decoder result */
void SRSC_DSP_reset (void)
{
    // Can be done better, but...
    _srscAudioContext.c34.state = 0;
}

