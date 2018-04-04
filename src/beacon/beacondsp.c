#include <math.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <stdio.h>

#include "lpclib.h"
#include "sys.h"
#include "beaconprivate.h"
#include "bsp.h"
#if (BOARD_RA == 2)
#include "usbuser_config.h"
#endif


#define SAMPLES_PER_BIT         (16000 / 400)
#define SAMPLES_PER_BIT_4       (SAMPLES_PER_BIT / 4)
#define CORR_HISTORY_LENGTH     (2 * (SAMPLES_PER_BIT - 1))

#define THRESHOLD_LOW           (-0.01875f)
#define THRESHOLD_HIGH          (0.01875f)

#define BEACON_NUM_FRAMES       (10)


enum {
    BEACON_DEBUGAUDIO_NORMAL = 0,

    BEACON_DEBUGAUDIO_1,
    BEACON_DEBUGAUDIO_2,
    BEACON_DEBUGAUDIO_3,

    __BEACON_DEBUGAUDIO_MAX__
};


static struct _BEACON_Audio {
    float filter[SAMPLES_PER_BIT * 10];
    int filterIndex;
    float dcTap;

    float corrBuffer[CORR_HISTORY_LENGTH + 256];

    float filterOut[256];
    float testOut[256];

    /* Clock recovery */
    int bitClockPhase;
    int bitClockDelta;
    int lastInput;
    float samplePoints[256];

    uint32_t rxSync;
    uint32_t rxData[4];
    uint32_t rxCount;
    bool rxActive;
    uint8_t emergency;

    int debugAudioChannel;

    int frameWrIndex;
    uint8_t frames[BEACON_NUM_FRAMES][15];
} _beaconAudioContext;



/* Process a batch of 16-kHz audio samples */
void BEACON_DSP_processAudio (const int32_t *rawAudio, float *cookedAudio, int nSamples)
{
    int n;
    int i;
    float sx;
    struct _BEACON_Audio *handle = &_beaconAudioContext;

    /* The function is hard-coded for a 256-sample buffer */
    if (nSamples != 256) {
        return;
    }

    for (n = 0; n < 256; n++) {
        sx = rawAudio[n] / 32768.0f;

        /* Moving Average Filter */
        handle->filter[handle->filterIndex] = sx;
        handle->filterIndex = (handle->filterIndex + 1) % SAMPLES_PER_BIT_4;
        sx = 0;
        for (i = 0; i < SAMPLES_PER_BIT_4; i++) {
            sx += handle->filter[i];
        }
        sx /= SAMPLES_PER_BIT_4;

        /* DC filter */ //TODO
        sx = sx + handle->dcTap;
        handle->dcTap -= sx / 256.0f;

        handle->corrBuffer[CORR_HISTORY_LENGTH + n] = sx;
        cookedAudio[n] = sx;

        /* Edge detect */
        bool haveEdge = false;
        if ((handle->lastInput == 0) && (sx > THRESHOLD_HIGH)) {
            haveEdge = true;
            handle->lastInput = 1;
        }
        if ((handle->lastInput == 1) && (sx < THRESHOLD_LOW)) {
            haveEdge = true;
            handle->lastInput = 0;
        }

        /* Bit clock correction */
        if (haveEdge
            && (handle->bitClockPhase >= SAMPLES_PER_BIT / 4)
            && (handle->bitClockPhase < SAMPLES_PER_BIT / 2 - 1)) {
            handle->bitClockDelta = 1;
        }
        if (haveEdge
            && (handle->bitClockPhase < SAMPLES_PER_BIT / 4)
            && (handle->bitClockPhase > 1)) {
            handle->bitClockDelta = -1;
        }

        /* Bit clock */
        handle->bitClockPhase += 1;
        if (handle->bitClockPhase + handle->bitClockDelta >= SAMPLES_PER_BIT / 2) {
            handle->bitClockPhase = 0;
        }

        /* Sample */
        handle->samplePoints[n] = -0.5f;
        if (handle->bitClockPhase == 7) {  //TODO
            if (handle->rxActive) {
                /* One (half) bit more */
                ++handle->rxCount;

                /* Save every other bit */
                if ((handle->rxCount % 2) == 1) {
                    handle->rxData[3] <<= 1;
                    if (handle->rxData[2] & (1u << 31)) {
                        handle->rxData[3] |= 1;
                    }
                    handle->rxData[2] <<= 1;
                    if (handle->rxData[1] & (1u << 31)) {
                        handle->rxData[2] |= 1;
                    }
                    handle->rxData[1] <<= 1;
                    if (handle->rxData[0] & (1u << 31)) {
                        handle->rxData[1] |= 1;
                    }
                    handle->rxData[0] <<= 1;
                    handle->rxData[0] |= handle->lastInput;
                }

                /* Check if enough bits for a long format message have been collected */
                if (handle->rxCount >= 2*120) {
                    handle->rxActive = false;
                    handle->rxCount = 0;

                    /* Store frame in FIFO */
                    memcpy(&handle->frames[handle->frameWrIndex], handle->rxData, 15);

                    /* Report */
                    {
                        LPCLIB_Event event;
                        LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
                        event.opcode = APP_EVENT_RAW_FRAME;
                        event.block = SONDE_BEACON;
                        event.parameter = &handle->frames[handle->frameWrIndex];
                        event.channel = handle->emergency;
                        SYS_handleEvent(event);
                    }

                    /* Next frame */
                    ++handle->frameWrIndex;
                    if (handle->frameWrIndex >= BEACON_NUM_FRAMES) {
                        handle->frameWrIndex = 0;
                    }
                }
            }
            else {
                /* Store bit in sync word correlator */
                handle->rxSync <<= 1;
                handle->rxSync |= handle->lastInput;

                if ((handle->rxSync & 0x3FFFFF) == 0x2959AA) {
                    handle->samplePoints[n] = 0.5f;
                    handle->rxActive = true;
                    handle->rxCount = 0;
                    handle->emergency = 0;
                }
                if ((handle->rxSync & 0x3FFFFF) == 0x29A655) {
                    handle->samplePoints[n] = 0.5f;
                    handle->rxActive = true;
                    handle->rxCount = 0;
                    handle->emergency = 1;
                }
            }
        }
    }
}


void BEACON_DSP_initAudio (void)
{
    struct _BEACON_Audio *handle = &_beaconAudioContext;

    handle->dcTap = 0;
}



void BEACON_handleAudioCallback (int32_t *samples, int nSamples)
{
    struct _BEACON_Audio *handle = &_beaconAudioContext;

    BEACON_DSP_processAudio(samples, handle->filterOut, nSamples);
#if (BOARD_RA == 2)
    switch (handle->debugAudioChannel) {
        case BEACON_DEBUGAUDIO_1:
            USBUSER_writeAudioStereo_float_float(handle->testOut, handle->filterOut, nSamples);
            break;
        case BEACON_DEBUGAUDIO_2:
            USBUSER_writeAudioStereo_i32_float(samples, handle->filterOut, nSamples);
            break;
        case BEACON_DEBUGAUDIO_3:
            USBUSER_writeAudioStereo_float_float(handle->samplePoints, handle->filterOut, nSamples);
            break;
        default:
            USBUSER_writeAudioStereo_i32_i32(samples, samples, nSamples);
            break;
    }
#endif
}


void BEACON_selectDebugAudio (int debugAudioChannel)
{
    struct _BEACON_Audio *handle = &_beaconAudioContext;

    if (debugAudioChannel < __BEACON_DEBUGAUDIO_MAX__) {
        handle->debugAudioChannel = debugAudioChannel;
    }
}

