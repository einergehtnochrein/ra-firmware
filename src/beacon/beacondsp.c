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


static struct _BEACON_Audio {
    float filter[SAMPLES_PER_BIT_4];
    int filterIndex;
    float dcTap;

    float corrBuffer[CORR_HISTORY_LENGTH + 256];

    float filterOut[256];
float testOut[256];
float syncDetect[256];

    /* Clock recovery */
    int bitClockPhase;
    int bitClockInputEdgeDetect;
float samplePoints[256];

    uint32_t rxData;
    uint32_t rxCount;
} _beaconAudioContext;


static const float _2pi = 2.0 * M_PI;




uint32_t timeDSP[8];


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
        handle->dcTap -= sx / 128.0f;

        handle->corrBuffer[CORR_HISTORY_LENGTH + n] = sx;

        /* Bit clock recovery */
        handle->bitClockPhase += 1;
        if (handle->bitClockPhase >= SAMPLES_PER_BIT / 2) {
            handle->bitClockPhase = 0;
        }
        if (((handle->bitClockInputEdgeDetect == 0) && (sx > THRESHOLD_HIGH)) ||
            ((handle->bitClockInputEdgeDetect == 1) && (sx < THRESHOLD_LOW))) {
            handle->bitClockInputEdgeDetect = (handle->bitClockInputEdgeDetect == 1) ? 0 : 1;
            if ((handle->bitClockPhase >= 2) && (handle->bitClockPhase <= SAMPLES_PER_BIT / 4)) {
                --handle->bitClockPhase;
                if (handle->bitClockPhase < 0) {
                    handle->bitClockPhase = SAMPLES_PER_BIT / 2 - 1;
                }
            }
            if ((handle->bitClockPhase > SAMPLES_PER_BIT / 4) && (handle->bitClockPhase < SAMPLES_PER_BIT / 2 - 1)) {
                ++handle->bitClockPhase;
                if (handle->bitClockPhase >= SAMPLES_PER_BIT / 2) {
                    handle->bitClockPhase = 0;
                }
            }
        }
//        handle->samplePoints[n] = (handle->bitClockPhase == SAMPLES_PER_BIT / 4) ? 1 : 0;
handle->samplePoints[n] = handle->bitClockPhase / (float)(SAMPLES_PER_BIT/2) - 0.5f;

        cookedAudio[n] = handle->samplePoints[n] ? 0.5f : -0.5f;
    }

    /* Correlator */
    for (n = SAMPLES_PER_BIT - 1; n < SAMPLES_PER_BIT - 1 + 256; n++) {
        /* Calculate mean of pre and post bit time */
        float meanPre = 0;
        for (i = n - (SAMPLES_PER_BIT - 1); i <= n; i++) {
            meanPre += handle->corrBuffer[i];
        }
        meanPre /= SAMPLES_PER_BIT;
        float meanPost = 0;
        for (i = n; i <= n + (SAMPLES_PER_BIT - 1); i++) {
            meanPost += handle->corrBuffer[i];
        }
        meanPost /= SAMPLES_PER_BIT;

        sx = 0;
        for (i = 0; i < SAMPLES_PER_BIT; i++) {
            sx += (handle->corrBuffer[n - (SAMPLES_PER_BIT - 1) + i] - meanPre)
                * (handle->corrBuffer[n + i] - meanPost);
        }

//        cookedAudio[n - (SAMPLES_PER_BIT - 1)] = sx;
handle->testOut[n - (SAMPLES_PER_BIT - 1)] = sx;

        /* Sample data */
handle->syncDetect[n - (SAMPLES_PER_BIT - 1)] = -0.5f;
        if (handle->samplePoints[n - (SAMPLES_PER_BIT - 1)]) {
            int lastNRZ = handle->rxData & 1;
            int thisNRZ = (sx < 0) ? (lastNRZ ^ 1) : lastNRZ;
            handle->rxData = (handle->rxData << 1) | thisNRZ;
            ++handle->rxCount;
if(handle->rxCount>=9){
  if((handle->rxData&0x1FF)==0x02F){
    handle->syncDetect[n - (SAMPLES_PER_BIT - 1)] = 0.5f;
  }
  if((handle->rxData&0x1FF)==0x0D0){
    handle->syncDetect[n - (SAMPLES_PER_BIT - 1)] = 0.5f;
  }
}
        }
    }

    /* Samples that can't be processed now go to history buffer */
    memcpy(handle->corrBuffer,
           &handle->corrBuffer[256],
           CORR_HISTORY_LENGTH * sizeof(float));
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
//    USBUSER_writeAudioStereo_i32_float(samples, handle->filterOut, nSamples);
//    USBUSER_writeAudioStereo_float_float(handle->testOut, handle->filterOut, nSamples);
    USBUSER_writeAudioStereo_float_float(handle->testOut, handle->samplePoints, nSamples);
#endif
}



