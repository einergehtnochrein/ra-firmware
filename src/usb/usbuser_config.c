/* Copyright (c) 2013, DF9DQ
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



#include "lpclib.h"

#include "app.h"
#include "usbuser_config.h"
#include "usbuser_descriptors.c"
#include "sys.h"


__SECTION(".usbworkspace")
static uint8_t _usbWorkspace[0x800] __ALIGN(1024*1024*4);

#define USBUSER_AUDIO_ENDPOINT_BUFFER_N     3
#define USBUSER_AUDIO_ENDPOINT_BUFFER_SIZE  128
#define USBUSER_AUDIO_RINGBUFFER_SAMPLES_N  1024
#define USBUSER_AUDIO_SAMPLES_NOMINAL_N     16

__SECTION(".usbaudiobuffers")
static int16_t _usbAudioEndpointBuffer[USBUSER_AUDIO_ENDPOINT_BUFFER_N][USBUSER_AUDIO_ENDPOINT_BUFFER_SIZE / sizeof(int16_t)] __ALIGN(64);

static int16_t _usbAudioRingBuffer[USBUSER_AUDIO_RINGBUFFER_SAMPLES_N][2] __ALIGN(4);

struct _USBContext {
    USBD_HANDLE_T hUsb;
    const USBD_API_T *pUsbApi;
    USBD_HANDLE_T hCdc;
    USBD_API_INIT_PARAM_T param;
    USB_CORE_DESCS_T desc;
    bool configured;

    USBAUDIO_Handle audio;

    struct _audiomm {
        int readBufferIndex;
        int readPos;
        int writePos;
        int bufferLevel[USBUSER_AUDIO_ENDPOINT_BUFFER_N];

        TIMER_Handle adapter;
        uint32_t adapterModulus;
        uint32_t dmaTimestamp;
    } audiomm;
} usbContext;


static LPCLIB_Result _USBUSER_handleEventAudio (LPCLIB_Event event);


static USBAUDIO_ControlRange _usbAudioControls[] = {
    { .unit_id          = USBCONFIG_UNIT_FEATURE,
      .control_id       = USBAC_CS_FU_MUTE_CONTROL,
      .min              = 0,
      .max              = 1,
      .current          = 0,
      .resolution       = 1,
      .length           = 1,
      .callback         = NULL,
    },
    { .unit_id          = USBCONFIG_UNIT_FEATURE,
      .control_id       = USBAC_CS_FU_VOLUME_CONTROL,
      .min              = -6 * 0x100,
      .max              = 0,
      .current          = 0,
      .resolution       = 6 * 0x100,
      .length           = 2,
      .callback         = _USBUSER_handleEventAudio,
    },
};

static const USBAUDIO_ControlInterface _usbAudioControlInterface = {
    .interfaceNumber            = USBCONFIG_INTERFACE_AUDIO_CONTROL,
    .usesInterrupt              = false,
    .endpointNumberInterrupt    = 0,
    .numControls                = sizeof(_usbAudioControls) / sizeof(_usbAudioControls[0]),
    .controls                   = _usbAudioControls,
};

const uint32_t _usbAudioSamplingFrequencies[] = {16000ul};

const USBAUDIO_StreamingInterfaceParams _usbAudioStreamingParams[] = {
    { .numEndpoints             = 0,
    },
    { .numEndpoints             = 1,
      .linkedTerminal           = USBCONFIG_UNIT_TERMINAL_OUT,
      .endpointNumber           = USBCONFIG_AUDIO_IN_EP,
      .packetSize               = USBCONFIG_AUDIO_IN_EP_SIZE,
      .supportedControls        = 0x01,     /* Sampling frequency */
      .numSamplingFrequencies   = 1,
      .samplingFrequencies      = _usbAudioSamplingFrequencies,
    },
};

USBAUDIO_StreamingInterface _usbAudioStreamingInterfaces[] = {
    { .interfaceNumber          = USBCONFIG_INTERFACE_AUDIO_STREAM,
      .numAltSettings           = 2,
      .activeSetting            = 0,
      .currentSamplerate        = 1,
      .isMuted                  = false,
      .params                   = _usbAudioStreamingParams,
    },
};

const uint8_t _usbAudioEndpointList[] = {USBCONFIG_AUDIO_IN_EP};

static const USBAUDIO_FunctionDeclaration _usbAudioFunction16k = {
    .controlInterface       = &_usbAudioControlInterface,
    .numStreamingInterfaces = 1,
    .streamingInterfaces    = _usbAudioStreamingInterfaces,
    .callback               = _USBUSER_handleEventAudio,
    .numEndpoints           = sizeof(_usbAudioEndpointList) / sizeof(_usbAudioEndpointList[0]),
    .pEndpointList          = _usbAudioEndpointList,
};



/* Handle events from generic audio class. */
static LPCLIB_Result _USBUSER_handleEventAudio (LPCLIB_Event event)
{
#if 0
    FIFIAUDIO_Message *pMessage;
    FIFIDSP_Config dspConfig;

    if (!fifiaudio.queue) {
        return LPCLIB_SUCCESS;
    }

    switch (event.opcode) {
    case USBAUDIO_EVENT_INTERFACE_CHANGE:
        if (event.block == USBCONFIG_INTERFACE_AUDIO_STREAMING_1) {
            pMessage = osMailAlloc(fifiaudio.queue, 0);
            if (pMessage) {
                pMessage->opcode = FIFIAUDIO_OPCODE_IQ_ENABLE_16;
                if (event.channel == 1) {
                    pMessage->opcode = FIFIAUDIO_OPCODE_IQ_ENABLE_32;
                }
                pMessage->enable = (event.channel != 0);      /* Non-zero alternate settings enable codec */
                osMailPut(fifiaudio.queue, pMessage);
            }
        }
        break;

    case USBAUDIO_EVENT_SET_CONTROL:
        if ((event.block == USBCONFIG_UNIT_FEATURE_IQ) &&
            (event.channel == USBAC_CS_FU_VOLUME_CONTROL)) {
            fifiaudio.volumeIQ = (int32_t)event.parameter;  /* Extract volume (16 bit) from event */
            BSP_setCodecVolume(fifiaudio.volumeIQ, fifiaudio.sampleRateIQ, fifiaudio.sampleSize32);
        }
        break;

    case USBAUDIO_EVENT_ENDPOINT:
        if ((event.block == USBCONFIG_STREAMING_EP1) &&
            (event.channel == USBAC_CS_EP_SAMPLING_FREQ_CONTROL)) {
            pMessage = osMailAlloc(fifiaudio.queue, 0);
            if (pMessage) {
                pMessage->opcode = FIFIAUDIO_OPCODE_SET_IQ_SPEED;
                pMessage->sampleRate = (uint32_t)event.parameter;
                osMailPut(fifiaudio.queue, pMessage);
            }

            dspConfig.opcode = FIFIDSP_OPCODE_SET_INPUT_RATE;
            dspConfig.sampleRate = (uint32_t)event.parameter;
            FIFIDSP_ioctl(fifiaudio.dsp, &dspConfig);
        }
        break;
    }
#else
(void)event;
#endif
    return LPCLIB_SUCCESS;
}


static const TIMER_Config adapterConfig[] = {
    {.opcode = TIMER_OPCODE_MODE,
     {.mode = {.mode = TIMER_MODE_TIMER, }}},

    {.opcode = TIMER_OPCODE_CONFIG_MATCH,
     {.match = {
         .channel = TIMER_MATCH0,
         .intOnMatch = DISABLE,
         .resetOnMatch = ENABLE,
         .stopOnMatch = DISABLE,
         .function = TIMER_MATCH_OUTPUT_NONE,
         .pwm = DISABLE, }}},

    TIMER_CONFIG_END
};


static ErrorCode_t _USBUSER_handleConfigureEvent (USBD_HANDLE_T hUsb)
{
    struct _USBContext *handle = &usbContext;
    USB_CORE_CTRL_T *core = (USB_CORE_CTRL_T *)hUsb;

    handle->configured = core->config_value != 0;

    /* Timer used for audio rate adaptation */
    if (handle->configured) {
        TIMER_open(AUDIO_ADAPTER, &handle->audiomm.adapter);
        TIMER_ioctl(handle->audiomm.adapter, &adapterConfig[0]);
        TIMER_stop(handle->audiomm.adapter);
        TIMER_write(handle->audiomm.adapter, 0);
        handle->audiomm.adapterModulus =
            USBUSER_AUDIO_RINGBUFFER_SAMPLES_N * (CLKPWR_getBusClock(AUDIO_ADAPTER) / 16000);
        TIMER_writeMatch(handle->audiomm.adapter, TIMER_MATCH0, handle->audiomm.adapterModulus - 1);
        TIMER_run(handle->audiomm.adapter);
    }
    else {
        TIMER_close(&handle->audiomm.adapter);
    }

    return LPC_OK;
}


static ErrorCode_t _USBUSER_frameHandler (USBD_HANDLE_T hUsb)
{
    (void)hUsb;
    struct _USBContext *handle = &usbContext;

    USBAUDIO_frameHandler(handle->audio, LPC_USB0->INFO & 0x7FF);

    return LPC_OK;
}



/****** Implementation of USBAUDIO memory model ******/


static void _USBUSER_audioModelInit (void *context)
{
    struct _audiomm *handle = (struct _audiomm *)context;

    handle->readPos = 0;
    handle->writePos = 0;
}


static void _USBUSER_audioModelGetNextBuffer (void *context, uint32_t *address, uint32_t *length)
{
    struct _audiomm *handle = (struct _audiomm *)context;
    int32_t temp;
    uint32_t expectedReadPos;
    int n;
    int i;


    /* Make an estimate of the current write pointer position.
     * The last marker was taken when the write pointer passed the buffer start.
     *
     *                  current_time - marker_time
     * write_position = -------------------------- * buffer_size
     *                           modulus
     *
     * The expected read position should be half a buffer length behind.
     */

    /* Estimated position of write pointer. (Unit: timer ticks) */
    temp = (TIMER_read(handle->adapter) + handle->adapterModulus) - handle->dmaTimestamp;
    if ((uint32_t)temp >= handle->adapterModulus) {
        temp -= handle->adapterModulus;
    }

    /* Scale to samples (0...USBUSER_AUDIO_RINGBUFFER_SAMPLES_N-1) */
    temp = (temp * USBUSER_AUDIO_RINGBUFFER_SAMPLES_N) / handle->adapterModulus;

    /* Deviation of read position from expected read position.
     * Range: +/- USBUSER_AUDIO_RINGBUFFER_SAMPLES_N/2, where negative values indicate that read pointer lags behind.
     */
    expectedReadPos = (temp + USBUSER_AUDIO_RINGBUFFER_SAMPLES_N / 2) % USBUSER_AUDIO_RINGBUFFER_SAMPLES_N;

    temp = ((handle->readPos + (3 * USBUSER_AUDIO_RINGBUFFER_SAMPLES_N) / 2) - expectedReadPos) % USBUSER_AUDIO_RINGBUFFER_SAMPLES_N
         - USBUSER_AUDIO_RINGBUFFER_SAMPLES_N / 2;

    /* Calculate necessary adjustment. */
    n = 16;
    if ((temp < -50) || (temp > +50)) {
        /* Too far off. Force sync, and accept losing samples. */
        handle->readPos = expectedReadPos;
    }
    else if (temp < -5) {
        /* Internal clock too fast. Long packet (+1 sample) */
        n++;
    }
    else if (temp > +5) {
        /* Internal clock too slow. Short packet (-1 sample) */
        n--;
    }

    /* Copy to endpoint buffer */
    for (i = 0; i < n; i++) {
        _usbAudioEndpointBuffer[handle->readBufferIndex][2*i+0] = _usbAudioRingBuffer[handle->readPos][0];
        _usbAudioEndpointBuffer[handle->readBufferIndex][2*i+1] = _usbAudioRingBuffer[handle->readPos][1];

        ++handle->readPos;
        if (handle->readPos >= USBUSER_AUDIO_RINGBUFFER_SAMPLES_N) {
            handle->readPos = 0;
        }
    }

    *address = (uint32_t)&_usbAudioEndpointBuffer[handle->readBufferIndex];
    *length = n * 2 * sizeof(int16_t);

    /* Select next endpoint buffer */
    ++handle->readBufferIndex;
    if (handle->readBufferIndex >= USBUSER_AUDIO_ENDPOINT_BUFFER_N) {
        handle->readBufferIndex = 0;
    }
}


static const USBAUDIO_MemoryModel _usbuserAudioMemModel = {
    .init = _USBUSER_audioModelInit,
    .getNextBuffer = _USBUSER_audioModelGetNextBuffer,
};


void USBUSER_writeAudioStereo_i32_float (const int32_t *buffer1, const float *buffer2, int nSamples)
{
    int n;
    struct _audiomm *handle = &usbContext.audiomm;


    for (n = 0; n < nSamples; n++) {
        _usbAudioRingBuffer[handle->writePos][0] = buffer1[n];
        _usbAudioRingBuffer[handle->writePos][1] = 32768.0f * buffer2[n];

        ++handle->writePos;
        if (handle->writePos >= USBUSER_AUDIO_RINGBUFFER_SAMPLES_N) {
            handle->writePos = 0;
            handle->dmaTimestamp = TIMER_read(handle->adapter);
        }
    }
}


void USBUSER_open (void)
{
    struct _USBContext *handle = &usbContext;
    ErrorCode_t ret = LPC_OK;


    /* Init pointer to ROM API entry */
    handle->pUsbApi = (const USBD_API_T *) pRom->pUsbd;

    handle->param.usb_reg_base = LPC_USB_BASE;
    handle->param.mem_base = (uint32_t)&_usbWorkspace;
    handle->param.mem_size = sizeof(_usbWorkspace);
    handle->param.max_num_ep = USBCONFIG_MAX_NUM_EP;
    handle->param.USB_SOF_Event = _USBUSER_frameHandler;
    handle->param.USB_Configure_Event = _USBUSER_handleConfigureEvent;

    /* Set the USB descriptors */
    handle->desc.device_desc = (uint8_t *) &appDeviceDescriptor;
    handle->desc.string_desc = (uint8_t *) &theUSB_StringDescriptor;

    /* Note, to pass USBCV test full-speed only devices should have both
     * descriptor arrays point to same location and device_qualifier set
     * to 0.
     */
    handle->desc.high_speed_desc = (uint8_t *)&appConfiguration1;
    handle->desc.full_speed_desc = (uint8_t *)&appConfiguration1;
    handle->desc.device_qualifier = 0;

    /* Start ROM stack */
    ret = handle->pUsbApi->hw->Init(&handle->hUsb, &handle->desc, &handle->param);
    if (ret == LPC_OK) {
        ret = USBSerial_init(
                    0,
                    handle->hUsb,
                    &appConfiguration1.serialCIF.interface,
                    &appConfiguration1.serialDIF.interface,
                    &handle->param.mem_base,
                    &handle->param.mem_size);

        if (ret == LPC_OK) {
            ret = USBAUDIO_init(
                        handle->hUsb,
                        &_usbAudioFunction16k,
                        &_usbuserAudioMemModel,
                        &handle->audiomm,
                        &handle->audio);

            if (ret == LPC_OK) {
                /* now connect */
                handle->pUsbApi->hw->Connect(handle->hUsb, 1);
            }
        }
    }
}


bool USBUSER_isConfigured (void)
{
    return usbContext.configured;
}


void USBUSER_worker (void)
{
}


void USB_IRQHandler (void)
{
    /* Call the ROM driver, which may call application callbacks as required. */
    usbContext.pUsbApi->hw->ISR(usbContext.hUsb);
}
