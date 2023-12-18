#include <limits.h>
#include <math.h>
#include <stdio.h>

#include "lpclib.h"
#include "sys.h"
#include "monprivate.h"

static const float DEC_K00 = 0.07983398f;
static const float DEC_K01 = 0.545166f;
static const float DEC_K10 = 0.2838135f;
static const float DEC_K11 = 0.8342285f;

// Max number of samples to be Base64 encoded
#define N_BASE64_MAX    180
// Smallest multiple of 256 to take longest Base64 string
#define BASE64_BUFSIZE  (((((N_BASE64_MAX + 2) / 3) * 4 + 1 + 255) / 256) * 256)
// Number of Base64 buffers
#define BASE64_NUM_BUF  2


typedef struct MONDSP_Context *MONDSP_Handle;

static struct MONDSP_Context {
    /* Decimator */
    struct _decim {
        int state;
        float A0[3];
        float A1[3];
    } decim;

    /* ADPCM encoder */
    adpcm_t adpcm;
    uint8_t outbuf[256];
    uint32_t outbuf_rd_index;
    uint32_t outbuf_wr_index;
    uint32_t outbuf_bitcnt;

    /* Base64 encoding */
    char b64_buf[BASE64_NUM_BUF][BASE64_BUFSIZE];
    int b64_buf_index;
} _monDspContext;


static void MON_DSP_appendBits (MONDSP_Handle handle, uint8_t I)
{
    for (int i = 0; i < 2; i++) {
        if (I & (1 << (2 - 1 - i))) {
            handle->outbuf[handle->outbuf_wr_index] |= 1 << (7 - handle->outbuf_bitcnt);
        }

        if (++handle->outbuf_bitcnt >= 8) {
            handle->outbuf_wr_index = (handle->outbuf_wr_index + 1) % sizeof(handle->outbuf);
            handle->outbuf_bitcnt = 0;
            handle->outbuf[handle->outbuf_wr_index] = 0;
        }
    }
}


/* Return a string with a available ADPCM output samples, formatted for immediate
 * transmission via BLE and USB.
 * Data is Base64 encoded.
 */
static char * MON_DSP_makeString (MONDSP_Handle handle)
{
    const char * base64encode = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int mod = sizeof(handle->outbuf);
    int N = (mod + handle->outbuf_wr_index - handle->outbuf_rd_index) % mod;
    N = 3 * (N / 3);
    if (N > N_BASE64_MAX) {
        N = N_BASE64_MAX;
    }

    handle->b64_buf[handle->b64_buf_index][0] = 0;
    int n = 0;
    for (int i = 0; i < N; i += 3) {
        uint8_t c0 = handle->outbuf[handle->outbuf_rd_index];
        handle->outbuf_rd_index = (handle->outbuf_rd_index + 1) % mod;
        uint8_t c1 = handle->outbuf[handle->outbuf_rd_index];
        handle->outbuf_rd_index = (handle->outbuf_rd_index + 1) % mod;
        uint8_t c2 = handle->outbuf[handle->outbuf_rd_index];
        handle->outbuf_rd_index = (handle->outbuf_rd_index + 1) % mod;

        uint8_t e0 = (c0 >> 2) & 0x3F;
        uint8_t e1 = ((c0 << 4) | (c1 >> 4)) & 0x3F;
        uint8_t e2 = ((c1 << 2) | (c2 >> 6)) & 0x3F;
        uint8_t e3 = c2 & 0x3F;
        handle->b64_buf[handle->b64_buf_index][n++] = base64encode[e0];
        handle->b64_buf[handle->b64_buf_index][n++] = base64encode[e1];
        handle->b64_buf[handle->b64_buf_index][n++] = base64encode[e2];
        handle->b64_buf[handle->b64_buf_index][n++] = base64encode[e3];
    }
    handle->b64_buf[handle->b64_buf_index][n] = 0;

    // Swap buffers
    handle->b64_buf_index = (handle->b64_buf_index + 1) % BASE64_NUM_BUF;

    return &handle->b64_buf[handle->b64_buf_index][0];
}


/* Process a batch of 16-kHz audio samples */
static void MON_DSP_processAudio (const int32_t *rawAudio, int nSamples)
{
    int n;
    float sx, sy, stemp;
    struct MONDSP_Context *handle = &_monDspContext;


    for (n = 0; n < nSamples; n++) {
        sx = rawAudio[n] / 32768.0f;

        /*
         * Halfband decimator to reduce sample rate down to 8 kHz.
         * This limits audio bandwidth to less than 4 kHz, which is below the highest audio frequency
         * used by C34/C50 AFSK signals. However, those sonde signals are still perfectly
         * recognizable as such.
         */
        if (handle->decim.state == 0) {
            stemp = (sx - handle->decim.A0[1]) * DEC_K00 + handle->decim.A0[0];
            handle->decim.A0[0] = sx;
            sy = (stemp - handle->decim.A0[2]) * DEC_K01 + handle->decim.A0[1];
            handle->decim.A0[1] = stemp;
            handle->decim.A0[2] = sy;

            /* Add last state of A1 branch (this is the z^-1 delay!) */
            sx = (sy + handle->decim.A1[2]) / 2.0f;

            handle->decim.state = 1;
        } else {
            stemp = (sx - handle->decim.A1[1]) * DEC_K10 + handle->decim.A1[0];
            handle->decim.A1[0] = sx;
            sy = (stemp - handle->decim.A1[2]) * DEC_K11 + handle->decim.A1[1];
            handle->decim.A1[1] = stemp;
            handle->decim.A1[2] = sy;

            handle->decim.state = 0;
            continue;   /* Skip this 16 kHz cycle (decimate by 2) */
        }

        /* Scale to 14-bit signed integer (-20%) */
        int32_t sl = (int32_t)(sx * 4096.0f * 0.8f);
        MON_DSP_appendBits(handle, ADPCM_processSample(&handle->adpcm, sl));
    }

    _MON_sendAudio(MON_DSP_makeString(handle));
}


void MON_handleAudioCallback (int32_t *samples, int nSamples)
{
    MON_DSP_processAudio(samples, nSamples);
}


void MON_DSP_reset (void)
{
    struct MONDSP_Context *handle = &_monDspContext;

    memset(&handle->decim, 0, sizeof(handle->decim));
    ADPCM_init(&handle->adpcm);
}
