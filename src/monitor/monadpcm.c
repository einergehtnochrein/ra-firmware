#include <limits.h>
#include <math.h>
#include <stdio.h>

#include "lpclib.h"
#include "sys.h"
#include "monprivate.h"

static const int32_t ADPCM_QTAB[] = {261, INT_MAX};
static const int32_t ADPCM_DQLNTAB[] = {116, 365, 365, 116};
static const int32_t ADPCM_WI[] = {-704, 14048, 14048, -704};
static const int32_t ADPCM_FI[] = {0, 0xE00, 0xE00, 0};

static uint8_t _ADPCM_quantize (struct _adpcm *adpcm, int32_t d, int32_t y)
{
    (void)adpcm;
    int32_t dqm = abs(d);
    int32_t exp = 32 - __CLZ(dqm / 2);
    int32_t mant = ((dqm * 128) >> exp) & 0x7F;
    int32_t dl = (exp << 7) + mant;

    int32_t dln = dl - (y / 4);

    uint8_t I = 0;
    for (int i = 0; i < 2; i++) {
        if (dln < ADPCM_QTAB[i]) {
            I = i;
            break;
        }
    }

    if (d < 0) {
        return 0x3 - I;
    }

    if (d == 0) {
        return 0x3;
    }

    return I;
}


static int16_t _ADPCM_reconstruct (struct _adpcm *adpcm, uint8_t I, int32_t y)
{
    (void)adpcm;
    int sign = (I & 0x2) != 0;

    int32_t dql = ADPCM_DQLNTAB[I] + y / 4;
    if (dql < 0) {
        return sign ? -0x8000 : 0;
    }

    int32_t dex = (dql >> 7) & 0xF;
    int32_t dqt = 128 + (dql & 127);
    int16_t dq = (dqt << 7) >> (14 - dex);

    return sign ? dq - 0x8000 : dq;
}


static void _ADPCM_update (struct _adpcm *adpcm, uint8_t I, int32_t y, int16_t dq, int32_t sr, int32_t dqsez)
{
    int pk0 = dqsez < 0 ? 1 : 0;
    int32_t mag = dq & 0x7FFF;

    int32_t ylint = adpcm->yl >> 15;
    int32_t ylfrac = (adpcm->yl >> 10) & 0x1F;
    int32_t thr1 = (32 + ylfrac) << ylint;
    int32_t thr2 = ylint > 9 ? 31 << 10 : thr1;
    int32_t dqthr = (thr2 + (thr2 >> 1)) >> 1;
    int tr = 0;
    if ((adpcm->td != 0) && (mag > dqthr)) {
        tr = 1;
    }

    adpcm->yu = y + ((ADPCM_WI[I] - y) >> 5);
    if (adpcm->yu < 544) {
        adpcm->yu = 544;
    }
    if (adpcm->yu > 5120) {
        adpcm->yu = 5120;
    }
    adpcm->yl += adpcm->yu + (-adpcm->yl >> 6);

    int32_t a2p = 0;
    if (tr == 1) {
        adpcm->a[0] = adpcm->a[1] = 0;
        adpcm->b[0] = adpcm->b[1] = adpcm->b[2] = adpcm->b[3] = adpcm->b[4] = adpcm->b[5] = 0;
    } else {
        int pks1 = pk0 ^ adpcm->pk[0];
        a2p = adpcm->a[1] - (adpcm->a[1] >> 7);
        if (dqsez != 0) {
            int32_t fa1 = pks1 ? adpcm->a[0] : -adpcm->a[0];
            if (fa1 < -8191) {
                a2p -= 256;
            } else {
                if (fa1 > 8191) {
                    a2p += 255;
                } else {
                    a2p += fa1 >> 5;
                }
            }

            if (pk0 ^ adpcm->pk[1]) {
                if (a2p <= -12160) {
                    a2p = -12288;
                } else {
                    if (a2p >= 12416) {
                        a2p = 12288;
                    } else {
                        a2p -= 128;
                    }
                }
            } else {
                if (a2p <= -12416) {
                    a2p = -12288;
                } else {
                    if (a2p >= 12160) {
                        a2p = 12288;
                    } else {
                        a2p += 128;
                    }
                }
            }
        }
        adpcm->a[1] = a2p;

        adpcm->a[0] -= adpcm->a[0] >> 8;
        if (dqsez != 0) {
            adpcm->a[0] += pks1 ? -192 : 192;
        }
        int32_t a1ul = 15360 - a2p;
        if (adpcm->a[0] < -a1ul) {
            adpcm->a[0] = -a1ul;
        }
        if (adpcm->a[0] > a1ul) {
            adpcm->a[0] = a1ul;
        }

        for (int cnt = 0; cnt < 6; cnt++) {
            adpcm->b[cnt] -= adpcm->b[cnt] >> 8;
            if (dq & 0x7FFF) {
                adpcm->b[cnt] += (dq ^ adpcm->dq[cnt]) >= 0 ? 128 : -128;
            }
        }
    }

    adpcm->dq[5] = adpcm->dq[4];
    adpcm->dq[4] = adpcm->dq[3];
    adpcm->dq[3] = adpcm->dq[2];
    adpcm->dq[2] = adpcm->dq[1];
    adpcm->dq[1] = adpcm->dq[0];
    int16_t dq0 = dq >= 0 ? 0x20 : -992;
    if (mag != 0) {
        int16_t exp = 32 - __CLZ(mag);
        if (dq >= 0) {
            dq0 = (exp << 6) + ((mag << 6) >> exp);
        } else {
            dq0 = (exp << 6) + ((mag << 6) >> exp) - 0x400;
        }
    }
    adpcm->dq[0] = dq0;

    adpcm->sr[1] = adpcm->sr[0];
    if (sr == 0) {
        adpcm->sr[0] = 32;
    } else {
        if (sr > 0) {
            int32_t exp = 32 - __CLZ(sr);
            adpcm->sr[0] = (exp << 6) + ((sr << 6) >> exp);
        } else {
            if (sr > -32768) {
                int32_t exp = 32 - __CLZ(-sr);
                adpcm->sr[0] = (exp << 6) + ((-sr << 6) >> exp) - 0x400;
            } else {
                adpcm->sr[0] = -992;
            }
        }
    }

    adpcm->pk[1] = adpcm->pk[0];
    adpcm->pk[0] = pk0;

    adpcm->td = 0;
    if ((tr == 0) && (a2p < -11776)) {
        adpcm->td = 1;
    }

    adpcm->dms += (ADPCM_FI[I] - adpcm->dms) >> 5;
    adpcm->dml += (ADPCM_FI[I] * 4 - adpcm->dml) >> 7;

    if (tr == 1) {
        adpcm->ap = 256;
    } else {
        if (y < 1536) {
            adpcm->ap += (0x200 - adpcm->ap) >> 4;
        } else {
            if (adpcm->td == 1) {
                adpcm->ap += (0x200 - adpcm->ap) >> 4;
            } else {
                if (abs((adpcm->dms << 2) - adpcm->dml) >= adpcm->dml >> 3) {
                    adpcm->ap += (0x200 - adpcm->ap) >> 4;
                } else {
                    adpcm->ap += -adpcm->ap >> 4;
                }
            }
        }
    }
}


/* Process a single audio sample */
uint8_t ADPCM_processSample (adpcm_t *handle, int32_t sl)
{
    /* G.726 ADPCM encoder 16 kbit/s */
    int32_t sezi = ADPCM_predictor_zero(handle->b, handle->dq);
    int32_t sez = sezi >> 1;
    int32_t se = (sezi + ADPCM_predictor_pole(handle->a, handle->sr)) >> 1;

    int32_t d = sl - se;

    int32_t y = handle->yu;
    if (handle->ap < 256) {
        y = handle->yl >> 6;
        int32_t dif = handle->yu - y;
        int32_t al = handle->ap / 4;
        if (dif > 0) {
            y += (dif * al) >> 6;
        } else {
            y += (dif * al + 63) >> 6;
        }
    }

    uint8_t I = _ADPCM_quantize(handle, d, y);
    if ((I == 3) & (d >= 0)) {
        I = 0;
    }
    int16_t dq = _ADPCM_reconstruct(handle, I, y);

    int32_t sr = dq < 0 ? se - (dq & 0x3FFF) : se + dq;
    int32_t dqsez = sr + sez - se;

    _ADPCM_update(handle, I, y, dq, sr, dqsez);

#if CPPUTEST_ADPCM_TRACE
    handle->y = y;
    handle->d = d;
    handle->_dq = dq;
    handle->sez = sez;
    handle->se = se;
    handle->_sr = sr;
    handle->dqsez = dqsez;
#endif

    return I;
}


/* Initialize ADPCM encoder context */
void ADPCM_init (adpcm_t *handle)
{
    memset(handle, 0, sizeof(adpcm_t));
    handle->yl = 34816;
    handle->yu = 544;
    handle->sr[0] = handle->sr[1] = 32;
    handle->dq[0] = handle->dq[1] = handle->dq[2] = handle->dq[3]
                  = handle->dq[4] = handle->dq[5] = 32;
}
