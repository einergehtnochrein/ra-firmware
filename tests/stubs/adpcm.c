
#include "monitor/monprivate.h"


/*
 * This stub replaces the assembler implementation part of the ADPCM encoder.
 */


/* C implementation of target ASM functions */


int __CLZ (uint32_t x)
{
    int n = 0;
    for (int i = 0; i < 32; i++) {
        if (x & (1 << (31 - i))) {
            break;
        }
        ++n;
    }

    return n;
}


static int32_t ADPCM_fmult (int32_t an, int32_t srn)
{
    int32_t anmag = (an >= 0 ? an : -an) & 0x1FFF;
    int32_t anexp = 26 - __CLZ(anmag);
    int32_t anmant = anmag == 0 ? 32 : (anexp >= 0 ? anmag >> anexp : anmag << -anexp);
    int32_t wanexp = anexp + ((srn >> 6) & 0xF) - 17;
    int32_t wanmant = anmant * (srn & 0x3F) + 48;
    int32_t retval = wanexp >= 0 ? (wanmant << wanexp) & 0x7FFF : wanmant >> -wanexp;

    return (an ^ srn) < 0 ? -retval : retval;
}


int32_t ADPCM_predictor_zero (int32_t b[6], int16_t dq[6])
{
    int32_t sum = 0;
    for (int i = 0; i < 6; i++) {
        sum += ADPCM_fmult(b[i] >> 2, dq[i]);
    }

    return sum;
}


int32_t ADPCM_predictor_pole (int32_t a[2], int32_t sr[2])
{
    int32_t sum = 0;
    for (int i = 0; i < 2; i++) {
        sum += ADPCM_fmult(a[i] >> 2, sr[i]);
    }

    return sum;
}

