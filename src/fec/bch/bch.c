
#include <math.h>
#include <string.h>

#include "lpclib.h"
#include "bch.h"
#include "gf64.inc"
#include "gf128.inc"


/* BCH(127,106)
 *   m1(x) = x^7 + x^3 + 1
 *   m3(x) = x^7 + x^3 + x^2 + x + 1
 *   m5(x) = x^7 + x^4 + x^3 + x^2 + 1
 *   g(x) = m1(x) * m3(x) * m5(x)
 *        = x^21 + x^18 + x^17 + x^15 + x^14 + x^12 + x^11 + x^8 + x^7 + x^6 + x^5 + x + 1
 */


#define CODE_n                              (127)
#define CODE_k                              (106)
#define CODE_m                              (CODE_n - CODE_k)


static uint8_t _gfmul64 (uint8_t a, uint8_t b)
{
    int temp;

    if ((a == 0) || (b == 0)) {
        return 0;
    }

    temp = (int)_n2i64[a] + (int)_n2i64[b];
    if (temp >= 63) {
        temp -= 63;
    }

    return _i2n64[temp];
}

static uint8_t _gfadd64 (uint8_t a, uint8_t b)
{
    return a ^ b;
}

static uint8_t _gfinv64 (uint8_t a)
{
    return _i2n64[63 - _n2i64[a]];
}


static uint8_t _gfmul128 (uint8_t a, uint8_t b)
{
    int temp;

    if ((a == 0) || (b == 0)) {
        return 0;
    }

    temp = (int)_n2i128[a] + (int)_n2i128[b];
    if (temp >= 127) {
        temp -= 127;
    }

    return _i2n128[temp];
}

static uint8_t _gfadd128 (uint8_t a, uint8_t b)
{
    return a ^ b;
}

static uint8_t _gfinv128 (uint8_t a)
{
    return _i2n128[127 - _n2i128[a]];
}



uint8_t _bch127_syndroms[6];
uint8_t _bch63_syndroms[4];



static LPCLIB_Result _BCH_127_106_t3_getSyndroms (
        _BCH_127_106_t3_GetDataFunc readAccess,
        uint8_t *syndroms)
{
    int i;
    int n;
    uint32_t sum = 0;
    uint8_t alpha[6];
    volatile uint8_t zeta[6];
    volatile uint8_t syn[6];
    uint8_t bit;

    for (i = 0; i < 2*3; i++) {
        syn[i] = 0;
        alpha[i] = _i2n128[1 + i];
        zeta[i] = 1;
    }
    for (n = 0; n < 127; n++) {
        bit = readAccess(n);
        for (i = 0; i < 2*3; i++) {
            syn[i] = _gfadd128(syn[i], _gfmul128(zeta[i], bit));
            zeta[i] = _gfmul128(alpha[i], zeta[i]);
        }
    }
    for (i = 0; i < 2*3; i++) {
        sum += syn[i];
        syndroms[i] = syn[i];
    }

    /* All syndroms 0 --> valid code word */
    return (sum == 0) ? LPCLIB_SUCCESS : LPCLIB_ERROR;
}



static LPCLIB_Result _BCH_63_51_t2_getSyndroms (
        _BCH_63_51_t2_GetDataFunc readAccess,
        uint8_t *syndroms)
{
    int i;
    int n;
    uint32_t sum = 0;
    uint8_t alpha[4];
    volatile uint8_t zeta[4];
    volatile uint8_t syn[4];
    uint8_t bit;

    for (i = 0; i < 2*2; i++) {
        syn[i] = 0;
        alpha[i] = _i2n64[1 + i];
        zeta[i] = 1;
    }
    for (n = 0; n < 63; n++) {
        bit = readAccess(n);
        for (i = 0; i < 2*2; i++) {
            syn[i] = _gfadd64(syn[i], _gfmul64(zeta[i], bit));
            zeta[i] = _gfmul64(alpha[i], zeta[i]);
        }
    }
    for (i = 0; i < 2*2; i++) {
        sum += syn[i];
        syndroms[i] = syn[i];
    }

    /* All syndroms 0 --> valid code word */
    return (sum == 0) ? LPCLIB_SUCCESS : LPCLIB_ERROR;
}



static LPCLIB_Result _BCH_127_106_t3_decode (const uint8_t *S, uint8_t *pErrorLocations, int *pnErrors)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;
    uint8_t detA = _gfadd128(_gfmul128(S[0],S[1]), S[2]);
    int errorCount = 0;

    if (detA == 0) {
        /* Sanity check: S1 must be non-zero */
        if (S[0] == 0) {
            result = LPCLIB_ERROR;
        }
        else {
            pErrorLocations[0] = _n2i128[S[0]];
            errorCount = 1;
        }
    }
    else {
        uint8_t detAinv = _gfinv128(detA);
        uint8_t x;
        uint8_t Lambda[3];

        x = _gfmul128(S[0],S[2]);
        x = _gfadd128(x, _gfmul128(_gfmul128(S[0],S[0]),S[1]));
        Lambda[0] = _gfmul128(x, detAinv);

        x = _gfmul128(S[0], _gfmul128(S[1],S[1]));
        x = _gfadd128(x, _gfmul128(S[0],S[3]));
        x = _gfadd128(x, _gfmul128(S[1],S[2]));
        x = _gfadd128(x, S[4]);
        Lambda[1] = _gfmul128(x, detAinv);

        x = _gfmul128(_gfmul128(S[0],S[0]),S[3]);
        x = _gfadd128(x, _gfmul128(S[0],_gfmul128(S[1],S[2])));
        x = _gfadd128(x, _gfmul128(S[2],S[2]));
        x = _gfadd128(x, _gfmul128(S[0],S[4]));
        Lambda[2] = _gfmul128(x, detAinv);

        for (int i = 1; i < 127; i++) {
            uint8_t a = _i2n128[i];
            uint8_t y = 1;
            y = _gfadd128(y, _gfmul128(a, Lambda[0]));
            y = _gfadd128(y, _gfmul128(_gfmul128(a,a), Lambda[1]));
            y = _gfadd128(y, _gfmul128(_gfmul128(a,_gfmul128(a,a)), Lambda[2]));
            if (y == 0) {
                pErrorLocations[errorCount] = 127 - i;
                ++errorCount;
                if (errorCount >= 3) {
                    break;
                }
            }
        }
    }

    *pnErrors = errorCount;

    return result;
}



static LPCLIB_Result _BCH_63_51_t2_decode (const uint8_t *S, uint8_t *pErrorLocations, int *pnErrors)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;
    uint8_t detA = S[0];
    int errorCount = 0;

    /* Sanity check: S1 must be non-zero */
    if (S[0] == 0) {
        result = LPCLIB_ERROR;
    }
    else {
        uint8_t detAinv = _gfinv64(detA);
        uint8_t x;
        uint8_t Lambda[2];

        x = _gfmul64(S[0],S[0]);
        Lambda[0] = _gfmul64(x, detAinv);

        x = _gfmul64(S[0],S[1]);
        x = _gfadd64(x, S[2]);
        Lambda[1] = _gfmul64(x, detAinv);

        for (int i = 1; i < 63; i++) {
            uint8_t a = _i2n64[i];
            uint8_t y = 1;
            y = _gfadd64(y, _gfmul64(a, Lambda[0]));
            y = _gfadd64(y, _gfmul64(_gfmul64(a,a), Lambda[1]));
            if (y == 0) {
                pErrorLocations[errorCount] = 63 - i;
                ++errorCount;
                if (errorCount >= 2) {
                    break;
                }
            }
        }
    }

    *pnErrors = errorCount;

    return result;
}



LPCLIB_Result BCH_127_106_t3_process (
        _BCH_127_106_t3_GetDataFunc readAccess,
        _BCH_127_106_t3_ToggleDataFunc writeAccess,
        int *pnErrors)
{
    LPCLIB_Result result;
    int i;

    int nErrors = 0;

    /* Calculate the syndroms */
    result = _BCH_127_106_t3_getSyndroms(readAccess, _bch127_syndroms);
    if (result != LPCLIB_SUCCESS) {
        
static uint8_t Locations[3];

        result = _BCH_127_106_t3_decode(_bch127_syndroms, Locations, &nErrors);
        if (result == LPCLIB_SUCCESS) {
            /* Apply the corrections */
            for (i = 0; i < nErrors; i++) {
                writeAccess(Locations[i]);
            }

            /* Calculate syndroms again to verify the codeword is now correct */
            result = _BCH_127_106_t3_getSyndroms(readAccess, _bch127_syndroms);
            if (result != LPCLIB_SUCCESS) {
            }
        }
    }

    if (pnErrors) {
        *pnErrors = nErrors;
    }

    return result;
}



LPCLIB_Result BCH_63_51_t2_process (
        _BCH_63_51_t2_GetDataFunc readAccess,
        _BCH_63_51_t2_ToggleDataFunc writeAccess,
        int *pnErrors)
{
    LPCLIB_Result result;
    int i;

    int nErrors = 0;

    /* Calculate the syndroms */
    result = _BCH_63_51_t2_getSyndroms(readAccess, _bch63_syndroms);
    if (result != LPCLIB_SUCCESS) {
        
static uint8_t Locations63[2];

        result = _BCH_63_51_t2_decode(_bch63_syndroms, Locations63, &nErrors);
        if (result == LPCLIB_SUCCESS) {
            /* Apply the corrections */
            for (i = 0; i < nErrors; i++) {
                writeAccess(Locations63[i]);
            }

            /* Calculate syndroms again to verify the codeword is now correct */
            result = _BCH_63_51_t2_getSyndroms(readAccess, _bch63_syndroms);
            if (result != LPCLIB_SUCCESS) {
            }
        }
    }

    if (pnErrors) {
        *pnErrors = nErrors;
    }

    return result;
}



