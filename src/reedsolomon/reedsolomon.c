
#include <math.h>
#include <string.h>

#include "lpclib.h"
#include "reedsolomon.h"
#include "reedsolo.inc"

#define CODE_n                              255
#define CODE_m                              24


/* Polynomial of (max) degree m */
typedef struct {
    uint8_t c[CODE_m + 1];
    uint8_t degree;
} GFPoly;



static uint8_t _gfmul (uint8_t a, uint8_t b)
{
    int temp;

    if ((a == 0) || (b == 0)) {
        return 0;
    }

    temp = (int)_n2i[a] + (int)_n2i[b];
    if (temp >= CODE_n) {
        temp -= CODE_n;
    }

    return _i2n[temp];
}



static uint8_t _gfadd (uint8_t a, uint8_t b)
{
    return a ^ b;
}


/* Calculate the syndroms of the received code word.
 * Return LPCLIB_SUCCESS if a valid code word is detected (all syndroms are zero).
 */
static LPCLIB_Result _REEDSOLOMON_getSyndroms (_REEDSOLOMON_GetDataAddressFunc access, GFPoly *syndroms)
{
    int i, j;
    uint8_t alpha, zeta;
    uint8_t temp;
    uint32_t sum = 0;


    /* Calculate the syndroms */
    syndroms->degree = CODE_m - 1;
    for (i = 0; i < CODE_m; i++) {
        alpha = _i2n[i];
        zeta = alpha;
        temp = *access(0);
        for (j = 1; j < CODE_n; j++) {
            temp = _gfadd(temp, _gfmul(zeta, *access(j)));
            zeta = _gfmul(alpha, zeta);
        }
        syndroms->c[i] = temp;

        /* Add syndroms up to detect any non-zero syndrom */
        sum += syndroms->c[i];
    }

    /* All syndroms 0 --> valid code word */
    return (sum == 0) ? LPCLIB_SUCCESS : LPCLIB_ERROR;
}



/* Process the syndroms by the Berlekamp Massey algorithm.
 * Delivers the error locator polynomial and the error magnitude polynomial.
 * Returns LPCLIB_SUCCESS if the result is consistent (polynomial degrees match)
 */
static LPCLIB_Result _REEDSOLOMON_runBerlekampMassey (GFPoly *syndroms, GFPoly *errorLocatorPoly, GFPoly *errorMagnitudePoly)
{
    int K, L;
    uint8_t C[CODE_m/2 + 1];
    uint8_t e, inve;
    int i, j;
    uint8_t temp;

    /* Run Berlekamp Massey to determine error locator polynomial */
    K = 1;
    L = 0;
    memset(C, 0, sizeof(C));
    C[1] = 1;                               /* C(x) = x */
    memset(errorLocatorPoly, 0, sizeof(GFPoly));
    errorLocatorPoly->c[0] = 1;             /* Lambda(x) = 1 */
    memset(errorMagnitudePoly, 0, sizeof(GFPoly));

    while (K <= CODE_m) {
        /* Calculate e and its inverse */
        e = syndroms->c[K - 1];
        for (i = 1; i <= L; i++) {
            e = _gfadd(e, _gfmul(errorLocatorPoly->c[i], syndroms->c[K - 1 - i]));
        }

        /* Only continue if e is non-zero (invertible) */
        if (e != 0) {
            inve = _i2n[CODE_n - _n2i[e]];

            /* Lambda'(x) = Lambda(x) + e * C(x)
             * if 2L<K:  C(x) = Lambda(x) / e
             * Lambda(x) = Lambda'(x)
             */
            for (i = 0; i < CODE_m/2+1; i++) {
                temp = errorLocatorPoly->c[i];

                errorLocatorPoly->c[i] = _gfadd(temp, _gfmul(C[i], e));
                if (errorLocatorPoly->c[i] != 0) {
                    errorLocatorPoly->degree = i;
                }

                if (2 * L < K) {
                    C[i] = _gfmul(inve, temp);
                }
            }
            if (2 * L < K) {
                L = K - L;
            }
        }

        /* C(x) = C(x) * x */
        for (i = CODE_m/2+1 - 1; i > 0; i--) {
            C[i] = C[i - 1];
        }
        C[0] = 0;

        ++K;
    }

    /* Error magnitude polynomial Omega(x):
     * Omega(x) = (S(x) * Lambda(x)) mod x^m
     */
    for (i = 0; i <= syndroms->degree; i++) {
        for (j = 0; j <= errorLocatorPoly->degree; j++) {
            if (i + j < CODE_m) {
                errorMagnitudePoly->c[i + j] = _gfadd(errorMagnitudePoly->c[i + j], _gfmul(syndroms->c[i], errorLocatorPoly->c[j]));
            }
        }
    }
    for (i = 0; i <= CODE_m; i++) {
        if (errorMagnitudePoly->c[i] != 0) {
            errorMagnitudePoly->degree = i;
        }
    }

    /* The error magnitude polynomial must be of lower degree than the error locator polynomial */
    return (errorMagnitudePoly->degree < errorLocatorPoly->degree) ? LPCLIB_SUCCESS : LPCLIB_ERROR;
}



/* The error locator polynomial's zeroes are at the position of the errors.
 * Find by trying them all...
 * The array errorLocatorInv is filled with the inverse error locators.
 * The array errorPositions is filled with the error positions in the code word.
 * Return LPCLIB_SUCCESS if the expected number of errors is found (degree of error locator polynomial)
 */
static LPCLIB_Result _REEDSOLOMON_findErrorLocators (GFPoly *errorLocatorPoly, uint8_t *errorLocatorsInv, uint8_t *errorPositions, int *nErrors)
{
    int i, j;
    uint8_t alpha, zeta;
    uint8_t e;

    /* Sanity check: error locator polynomial must be of degree m/2 or less */
    if (errorLocatorPoly->degree > CODE_m / 2) {
        return 0;
    }

    *nErrors = 0;
    for (i = 0; i < CODE_n; i++) {
        alpha = _i2n[i + 1];
        zeta = alpha;

        e = errorLocatorPoly->c[0];
        for (j = 1; j <= errorLocatorPoly->degree; j++) {
            e = _gfadd(e, _gfmul(zeta, errorLocatorPoly->c[j]));
            zeta = _gfmul(alpha, zeta);
        }

        if (e == 0) {
            errorLocatorsInv[*nErrors] = alpha;                 // Xj^-1
            errorPositions[*nErrors] = CODE_n - _n2i[alpha];    // arg(Xj)
            ++(*nErrors);
        }
    }

    return (*nErrors == errorLocatorPoly->degree) ? LPCLIB_SUCCESS : LPCLIB_ERROR;
}



/* Calculate the derivative of a polynomial in GF(256).
 */
static void _REEDSOLOMON_derivePoly (GFPoly *poly, GFPoly *derived)
{
    int i;

    derived->c[0] = 0;
    for (i = 1; i <= poly->degree; i++) {
        if (i % 2) {
            derived->c[i - 1] = poly->c[i];
        }
        else {
            derived->c[i - 1] = 0;
        }
    }

    derived->degree = poly->degree ? poly->degree - 1 : 0;
}



/* Evaluate a polynomial for a given value */
static uint8_t _REEDSOLOMON_polyVal (GFPoly *poly, uint8_t val)
{
    int i;
    uint8_t sum = poly->c[0];
    uint8_t x = val;

    for (i = 1; i <= poly->degree; i++) {
        sum = _gfadd(sum, _gfmul(poly->c[i], x));
        x = _gfmul(x, val);
    }

    return sum;
}



#if 1
int nErrors;
GFPoly syndroms;
GFPoly errorLocatorPoly;
GFPoly errorLocatorPolyDerived;
GFPoly errorMagnitudePoly;
uint8_t errorLocatorsInv[CODE_m / 2];
uint8_t errorPositions[CODE_m / 2];
#endif



LPCLIB_Result REEDSOLOMON_process (_REEDSOLOMON_GetDataAddressFunc access, int *pnErrors)
{
    LPCLIB_Result result;
    int i;
    int pos;
    uint8_t y;
    uint8_t xjinv;
    uint8_t w;
    uint8_t ld;


    nErrors = 0;

    /* Calculate the syndroms */
    result = _REEDSOLOMON_getSyndroms(access, &syndroms);
    if (result != LPCLIB_SUCCESS) {
        /* Find error locator and error magnitude polynomials */
        result = _REEDSOLOMON_runBerlekampMassey(&syndroms, &errorLocatorPoly, &errorMagnitudePoly);
        if (result == LPCLIB_SUCCESS) {
            /* Determine error locators (and positions, which is the log of their inverse) */
            result = _REEDSOLOMON_findErrorLocators(&errorLocatorPoly, errorLocatorsInv, errorPositions, &nErrors);
            if (result == LPCLIB_SUCCESS) {
                _REEDSOLOMON_derivePoly(&errorLocatorPoly, &errorLocatorPolyDerived);
                for (i = 0; i < nErrors; i++) {
                    pos = errorPositions[i];
                    xjinv = errorLocatorsInv[i];

                    w = _REEDSOLOMON_polyVal(&errorMagnitudePoly, xjinv);
                    ld = _REEDSOLOMON_polyVal(&errorLocatorPolyDerived, xjinv);
                    y = _gfmul(w, _i2n[CODE_n - _n2i[ld]]);
                    y = _gfmul(y, _i2n[CODE_n - _n2i[xjinv]]);
                    *access(pos) ^= y;
                }

                /* Calculate the syndroms again after correction */
                //TODO: Necessary?
                result = _REEDSOLOMON_getSyndroms(access, &syndroms);
            }
        }
    }

    if (pnErrors) {
        *pnErrors = nErrors;
    }

    return result;
}



