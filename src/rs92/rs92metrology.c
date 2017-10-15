
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "rs92.h"
#include "rs92private.h"




/* Read 24-bit little-endian integer from memory */
static uint32_t _RS92_read24 (const uint8_t *p24)
{
    return p24[0] + 256 * p24[1] + 65536 * p24[2];
}



LPCLIB_Result _RS92_processMetrologyBlock (
        const struct _RS92_MetrologyBlock *rawMetro,
        RS92_CookedMetrology *cookedMetro,
        RS92_InstanceData *instance)
{
    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    int32_t ref1, ref2, ref3, ref4;
    int32_t u1, u2, t, p;
    float tempf;
    float r;
    float w0, w1, w2, w3;

    /* Read all the 24-bit unsigned ADC results */
    ref1 = _RS92_read24(rawMetro->REF1);
    ref2 = _RS92_read24(rawMetro->REF2);
    ref3 = _RS92_read24(rawMetro->REF3);
    ref4 = _RS92_read24(rawMetro->REF4);
    u1 = _RS92_read24(rawMetro->U1);
    u2 = _RS92_read24(rawMetro->U2);
    t = _RS92_read24(rawMetro->T);
    p = _RS92_read24(rawMetro->P);

    /* Calculating cooked metrology requires calibration data. Invalidate the result
     * if not enough calibration data is available.
     */

    /* Temperature */
    if (_RS92_checkValidCalibration(instance, CALIB_TEMPERATURE)) {
        tempf = 1.0f / (instance->coeff[29].value - (float)(ref1 - t) / (float)(ref1 - ref4));
        cookedMetro->temperature =
                        instance->coeff[23].value
                    + instance->coeff[24].value * tempf
                    + instance->coeff[25].value * tempf * tempf
                    + instance->coeff[26].value * tempf * tempf * tempf
                    + instance->coeff[27].value * tempf * tempf * tempf * tempf
                    + instance->coeff[28].value * tempf * tempf * tempf * tempf * tempf
                    ;
    }
    else {
        cookedMetro->temperature = NAN;
    }

    /* Pressure */
    if (_RS92_checkValidCalibration(instance, CALIB_PRESSURE)) {
        /* Aux value */
        tempf = 1.0f / (instance->coeff[11].value - (float)(ref1 - ref2) / (float)(ref1 - ref3));
        r =   instance->coeff[7].value
            + instance->coeff[8].value * tempf
            + instance->coeff[9].value * tempf * tempf
            + instance->coeff[10].value * tempf * tempf * tempf
            ;

        tempf = 1.0f / (instance->coeff[6].value - (float)(ref1 - p) / (float)(ref1 - ref4));

        w0 = 0
            + instance->coeff[0].value
            + instance->coeff[12].value * r
            + instance->coeff[16].value * r * r
            + instance->coeff[20].value * r * r * r
            ;
        w1 = 0
            + instance->coeff[1].value
            + instance->coeff[13].value * r
            + instance->coeff[17].value * r * r
            + instance->coeff[21].value * r * r * r
            ;
        w2 = 0
            + instance->coeff[2].value
            + instance->coeff[14].value * r
            + instance->coeff[18].value * r * r
            + instance->coeff[22].value * r * r * r
            ;
        w3 = 0
            + instance->coeff[3].value
            + instance->coeff[15].value * r
            + instance->coeff[19].value * r * r
            ;
        cookedMetro->pressure =
                  w0
                + w1 * tempf
                + w2 * tempf * tempf
                + w3 * tempf * tempf * tempf
                + instance->coeff[4].value * tempf * tempf * tempf * tempf
                + instance->coeff[5].value * tempf * tempf * tempf * tempf * tempf
                ;

        /* Altitude (from pressure) */
        if (cookedMetro->pressure > 226.32f) {
            cookedMetro->pressureAltitude = 44330.8f * (1.0f - expf(0.190263f * logf(cookedMetro->pressure / 1013.25f)));
        }
        else if (cookedMetro->pressure > 54.749f) {
            cookedMetro->pressureAltitude = 11000.0f - 6341.624f * logf(cookedMetro->pressure / 226.32f);
        }
        else {
            cookedMetro->pressureAltitude = 20000.0f + 216650.0f * (expf(-0.0292173f * logf(cookedMetro->pressure / 54.749f)) - 1.0f);
        }
    }
    else {
        cookedMetro->pressure = NAN;
        cookedMetro->pressureAltitude = NAN;
    }

#if 0
    /* Humidity TODO */
    if (1) {
        tempf = 1.0f / (calib->coeff[34].value - (float)(ref1 - u1) / (float)(ref1 - ref3));
        handle->cooked2.U1 = 0
                        + calib->coeff[30].value
                        + calib->coeff[31].value * tempf
                        + calib->coeff[32].value * tempf * tempf
                        + calib->coeff[33].value * tempf * tempf * tempf
                        ;
        tempf = 1.0f / (calib->coeff[49].value - (float)(ref1 - u2) / (float)(ref1 - ref3));
        handle->cooked2.U2 = 0
                        + calib->coeff[45].value
                        + calib->coeff[46].value * tempf
                        + calib->coeff[47].value * tempf * tempf
                        + calib->coeff[48].value * tempf * tempf * tempf
                        ;
        handle->cooked2.U = (handle->cooked2.U2 > handle->cooked2.U1) ? handle->cooked2.U2 : handle->cooked2.U1;
    }
    else {
    }
#else
(void)u1;
(void)u2;
#endif

    return LPCLIB_SUCCESS;
}

