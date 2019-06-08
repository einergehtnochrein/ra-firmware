
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "rs41.h"
#include "rs41private.h"




/* Read 24-bit little-endian integer from memory */
static uint32_t _RS41_read24 (const uint8_t *p24)
{
    return p24[0] + 256 * p24[1] + 65536 * p24[2];
}



LPCLIB_Result _RS41_processMetrologyBlock (
        const RS41_SubFrameMetrology *rawMetro,
        RS41_CookedMetrology *cookedMetro,
        RS41_InstanceData *instance)
{
    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result;

    /* The short metrology block (RS41-SGM in non-encrypted mode) is contained
     * in the regular metrology block.
     */
    result = _RS41_processMetrologyShortBlock(
        (const RS41_SubFrameMetrologyShort *)rawMetro,
        cookedMetro,
        instance);
    if (result != LPCLIB_SUCCESS) {
        return result;
    }

    /**********  Pressure sensor  *********/

    float refmin = _RS41_read24(rawMetro->adc[3].refmin);
    float current = (_RS41_read24(rawMetro->adc[3].current) - refmin)
                  / (_RS41_read24(rawMetro->adc[3].refmax) - refmin);

    //TODO Check if there is a pressure sensor!

    /* Result is invalid until we have enough calibration data */
    if (!_RS41_checkValidCalibration(instance, CALIB_PRESSURE)) {
        cookedMetro->pressure = NAN;
        cookedMetro->pressureAltitude = NAN;
    }
    else {
        /* Determine coefficients of polynomial */
        float r;
        r = rawMetro->pressurePolyTwist / 100.0;

        float w[6];
        w[0] = instance->pressurePoly[0]
            + instance->pressurePoly[7]  * r
            + instance->pressurePoly[11] * r * r
            + instance->pressurePoly[15] * r * r * r;
        w[1] = instance->pressurePoly[1]
            + instance->pressurePoly[8]  * r
            + instance->pressurePoly[12] * r * r
            + instance->pressurePoly[16] * r * r * r;
        w[2] = instance->pressurePoly[2]
            + instance->pressurePoly[9]  * r
            + instance->pressurePoly[13] * r * r
            + instance->pressurePoly[17] * r * r * r;  // NOTE: SM uses pressurePoly[13] here again!
        w[3] = instance->pressurePoly[3]
            + instance->pressurePoly[10] * r
            + instance->pressurePoly[14] * r * r;
        w[4] = instance->pressurePoly[4];
        w[5] = instance->pressurePoly[5];

        /* Compute pressure in hPa */
        float x = instance->pressurePoly[6] / current;
        cookedMetro->pressure = 0
            + w[0]
            + w[1] * x
            + w[2] * x * x
            + w[3] * x * x * x
            + w[4] * x * x * x * x
            + w[5] * x * x * x * x * x;

        /* Altitude from pressure */
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

    return result;
}



LPCLIB_Result _RS41_processMetrologyShortBlock (
        const RS41_SubFrameMetrologyShort *rawMetro,
        RS41_CookedMetrology *cookedMetro,
        RS41_InstanceData *instance)
{
    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    float refmin;
    float current[3];
    int i;

    for (i = 0; i < 3; i++) {
        refmin = _RS41_read24(rawMetro->adc[i].refmin);
        current[i] = (_RS41_read24(rawMetro->adc[i].current) - refmin)
                   / (_RS41_read24(rawMetro->adc[i].refmax) - refmin);
    }

    /**********  Temperature sensor 1  *********/

    /* Result is invalid until we have enough calibration data */
    if (!_RS41_checkValidCalibration(instance, CALIB_TEMPERATURE1)) {
        cookedMetro->temperature = NAN;
    }
    else {
        /* Reference values for temperature are two known resistors.
         * From that we can derive the resistance of the sensor.
         */
        float res = instance->refResistorLow
            + (instance->refResistorHigh - instance->refResistorLow) * current[0];

        float x = res * instance->polyT1[3];
        cookedMetro->temperature = 0
            + instance->polyT1[0]
            + instance->polyT1[1] * x
            + instance->polyT1[2] * x * x
            ;
    }

    /**********  Temperature sensor 2  *********/

    /* Result is invalid until we have enough calibration data */
    if (!_RS41_checkValidCalibration(instance, CALIB_TEMPERATURE2)) {
        cookedMetro->temperatureUSensor = NAN;
    }
    else {
        /* Reference values for temperature are two known resistors.
         * From that we can derive the resistance of the sensor.
         */
        float res = instance->refResistorLow
            + (instance->refResistorHigh - instance->refResistorLow) * current[2];

        float x = res * instance->polyT2[3];
        cookedMetro->temperatureUSensor = 0
            + instance->polyT2[0]
            + instance->polyT2[1] * x
            + instance->polyT2[2] * x * x
            ;
    }

    /**********  Humidity sensor  *********/

    /* Calculate the temperature dependency of water vapor saturation
     * according to the "WMO2000" formula.
     * Use a polynomial approximation to directly calculate the correction factor.
     */
    float T = cookedMetro->temperatureUSensor;
    float corr = 1.94589f
            + 7.142e-2f * T
            + 2.62e-3f * T * T
            + 4.7598e-5f * T * T * T
            + 3.1965e-7 * T * T * T * T
            ;
    //TODO
cookedMetro->humidityRAW = current[1];
cookedMetro->humidity = (current[1] - 1.0f) / corr;

    return LPCLIB_SUCCESS;
}

