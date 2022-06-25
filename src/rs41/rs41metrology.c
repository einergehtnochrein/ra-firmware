
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


/* Find the water vapor saturation pressure for a given temperature.
 * Uses the Hyland and Wexler equation with coefficients for T < 0°C.
 */
static float _RS41_waterVaporSaturationPressure (float Tcelsius)
{
    /* Convert to Kelvin */
    float T = Tcelsius + 273.15f;

    /* Apply some correction magic */
    T = 0
        - 0.4931358f
        + (1.0f + 4.6094296e-3f) * T
        - 1.3746454e-5f * T * T
        + 1.2743214e-8f * T * T * T
        ;

    /* Plug into H+W equation */
    float p = expf(-5800.2206f / T
                  + 1.3914993f
                  + 6.5459673f * logf(T)
                  - 4.8640239e-2f * T
                  + 4.1764768e-5f * T * T
                  - 1.4452093e-8f * T * T * T
                  );

    /* Scale result to hPa */
    return p / 100.0f;
}


LPCLIB_Result _RS41_processMetrologyBlock (
        const RS41_SubFrameMetrology *rawMetro,
        RS41_InstanceData *instance)
{
    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result;

    /**********  Pressure sensor  *********/

    float refmin = _RS41_read24(rawMetro->adc[3].refmin);
    float current = (_RS41_read24(rawMetro->adc[3].current) - refmin)
                  / (_RS41_read24(rawMetro->adc[3].refmax) - refmin);

    /* If there's no pressure sensor, don't try any calculation */
    if (current == 0) {
        instance->metro.pressure = NAN;
        instance->metro.pressureAltitude = NAN;
    }
    else {
        /* Result is invalid until we have enough calibration data */
        if (!_RS41_checkValidCalibration(instance, CALIB_PRESSURE)) {
            instance->metro.pressure = NAN;
            instance->metro.pressureAltitude = NAN;
        }
        else {
            /* Determine coefficients of polynomial */
            float r;
            r = rawMetro->temperaturePressureSensor / 100.0;
            instance->metro.temperaturePSensor = r;

            float w[6];
            w[0] = instance->params.matrixP[0]
                + instance->params.matrixP[7]  * r
                + instance->params.matrixP[11] * r * r
                + instance->params.matrixP[15] * r * r * r;
            w[1] = instance->params.matrixP[1]
                + instance->params.matrixP[8]  * r
                + instance->params.matrixP[12] * r * r
                + instance->params.matrixP[16] * r * r * r;
            w[2] = instance->params.matrixP[2]
                + instance->params.matrixP[9]  * r
                + instance->params.matrixP[13] * r * r
                + instance->params.matrixP[17] * r * r * r;    // NOTE: SM uses matrixP[13] here again!
            w[3] = instance->params.matrixP[3]
                + instance->params.matrixP[10] * r
                + instance->params.matrixP[14] * r * r;
            w[4] = instance->params.matrixP[4];
            w[5] = instance->params.matrixP[5];

            /* Compute pressure in hPa */
            float x = instance->params.matrixP[6] / current;
            instance->metro.pressure = 0
                + w[0]
                + w[1] * x
                + w[2] * x * x
                + w[3] * x * x * x
                + w[4] * x * x * x * x
                + w[5] * x * x * x * x * x;

            /* Altitude from pressure */
            if (instance->metro.pressure > 226.32f) {
                instance->metro.pressureAltitude = 44330.8f * (1.0f - expf(0.190263f * logf(instance->metro.pressure / 1013.25f)));
            }
            else if (instance->metro.pressure > 54.749f) {
                instance->metro.pressureAltitude = 11000.0f - 6341.624f * logf(instance->metro.pressure / 226.32f);
            }
            else {
                instance->metro.pressureAltitude = 20000.0f + 216650.0f * (expf(-0.0292173f * logf(instance->metro.pressure / 54.749f)) - 1.0f);
            }
        }
    }

    /* The short metrology block (RS41-SGM in non-encrypted mode) is contained
     * in the regular metrology block.
     */
    result = _RS41_processMetrologyShortBlock(
        (const RS41_SubFrameMetrologyShort *)rawMetro,
        instance);

    return result;
}



LPCLIB_Result _RS41_processMetrologyShortBlock (
        const RS41_SubFrameMetrologyShort *rawMetro,
        RS41_InstanceData *instance)
{
    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    float refmin;
    float current[3];
    int i,j;
    float sum;

    if (instance->is_SGM) {
        /* There is no pressure sensor in the military version */
        instance->metro.pressure = NAN;
        instance->metro.temperaturePSensor = NAN;
    }

    for (i = 0; i < 3; i++) {
        refmin = _RS41_read24(rawMetro->adc[i].refmin);
        current[i] = (_RS41_read24(rawMetro->adc[i].current) - refmin)
                   / (_RS41_read24(rawMetro->adc[i].refmax) - refmin);
    }

    /**********  Temperature sensor 1  *********/

    /* Result is invalid until we have enough calibration data */
    if (!_RS41_checkValidCalibration(instance, CALIB_TEMPERATURE)) {
        instance->metro.T = NAN;
    }
    else {
        /* Reference values for temperature are two known resistors.
         * From that we can derive the resistance of the sensor.
         */
        float res = instance->params.refResistorLow
            + (instance->params.refResistorHigh - instance->params.refResistorLow) * current[0];
        float x = res * instance->params.calT;
        float Tuncal = 0
            + instance->params.taylorT[0]
            + instance->params.taylorT[1] * x
            + instance->params.taylorT[2] * x * x
            ;

        /* Apply calibration polynomial */
        instance->metro.T = Tuncal + instance->params.polyT[0]
                + instance->params.polyT[1] * Tuncal
                + instance->params.polyT[2] * Tuncal * Tuncal
                + instance->params.polyT[3] * Tuncal * Tuncal * Tuncal
                + instance->params.polyT[4] * Tuncal * Tuncal * Tuncal * Tuncal
                + instance->params.polyT[5] * Tuncal * Tuncal * Tuncal * Tuncal * Tuncal;
    }

    /**********  Temperature sensor 2  *********/

    /* Result is invalid until we have enough calibration data */
    if (!_RS41_checkValidCalibration(instance, CALIB_TEMPERATURE_U)) {
        instance->metro.TU = NAN;
    }
    else {
        /* Reference values for temperature are two known resistors.
         * From that we can derive the resistance of the sensor.
         */
        float res = instance->params.refResistorLow
            + (instance->params.refResistorHigh - instance->params.refResistorLow) * current[2];
        float x = res * instance->params.calTU;
        instance->metro.TU = 0
            + instance->params.taylorTU[0]
            + instance->params.taylorTU[1] * x
            + instance->params.taylorTU[2] * x * x
            ;
    }

    /**********  Humidity sensor  *********/

    /* Result is invalid until we have enough calibration data */
    if (!_RS41_checkValidCalibration(instance, CALIB_HUMIDITY)) {
        instance->metro.RH = NAN;
    }
    else {
        /* We need a valid temperature result for the humidity sensor. */
        float TU = instance->metro.TU;
        if (!isnan(TU)) {
            /* Temperature compensation of the humidity sensor uses a slightly modified
             * temperature value.
             */
            float Trh = TU
                    + instance->params.polyTrh[0]
                    + instance->params.polyTrh[1] * TU
                    + instance->params.polyTrh[2] * TU * TU
                    + instance->params.polyTrh[3] * TU * TU * TU
                    + instance->params.polyTrh[4] * TU * TU * TU * TU
                    + instance->params.polyTrh[5] * TU * TU * TU * TU * TU;

            /* Compute absolute capacitance from the known references */
            instance->metro.C = instance->params.refCapLow
                    + (instance->params.refCapHigh - instance->params.refCapLow) * current[1];

            /* Apply calibration */
            instance->metro.Cp = (instance->metro.C / instance->params.calibU[0] - 1.0f) * instance->params.calibU[1];

            /* Get pressure (either from sensor or an estimate */
            float p = instance->metro.pressure;
            if (isnan(p)) {
                /* This might be an RS41 variant without pressure sensor.
                 * Use pressure calculated from known altitude.
                 */
                p = instance->gps.estimatedPressure;
                if (isnan(p) || (p == 0)) {
                    /* Unknown pressure. Just assume 30 hPa. Not too bad an assumption for our purpose... */
                    p = 30.0f;
                }
            }

            /* Compensation for low temperature and pressure at altitude */
            float Tp = (Trh - 20.0f) / 180.0f;
            sum = 0;
            float powc = 1.0f;
            p /= 1000.0f;
            for (i = 0; i < 3; i++) {
                float l = 0;
                float powt = 1.0f;
                for (j = 0; j < 4; j++) {
                    l += instance->params.matrixBt[4*i+j] * powt;
                    powt *= Tp;
                }
                float x = instance->params.vectorBp[i];
                sum += l * (x * p / (1.0f + x * p) - x * powc / (1.0f + x));
                powc *= instance->metro.Cp;
            }
            instance->metro.Cp -= sum;

            int k;
            sum = 0;
            float xj = 1.0f;
            for (j = 0; j < 7; j++) {
                float yk = 1.0f;
                for (k = 0; k < 6; k++) {
                    sum += xj * yk * instance->params.matrixU[j][k];
                    yk *= Tp;
                }
                xj *= instance->metro.Cp;
            }
            instance->metro.RHtu = sum;

            /* Since there is always a small difference between the temperature readings for
            * the atmospheric (main) tempoerature sensor and the temperature sensor inside
            * the humidity sensor device, transform the humidity value to the atmospheric conditions
            * with its different water vapor saturation pressure.
            */
            instance->metro.RH = sum
                * _RS41_waterVaporSaturationPressure(instance->metro.TU)
                / _RS41_waterVaporSaturationPressure(instance->metro.T);

            /* Dew point */
            float temp = logf(instance->metro.RH / 100.0f) + (17.625f * instance->metro.T / (243.04f + instance->metro.T));
            instance->metro.dewpoint = 243.04f * temp / (17.625f - temp);
        }
    }

    return LPCLIB_SUCCESS;
}


