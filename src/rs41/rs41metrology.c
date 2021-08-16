
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
        + (1.0f + 4.61e-3f) * T
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
        RS41_CookedMetrology *cookedMetro,
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

    /* If there's no pressure sensor, don't try any caclulation */
    if (current == 0) {
        cookedMetro->pressure = NAN;
        cookedMetro->pressureAltitude = NAN;
    }
    else {
        /* Result is invalid until we have enough calibration data */
        if (!_RS41_checkValidCalibration(instance, CALIB_PRESSURE)) {
            cookedMetro->pressure = NAN;
            cookedMetro->pressureAltitude = NAN;
        }
        else {
            /* Determine coefficients of polynomial */
            float r;
            r = rawMetro->temperaturePressureSensor / 100.0;
            cookedMetro->temperaturePSensor = r;

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
    }

    /* The short metrology block (RS41-SGM in non-encrypted mode) is contained
     * in the regular metrology block.
     */
    result = _RS41_processMetrologyShortBlock(
        (const RS41_SubFrameMetrologyShort *)rawMetro,
        cookedMetro,
        instance);

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
    int i,j;
    float sum;

    for (i = 0; i < 3; i++) {
        refmin = _RS41_read24(rawMetro->adc[i].refmin);
        current[i] = (_RS41_read24(rawMetro->adc[i].current) - refmin)
                   / (_RS41_read24(rawMetro->adc[i].refmax) - refmin);
    }

    /**********  Temperature sensor 1  *********/

    /* Result is invalid until we have enough calibration data */
    if (!_RS41_checkValidCalibration(instance, CALIB_TEMPERATURE)) {
        cookedMetro->T = NAN;
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
        cookedMetro->T = Tuncal + instance->params.polyT[0]
                + instance->params.polyT[1] * Tuncal
                + instance->params.polyT[2] * Tuncal * Tuncal
                + instance->params.polyT[3] * Tuncal * Tuncal * Tuncal
                + instance->params.polyT[4] * Tuncal * Tuncal * Tuncal * Tuncal
                + instance->params.polyT[5] * Tuncal * Tuncal * Tuncal * Tuncal * Tuncal;
    }

    /**********  Temperature sensor 2  *********/

    /* Result is invalid until we have enough calibration data */
    if (!_RS41_checkValidCalibration(instance, CALIB_TEMPERATURE_U)) {
        cookedMetro->temperatureUSensor = NAN;
    }
    else {
        /* Reference values for temperature are two known resistors.
         * From that we can derive the resistance of the sensor.
         */
        float res = instance->params.refResistorLow
            + (instance->params.refResistorHigh - instance->params.refResistorLow) * current[2];
        float x = res * instance->params.calTU;
        cookedMetro->temperatureUSensor = 0
            + instance->params.taylorTU[0]
            + instance->params.taylorTU[1] * x
            + instance->params.taylorTU[2] * x * x
            ;
    }

    /**********  Humidity sensor  *********/

    /* Result is invalid until we have enough calibration data */
    if (!_RS41_checkValidCalibration(instance, CALIB_HUMIDITY)) {
        cookedMetro->RH = NAN;
    }
    else {
        /* We need a valid temperature result for the humidity sensor. */
        float TU = cookedMetro->temperatureUSensor;
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
            cookedMetro->C = instance->params.refCapLow
                    + (instance->params.refCapHigh - instance->params.refCapLow) * current[1];

            /* Apply calibration */
            cookedMetro->Cp = (cookedMetro->C / instance->params.calibU[0] - 1.0f) * instance->params.calibU[1];

            /* Compensation for low temperature and pressure at altitude */
            float powc = 1.0f;
            sum = 0;
            float t = (Trh - 20.0f) / 180.0f;
            float p = cookedMetro->pressure;
            if (isnan(p)) {
                cookedMetro->RHtu = NAN;
                cookedMetro->RH = NAN;
                cookedMetro->dewpoint = NAN;
            }
            else {
                p /= 1000.0f;
                for (i = 0; i < 3; i++) {
                    float l = 0;
                    float powt = 1.0f;
                    for (j = 0; j < 4; j++) {
                        l += instance->params.matrixBt[4*i+j] * powt;
                        powt *= t;
                    }
                    float x = instance->params.vectorBp[i];
                    sum += l * ((x * p / (1.0f + x * p)) - x * powc / (1.0f + x));
                    powc *= cookedMetro->Cp;
                }
                cookedMetro->Cp -= sum;

                int k;
                sum = 0;
                float xj = 1.0f;
                for (j = 0; j < 7; j++) {
                    float yk = 1.0f;
                    for (k = 0; k < 6; k++) {
                        sum += xj * yk * instance->params.matrixU[j][k];
                        yk *= (Trh - 20.0f) / 180.0f;
                    }
                    xj *= cookedMetro->Cp;
                }
                cookedMetro->RHtu = sum;

                /* Since there is always a small difference between the temperature readings for
                * the atmospheric (main) tempoerature sensor and the temperature sensor inside
                * the humidity sensor device, transform the humidity value to the atmospheric conditions
                * with its different water vapor saturation pressure.
                */
                cookedMetro->RH = sum
                    * _RS41_waterVaporSaturationPressure(cookedMetro->temperatureUSensor)
                    / _RS41_waterVaporSaturationPressure(cookedMetro->T);

                /* Dew point */
                float temp = logf(cookedMetro->RH / 100.0f) + (17.625f * cookedMetro->T / (243.04f + cookedMetro->T));
                cookedMetro->dewpoint = 243.04f * temp / (17.625f - temp);
            }
        }
    }

    return LPCLIB_SUCCESS;
}


