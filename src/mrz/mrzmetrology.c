
#include "lpclib.h"
#include "mrz.h"
#include "mrzprivate.h"



/* Process a frame with meteorological measurements. */
LPCLIB_Result _MRZ_processMetrology (
        MRZ_Packet *packet,
        MRZ_InstanceData *instance)
{
    float f;

    if (!instance) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result = LPCLIB_SUCCESS;

    float temperature = NAN;
    float humidity = NAN;
    float pressure = NAN;
    float vbat = NAN;

    if (_MRZ_checkValidCalibration(instance, CALIB_TEMPERATURE)) {
        /* NTC temperature sensor with 100k bias at 3.3V */
        f = packet->rawTemperature / 100.0f;
        /* Convert to true input voltage [mV] */
        f = roundf((f*f*instance->calib.calibADC_T[0] + f*instance->calib.calibADC_T[1] + instance->calib.calibADC_T[2]) / 10.0f);
        /* Calculate NTC resistance */
        f /= 3300.0f;
        float Rt = 100e3f * f / (1.0f - f);
        /* Convert Rt to temperature */
        /* Use formula from https://github.com/rs1729/RS/blob/master/demod/mod/mp3h1mod.c */
        //TODO what kind of approximation is this? (NTC 10k, guessed B=3230) */
        temperature = instance->calib.calibNTC_B / logf(Rt/instance->calib.calibNTC_A) - instance->calib.calibNTC_C - 273.15f;
    }

    if (_MRZ_checkValidCalibration(instance, CALIB_HUMIDITY) && !isnan(temperature)) {
        /* Relative Humidity */
        f = packet->rawHumidity / 100.0;
        /* Convert to true input voltage [mV] */
        f = (f*f*instance->calib.calibADC_U[0] + f*instance->calib.calibADC_U[1] + instance->calib.calibADC_U[2]) / 10.0f;
        /* Voltage to RH conversion: Formula for "True RH" in HIH-5030 datasheet */
        humidity = (f / 3300.0f - 0.1515f) / (6.707256e-3f - 1.37376e-5 * temperature);
        /* Check for limits */
        humidity = fminf(humidity, 100.0f);
        humidity = fmaxf(humidity, 0);
    }

    if (_MRZ_checkValidCalibration(instance, CALIB_VBAT)) {
        /* 12-bit ADC, 3.3V reference, voltage divider 10k/10k */
        vbat = (instance->calib.rawVbat / 4096.0f) * 3.3f * 2.0f;
    }

    /* Sonde sends -1 if not equipped with pressure sensor */
    if (packet->rawPressure != -1) {
        pressure = packet->rawPressure * 0.2f;
    }

    instance->metro.temperature = temperature;
    instance->metro.humidity = humidity;
    instance->metro.pressure = pressure;
    instance->metro.batteryVoltage = vbat;

    return result;
}


