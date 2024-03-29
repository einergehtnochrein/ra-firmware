
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "imet54.h"
#include "imet54private.h"
#include "observer.h"
#include "rinex.h"


/* Find the water vapor saturation pressure for a given temperature.
 * Uses the Hyland and Wexler equation with coefficients for T < 0°C.
 */
static float _IMET54_waterVaporSaturationPressure (float Tcelsius)
{
    /* Convert to Kelvin */
    float T = Tcelsius + 273.15f;

    /* Plug into H+W equation */
    return expf(-5800.2206f / T
                + 1.3914993f
                + 6.5459673f * logf(T)
                - 4.8640239e-2f * T
                + 4.1764768e-5f * T * T
                - 1.4452093e-8f * T * T * T
                );
}


LPCLIB_Result _IMET54_processPayloadMain (
        const IMET54_SubFrameMain *payload,
        IMET54_InstanceData *instance)
{
    LLA_Coordinate lla;
    int32_t degrees;
    double minutes;

    /* GPS */
    degrees = payload->latitude / 1000000;
    minutes = (payload->latitude - 1000000 * degrees) / 10000.0;
    lla.lat = (degrees + minutes / 60.0) * M_PI / 180.0;
    degrees = payload->longitude / 1000000;
    minutes = (payload->longitude - 1000000 * degrees) / 10000.0;
    lla.lon = (degrees + minutes / 60.0) * M_PI / 180.0;
    lla.alt = payload->altitude / 10.0f;
    lla.climbRate = NAN;
    lla.velocity = NAN;
    lla.direction = NAN;

    instance->gps.observerLLA = lla;
    if (_IMET54_checkValidExtra(instance, EXTRA_USEDSATS)) {
        instance->gps.usedSats = payload->usedSats;
    }

    /* payload->pressure is zero if optional MS5607 pressure sensor not installed */
    float pressure = NAN;
    float temperaturePSensor = NAN;
    if (payload->pressure != 0) {
        pressure = payload->pressure / 100.0f;
        temperaturePSensor = payload->temperaturePSensor / 100.0f;
    }
    instance->metro.pressure = pressure;
    instance->metro.temperaturePSensor = temperaturePSensor;

    instance->metro.temperature = payload->temperature;
    instance->metro.temperatureRH = payload->temperatureRH;
    float f = _IMET54_waterVaporSaturationPressure(instance->metro.temperatureRH)
            / _IMET54_waterVaporSaturationPressure(instance->metro.temperature);
    f = payload->humidity * f;
    f = fmaxf(f, 0.0f);
    instance->metro.humidity = fminf(f, 100.0f);

    float temperatureCpu = NAN;
    float temperatureInner = NAN;
    float batteryVoltage = NAN;
    if (_IMET54_checkValidExtra(instance, EXTRA_TEMPCPU)) {
        temperatureCpu = instance->extra.cpuTemperature / 100.0f;
    }
    if (_IMET54_checkValidExtra(instance, EXTRA_TEMPINNER)) {
        temperatureInner = instance->extra.lm75Temperature / 100.0f;
    }
    if (_IMET54_checkValidExtra(instance, EXTRA_VBAT)) {
        batteryVoltage = instance->extra.batteryVoltage / 100.0f;
    }
    instance->metro.temperatureCpu = temperatureCpu;
    instance->metro.temperatureInner = temperatureInner;
    instance->batteryVoltage = batteryVoltage;

    return LPCLIB_SUCCESS;
}

