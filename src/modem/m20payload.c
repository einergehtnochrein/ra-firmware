
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "m20.h"
#include "m20private.h"
#include "observer.h"
#include "rinex.h"


static const double _m20_coordinateFactor = M_PI / (180.0 * 1e6);


/* Read 24-bit little-endian signed integer from memory */
static int32_t _M20_read24_BigEndian (const uint8_t *p24)
{
    int32_t value = p24[2] + 256 * p24[1] + 65536 * p24[0];
    if (value & (1u << 23)) {
        value |= 0xFF000000;
    }

    return value;
}



LPCLIB_Result _M20_processPayloadInner (
        const struct _M20_PayloadInner *payload,
        M20_CookedGps *cookedGps,
        M20_CookedMetrology *cookedMetro)
{
    LLA_Coordinate lla;

    lla.lat = NAN;
    lla.lon = NAN;
    lla.alt = NAN;
    lla.climbRate = NAN;
    lla.velocity = NAN;
    lla.direction = NAN;

    float altitude = _M20_read24_BigEndian(&payload->altitude[0]) / 100.0f;
    if (altitude > -100.0) {
        lla.alt = altitude;
        float ve = payload->speedE * 0.01f;
        float vn = payload->speedN * 0.01f;
        lla.velocity = sqrtf(ve * ve + vn * vn);
        lla.direction = atan2f(ve, vn);
        if (lla.direction <= 0) {
            lla.direction += 2.0f * M_PI;
        }
        GPS_applyGeoidHeightCorrection(&lla);
    }

    cookedGps->observerLLA = lla;

    /* Sensors */
    cookedMetro->temperature = NAN;
    cookedMetro->humidity = NAN;

    // Humidity
    uint16_t humval = payload->humidity;
    if (!isnan(cookedMetro->temperature)) {
        float T = cookedMetro->temperature - 25.0f;
        if ((humval > 0) && (humval < 48000) && !isnan(cookedMetro->humidityCalibration)) {
            float x = (humval + 80000.0f) * cookedMetro->humidityCalibration * (1.0f - 5.8e-4f * T);
            x = 4.16e9f / x;
            x = 10.087f*x*x*x - 211.62f*x*x + 1388.2f*x - 2797.0f;
            x = fmaxf(x, 0.0f);
            x = fminf(x, 100.0f);
            cookedMetro->humidity = x;
        }
    }

    return LPCLIB_SUCCESS;
}



LPCLIB_Result _M20_processPayload (
        const struct _M20_Payload *payload,
        _Bool valid,
        M20_CookedGps *cookedGps,
        M20_CookedMetrology *cookedMetro)
{
    LLA_Coordinate lla;

    lla = cookedGps->observerLLA;

    if (valid) {
        lla.lat = (double)payload->latitude * _m20_coordinateFactor;
        lla.lon = (double)payload->longitude * _m20_coordinateFactor;
        lla.climbRate = 0.01f * (float)payload->climbRate;

        cookedMetro->pressure = payload->pressure / 16.0f;
        cookedMetro->batteryVoltage = payload->vbat * (3.0f / 228.0f);  //TODO use ADC VDD and resistor divider
        cookedMetro->cpuTemperature = payload->cpuTemperature * 0.4f;
        cookedMetro->humidityCalibration = 6.4e8f / (payload->humidityCalibration + 80000.0f);
        cookedMetro->adc_pb1 = payload->adc_pb1_pc3[0] + (256 * (payload->adc_pb1_pc3[1] % 16));
        cookedMetro->adc_pc3 = (payload->adc_pb1_pc3[1] / 16) + (16 * payload->adc_pb1_pc3[2]);
        cookedMetro->adc_pc0 = payload->adc_pc0_pc1[0] + (256 * (payload->adc_pc0_pc1[1] % 16));
        cookedMetro->adc_pc1 = (payload->adc_pc0_pc1[1] / 16) + (16 * payload->adc_pc0_pc1[2]);
        cookedMetro->adc_pc2 = payload->adc_pc2;

        cookedGps->sats[0] = payload->satStatus[0] & 7;
        cookedGps->sats[1] = (payload->satStatus[0] >> 3) & 7;
        cookedGps->sats[2] = ((payload->satStatus[0] >> 6) & 3) | ((payload->satStatus[1] & 1) << 2);
        cookedGps->sats[3] = (payload->satStatus[1] >> 1) & 7;
        cookedGps->sats[4] = (payload->satStatus[1] >> 4) & 7;
        cookedGps->sats[5] = ((payload->satStatus[1] >> 7) & 1) | ((payload->satStatus[2] & 3) << 1);
        cookedGps->sats[6] = (payload->satStatus[2] >> 2) & 7;
        cookedGps->sats[7] = (payload->satStatus[2] >> 5) & 7;
        cookedGps->sats[8] = payload->satStatus[3] & 7;
        cookedGps->sats[9] = (payload->satStatus[3] >> 3) & 7;
        cookedGps->sats[10] = ((payload->satStatus[3] >> 6) & 3) | ((payload->satStatus[4] & 1) << 2);
        cookedGps->sats[11] = (payload->satStatus[4] >> 1) & 7;
        cookedGps->sats[12] = (payload->satStatus[4] >> 4) & 7;
    }

    cookedGps->observerLLA = lla;

    return LPCLIB_SUCCESS;
}


