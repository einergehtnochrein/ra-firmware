
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

    /* Sanity check */
    bool ok = true;

    lla.lat = NAN;
    lla.lon = NAN;
    lla.alt = NAN;
    lla.climbRate = NAN;
    lla.velocity = NAN;
    lla.direction = NAN;
    if (ok) {
        lla.alt = _M20_read24_BigEndian(&payload->altitude[0]) / 100.0;
        float ve = payload->speedE * 0.01f;
        float vn = payload->speedN * 0.01f;
        lla.velocity = sqrtf(ve * ve + vn * vn);
        lla.direction = atan2f(ve, vn);
        if (lla.direction <= 0) {
            lla.direction += 2.0f * M_PI;
        }
    }

    if (lla.alt < -100.0) {
        return LPCLIB_ERROR;
    }

    cookedGps->observerLLA = lla;

    /* Sensors */
    cookedMetro->temperature = NAN;
    cookedMetro->humidity = NAN;

    return LPCLIB_SUCCESS;
}



LPCLIB_Result _M20_processPayload (
        const struct _M20_Payload *payload,
        _Bool valid,
        M20_CookedGps *cookedGps)
{
    ECEF_Coordinate ecef;
    LLA_Coordinate lla;

    lla = cookedGps->observerLLA;

    if (valid) {
        lla.lat = (double)payload->latitude * _m20_coordinateFactor;
        lla.lon = (double)payload->longitude * _m20_coordinateFactor;
        lla.climbRate = 0.01f * (float)payload->climbRate;
    }

    GPS_convertLLA2ECEF(&lla, &ecef);

    if (lla.alt < -100.0) {
        return LPCLIB_ERROR;
    }

    cookedGps->observerECEF = ecef;
    cookedGps->observerLLA = lla;

    return LPCLIB_SUCCESS;
}


