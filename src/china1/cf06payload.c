
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "cf06.h"
#include "cf06private.h"
#include "observer.h"
#include "rinex.h"



LPCLIB_Result _CF06_processPayloadBlock1 (
        const CF06_PayloadBlock1 *payload,
        CF06_CookedGps *cookedGps,
        CF06_CookedMetrology *cookedMetro)
{
    LLA_Coordinate lla;

    lla.lat = NAN;
    lla.lon = NAN;
    lla.alt = NAN;
    lla.climbRate = NAN;
    lla.velocity = NAN;
    lla.direction = NAN;

    float altitude = payload->altitude / 1000.0f;
    if (altitude > -100.0) {
        lla.alt = altitude;
        lla.lon = payload->longitude * (M_PI * 1e-7f / 180.0f);
        lla.lat = payload->latitude * (M_PI * 1e-7f / 180.0f);
        float ve = payload->speed_east / 100.0f;
        float vn = payload->speed_north / 100.0f;;
        lla.velocity = sqrtf(ve * ve + vn * vn);
        lla.direction = atan2f(ve, vn);
        if (lla.direction <= 0) {
            lla.direction += 2.0f * M_PI;
        }
        GPS_applyGeoidHeightCorrection(&lla);
    }

    cookedGps->observerLLA = lla;

    return LPCLIB_SUCCESS;
}



LPCLIB_Result _CF06_processPayloadBlock2 (
        const CF06_PayloadBlock2 *payload,
        CF06_CookedGps *cookedGps,
        CF06_CookedMetrology *cookedMetro)
{
    return LPCLIB_SUCCESS;
}

