
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "m10.h"
#include "m10private.h"
#include "observer.h"
#include "rinex.h"


static const double _m10_coordinateFactor = M_PI / (double)(1u << 31);


LPCLIB_Result _M10_processGpsBlock (
        const struct _M10_GpsBlock *rawGps,
        M10_CookedGps *cookedGps)
{
    ECEF_Coordinate ecef;
    LLA_Coordinate lla;

    cookedGps->visibleSats = rawGps->visibleSats;

    /* Sanity check */
    bool ok = true;
    if (cookedGps->visibleSats < 4) ok = false;

    if (!ok) {
        lla.lat = NAN;
        lla.lon = NAN;
        lla.alt = NAN;
        lla.velocity = NAN;
        lla.direction = NAN;
        lla.climbRate = NAN;
    }
    else {
        lla.lat = (double)rawGps->latitude * _m10_coordinateFactor;
        lla.lon = (double)rawGps->longitude * _m10_coordinateFactor;
        lla.alt = (double)rawGps->altitude / 1000.0;

        float ve = (float)rawGps->speedEast / 200.0f;
        float vn = (float)rawGps->speedNorth / 200.0f;
        lla.velocity = sqrtf(ve * ve + vn * vn);
        float direction = atan2f(ve, vn);
        if (direction < 0) {
            direction += 2 * M_PI;
        }
        lla.direction = direction;
        lla.climbRate = (float)rawGps->speedVertical / 200.0f;
    }

    GPS_convertLLA2ECEF(&lla, &ecef);

    if (lla.alt < -100.0) {
        return LPCLIB_ERROR;
    }

    cookedGps->observerECEF = ecef;
    cookedGps->observerLLA = lla;

    return LPCLIB_SUCCESS;
}

