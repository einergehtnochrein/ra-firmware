
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


static const double _m10_coordinateFactor = M_PI / 2.147483648e9;


LPCLIB_Result _M10_processGpsBlock (
        const struct _M10_GpsBlock *rawGps,
        M10_CookedGps *cookedGps)
{
    ECEF_Coordinate ecef;
    LLA_Coordinate lla;

    lla.lat = (double)rawGps->latitude * _m10_coordinateFactor;
    lla.lon = (double)rawGps->longitude * _m10_coordinateFactor;
    lla.alt = (double)rawGps->altitude / 1000.0;
    GPS_convertLLA2ECEF(&lla, &ecef);

    if (lla.alt < -100.0) {
        return LPCLIB_ERROR;
    }

    cookedGps->observerECEF = ecef;
    cookedGps->observerLLA = lla;

    return LPCLIB_SUCCESS;
}

