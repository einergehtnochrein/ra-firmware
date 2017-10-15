
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
#include "observer.h"
#include "rinex.h"



LPCLIB_Result _RS41_processGpsPositionBlock (
        const RS41_SubFrameGpsPosition *rawGps,
        RS41_CookedGps *cookedGps)
{
    ECEF_Coordinate ecef;
    LLA_Coordinate lla;

    ecef.x = rawGps->ecefX / 100.0;
    ecef.y = rawGps->ecefY / 100.0;
    ecef.z = rawGps->ecefZ / 100.0;
    ecef.vx = rawGps->speedX / 100.0f;
    ecef.vy = rawGps->speedY / 100.0f;
    ecef.vz = rawGps->speedZ / 100.0f;
    GPS_convertECEF2LLA(&ecef, &lla);

    if (lla.alt < -100.0) {
        return LPCLIB_ERROR;
    }

    cookedGps->observerECEF = ecef;
    cookedGps->observerLLA = lla;

    return LPCLIB_SUCCESS;
}


/* Process the GPS block (sat info). */
LPCLIB_Result _RS41_processGpsInfoBlock (
        const RS41_SubFrameGpsInfo *rawGps,
        RS41_CookedGps *cookedGps)
{
    int i;
    uint8_t prn;
    uint8_t nSats = 0;

    cookedGps->gpstime = rawGps->timeOfWeek / 1000.0;

    for (i = 0; i < 12; i++) {
        prn = rawGps->sats[i].prn;

        /* Count number of valid sats */
        if (prn != (uint8_t)-1) {
            ++nSats;
        }
    }

    cookedGps->visibleSats = nSats;

    return LPCLIB_SUCCESS;
}

