
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "pilot.h"
#include "pilotprivate.h"
#include "observer.h"
#include "rinex.h"


static const double _pilot_coordinateFactor = (2.0 * M_PI) / (1e6 * 360.0);

LPCLIB_Result _PILOT_processGpsBlock (
        const struct _PILOT_Payload *payload,
        PILOT_CookedGps *cookedGps)
{
    ECEF_Coordinate ecef;
    LLA_Coordinate lla;

    memset(&ecef, 0, sizeof(ecef));
    memset(&lla, 0, sizeof(lla));

    /* Sanity check */
    bool ok = true;
    if (payload->usedSats < 4) ok = false;
    if (payload->latitude >= 90000000) ok = false;
    if (payload->latitude <= -90000000) ok = false;
    if (payload->longitude >= 180000000) ok = false;
    if (payload->longitude <= -180000000) ok = false;
    
    if (!ok) {
        lla.lat = NAN;
        lla.lon = NAN;
        lla.alt = NAN;
        lla.climbRate = NAN;
        lla.velocity = NAN;
        lla.direction = NAN;
    }
    else {
        lla.lat = (double)payload->latitude * _pilot_coordinateFactor;
        lla.lon = (double)payload->longitude * _pilot_coordinateFactor;
        lla.alt = (double)payload->altitude / 100.0;

        float ve = (float)payload->speedEast / 100.0f;
        float vn = (float)payload->speedNorth / 100.0f;
        lla.velocity = sqrtf(ve * ve + vn * vn);
        lla.direction = atan2f(ve, vn);
        if (lla.direction < 0) {
            lla.direction += 2 * M_PI;
        }

        lla.climbRate = (float)payload->climbRate / 100.0f;

        GPS_applyGeoidHeightCorrection(&lla);
        GPS_convertLLA2ECEF(&lla, &ecef);
    }

    cookedGps->observerECEF = ecef;
    cookedGps->observerLLA = lla;
    cookedGps->usedSats = payload->usedSats;
    if (payload->hdop == 9998) {
        cookedGps->hdop = NAN;
    }
    else {
        cookedGps->hdop = (float)payload->hdop / 100.0f;
    }
    if (payload->vdop == 9998) {
        cookedGps->vdop = NAN;
    }
    else {
        cookedGps->vdop = (float)payload->vdop / 100.0f;
    }

    uint8_t n = 0;
    for (int i = 0; i < 12; i++) {
        if (payload->unk36[i] > 0) {
            ++n;
        }
    }
    cookedGps->visibleSats = n;

    return LPCLIB_SUCCESS;
}

