
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "imet.h"
#include "imetprivate.h"



LPCLIB_Result _IMET_processGpsFrame (
        const IMET_FrameGps *rawGps,
        IMET_CookedGps *cookedGps)
{
    if (rawGps->frameType != IMET_FRAME_GPS) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    cookedGps->observerLLA.lat = M_PI * (rawGps->latitude / 180.0f);
    cookedGps->observerLLA.lon = M_PI * (rawGps->longitude / 180.0f);
    cookedGps->observerLLA.alt = rawGps->altitude;
    GPS_convertLLA2ECEF(&cookedGps->observerLLA, &cookedGps->observerECEF);

    cookedGps->usedSats = rawGps->numSats;
    cookedGps->utc.hour = rawGps->hour;
    cookedGps->utc.minute = rawGps->minute;
    cookedGps->utc.second = rawGps->second;

    cookedGps->climbRate = NAN;
    cookedGps->groundSpeed = NAN;
    cookedGps->direction = NAN;

    cookedGps->updated = true;

    return LPCLIB_SUCCESS;
}


LPCLIB_Result _IMET_processGpsxFrame (
        const IMET_FrameGpsx *rawGps,
        IMET_CookedGps *cookedGps)
{
    if (rawGps->frameType != IMET_FRAME_GPSX) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    cookedGps->observerLLA.lat = M_PI * (rawGps->latitude / 180.0f);
    cookedGps->observerLLA.lon = M_PI * (rawGps->longitude / 180.0f);
    cookedGps->observerLLA.alt = rawGps->altitude;
    GPS_convertLLA2ECEF(&cookedGps->observerLLA, &cookedGps->observerECEF);

    cookedGps->usedSats = rawGps->numSats;
    cookedGps->utc.hour = rawGps->hour;
    cookedGps->utc.minute = rawGps->minute;
    cookedGps->utc.second = rawGps->second;

    cookedGps->climbRate = rawGps->verticalVelocity;
    float evel = rawGps->eastVelocity;
    float nvel = rawGps->northVelocity;
    cookedGps->groundSpeed = sqrtf(evel * evel + nvel * nvel);
    cookedGps->direction = atan2f(evel, nvel) * (180.0f / M_PI);

    cookedGps->updated = true;

    return LPCLIB_SUCCESS;
}

