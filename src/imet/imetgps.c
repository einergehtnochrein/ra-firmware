
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
    /* Altitude is transmitted with a +5000 m offset.
     * See Wendell, Jordan: "iMet-1-RSB Radiosonde Protocol, Version 1.11, August 12, 2009"
     */
    cookedGps->observerLLA.alt = rawGps->altitudePlus5000 - 5000;
    cookedGps->observerLLA.climbRate = NAN;
    cookedGps->observerLLA.velocity = NAN;
    cookedGps->observerLLA.direction = NAN;
    GPS_convertLLA2ECEF(&cookedGps->observerLLA, &cookedGps->observerECEF);

    cookedGps->usedSats = rawGps->numSats;
    cookedGps->utc.hour = rawGps->hour;
    cookedGps->utc.minute = rawGps->minute;
    cookedGps->utc.second = rawGps->second;

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
    /* Altitude is transmitted with a +5000 m offset.
     * See Wendell, Jordan: "iMet-1-RSB Radiosonde Protocol, Version 1.11, August 12, 2009"
     */
    cookedGps->observerLLA.alt = rawGps->altitudePlus5000 - 5000;
    cookedGps->observerLLA.climbRate = rawGps->verticalVelocity;
    float ve = rawGps->eastVelocity;
    float vn = rawGps->northVelocity;
    cookedGps->observerLLA.velocity = sqrtf(ve * ve + vn * vn);
    cookedGps->observerLLA.direction = atan2f(ve, vn);
    if (cookedGps->observerLLA.direction <= 0) {
        cookedGps->observerLLA.direction += 2.0f * M_PI;
    }
    GPS_convertLLA2ECEF(&cookedGps->observerLLA, &cookedGps->observerECEF);

    cookedGps->usedSats = rawGps->numSats;
    cookedGps->utc.hour = rawGps->hour;
    cookedGps->utc.minute = rawGps->minute;
    cookedGps->utc.second = rawGps->second;


    cookedGps->updated = true;

    return LPCLIB_SUCCESS;
}

