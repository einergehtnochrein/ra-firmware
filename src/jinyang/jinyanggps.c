
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "jinyang.h"
#include "jinyangprivate.h"



LPCLIB_Result _JINYANG_processGpsFrame (
        JINYANG_SubFrameGps *rawGps,
        JINYANG_InstanceData *instance)
{
    float f, degrees, minutes;
    uint32_t thisTime;

    if (instance == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* We may get the same coordinates twice (with identical time stamp. Ignore that. */
    thisTime = rawGps->time;
    if (thisTime == instance->lastGpsTime) {
        return LPCLIB_ERROR;
    }
    instance->lastGpsTime = thisTime;

    f = rawGps->latitude / 100.0f;
    degrees = floorf(f);
    minutes = f - degrees;
    instance->gps.observerLLA.lat = (degrees + minutes / 0.6f) * (M_PI / 180.0);
    f = rawGps->longitude / 100.0f;
    degrees = floorf(f);
    minutes = f - degrees;
    instance->gps.observerLLA.lon = (degrees + minutes / 0.6f) * (M_PI / 180.0);
    instance->gps.observerLLA.alt = rawGps->altitude;
    instance->gps.observerLLA.velocity = (rawGps->velocity / 100.0f) / 1.944f;  //TODO: assuming "kn"
    instance->gps.observerLLA.direction = (rawGps->direction / 100.0f) * (M_PI / 180.0);
//TODO Check if one of the remaining rawGps fields contains the climb rate
    instance->gps.observerLLA.climbRate = NAN;

    GPS_applyGeoidHeightCorrection(&instance->gps.observerLLA);

instance->gps.unk1 = rawGps->unk1;
instance->gps.unk2 = rawGps->unk2;
instance->gps.unk3 = rawGps->unk3;
instance->gps.unk4 = rawGps->unk4;

    return LPCLIB_SUCCESS;
}


