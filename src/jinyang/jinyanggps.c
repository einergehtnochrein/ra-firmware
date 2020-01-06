
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
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

    if (instance == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

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
    instance->gps.observerLLA.climbRate = NAN;

    GPS_applyGeoidHeightCorrection(&instance->gps.observerLLA);
    GPS_convertLLA2ECEF(&instance->gps.observerLLA, &instance->gps.observerECEF);

    return LPCLIB_SUCCESS;
}


