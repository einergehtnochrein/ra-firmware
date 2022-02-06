
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "mrz.h"
#include "mrzprivate.h"



LPCLIB_Result _MRZ_processGpsFrame (
        MRZ_Packet *rawGps,
        MRZ_InstanceData *instance)
{
    uint32_t thisTime;

    if (instance == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* We get the same coordinates multiple times (with identical time stamp. Ignore that. */
    thisTime = rawGps->hour * 3609 + rawGps->minute * 60 + rawGps->second;
    if (thisTime == instance->lastGpsTime) {
        return LPCLIB_ERROR;
    }
    instance->lastGpsTime = thisTime;

    instance->gps.usedSats = rawGps->gps.usedSats;
    instance->gps.observerECEF.x = rawGps->gps.ecef_x / 100.0f;
    instance->gps.observerECEF.y = rawGps->gps.ecef_y / 100.0f;
    instance->gps.observerECEF.z = rawGps->gps.ecef_z / 100.0f;
    instance->gps.observerECEF.vx = rawGps->gps.ecef_vx / 100.0f;
    instance->gps.observerECEF.vy = rawGps->gps.ecef_vy / 100.0f;
    instance->gps.observerECEF.vz = rawGps->gps.ecef_vz / 100.0f;
    GPS_convertECEF2LLA(&instance->gps.observerECEF, &instance->gps.observerLLA);

    GPS_applyGeoidHeightCorrection(&instance->gps.observerLLA);

    /* Invalidate position if this is not a valid position solution */
    if (instance->gps.usedSats == 0) {
        instance->gps.observerLLA.lat = NAN;
        instance->gps.observerLLA.lon = NAN;
        instance->gps.observerLLA.alt = NAN;
    }

    return LPCLIB_SUCCESS;
}


