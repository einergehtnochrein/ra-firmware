
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "meisei.h"
#include "meiseiprivate.h"



LPCLIB_Result _MEISEI_processGpsFrame (
        MEISEI_InstanceData *instance)
{
    int32_t i32;

    if (instance == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    i32 = (_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 1) << 16)
            | _MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 2);
    instance->gps.observerLLA.lat = (i32 / 1e7) * (M_PI / 180.0);
    i32 = (_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 3) << 16)
            | _MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 4);
    instance->gps.observerLLA.lon = (i32 / 1e7) * (M_PI / 180.0);
    i32 = (_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 5) << 16)
            | _MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 6);
    instance->gps.observerLLA.alt = (i32 / 1e2);
    instance->gps.observerLLA.velocity = (int16_t)_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 7) / 100.0f;
    instance->gps.observerLLA.direction = ((int16_t)_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 8) / 100.0f) * M_PI / 180.0f;
    instance->gps.observerLLA.climbRate = (int16_t)_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 9) / 100.0f;

    GPS_convertLLA2ECEF(&instance->gps.observerLLA, &instance->gps.observerECEF);

    return LPCLIB_SUCCESS;
}


