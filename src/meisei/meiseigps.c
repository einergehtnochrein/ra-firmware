
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
    int32_t pos32;
    float f;

    if (instance == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (instance->model == MEISEI_MODEL_RS11G) {
        i32 = (_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 1) << 16)
                | _MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 2);
        instance->gps.observerLLA.lat = (i32 / 1e7) * (M_PI / 180.0);
        i32 = (_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 3) << 16)
                | _MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 4);
        instance->gps.observerLLA.lon = (i32 / 1e7) * (M_PI / 180.0);
        i32 = (_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 5) << 16)
                | _MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 6);
        instance->gps.observerLLA.alt = (i32 / 1e2);
        instance->gps.observerLLA.velocity = NAN;
        instance->gps.observerLLA.direction = NAN;
        instance->gps.observerLLA.climbRate = NAN;

        GPS_applyGeoidHeightCorrection(&instance->gps.observerLLA);
        GPS_convertLLA2ECEF(&instance->gps.observerLLA, &instance->gps.observerECEF);
    }
    else if (instance->model == MEISEI_MODEL_IMS100) {
        /* Check if GPS position solution is valid */
        i32 = _MEISEI_getPayloadHalfWord(instance->gpsPacketOdd.fields, 2);
        if ((i32 & (3u << 8)) == (3u << 8)) {
            i32 = (_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 1) << 16)
                    | _MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 2);
            pos32 = (i32 >= 0) ? i32 : -i32;
            f = pos32 / 1000000;
            pos32 %= 1000000;
            f += (pos32 / 1e4f) / 60.0f;
            f = (i32 >= 0) ? f : -f;
            instance->gps.observerLLA.lat = f * (M_PI / 180.0);
            i32 = (_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 3) << 16)
                    | _MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 4);
            pos32 = (i32 >= 0) ? i32 : -i32;
            f = pos32 / 1000000;
            pos32 %= 1000000;
            f += (pos32 / 1e4f) / 60.0f;
            f = (i32 >= 0) ? f : -f;
            instance->gps.observerLLA.lon = f * (M_PI / 180.0);
            i32 = (_MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 5) << 16)
                    | _MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 6);
            instance->gps.observerLLA.alt = ((i32 >> 8) / 1e2);     /* Altitude is a 24-bit field */
            instance->gps.observerLLA.velocity = NAN;
            instance->gps.observerLLA.direction = NAN;
            instance->gps.observerLLA.climbRate = NAN;

            GPS_applyGeoidHeightCorrection(&instance->gps.observerLLA);
            GPS_convertLLA2ECEF(&instance->gps.observerLLA, &instance->gps.observerECEF);
        }
    }
    else  {
        instance->gps.observerLLA.lat = NAN;
        instance->gps.observerLLA.lon = NAN;
        instance->gps.observerLLA.alt = NAN;
        instance->gps.observerLLA.velocity = NAN;
        instance->gps.observerLLA.direction = NAN;
        instance->gps.observerLLA.climbRate = NAN;
    }

    return LPCLIB_SUCCESS;
}


