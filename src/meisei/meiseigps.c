
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
        const MEISEI_Packet *packet,
        MEISEI_InstanceData *instance)
{
    int32_t i32;

    if (instance == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Ignore frame with no valid GPS data */
    if (_MEISEI_getPayloadHalfWord(packet->fields, 0) == 0) {
        return LPCLIB_PENDING;
    }

    i32 = (_MEISEI_getPayloadHalfWord(packet->fields, 1) << 16) | _MEISEI_getPayloadHalfWord(packet->fields, 2);
    instance->gps.observerLLA.lat = (i32 / 1e7) * (M_PI / 180.0);
    i32 = (_MEISEI_getPayloadHalfWord(packet->fields, 3) << 16) | _MEISEI_getPayloadHalfWord(packet->fields, 4);
    instance->gps.observerLLA.lon = (i32 / 1e7) * (M_PI / 180.0);
    i32 = (_MEISEI_getPayloadHalfWord(packet->fields, 5) << 16) | _MEISEI_getPayloadHalfWord(packet->fields, 6);
    instance->gps.observerLLA.alt = (i32 / 1e2);
    instance->gps.observerLLA.velocity = (int16_t)_MEISEI_getPayloadHalfWord(packet->fields, 7) / 100.0f;
    instance->gps.observerLLA.direction = ((int16_t)_MEISEI_getPayloadHalfWord(packet->fields, 8) / 100.0f) * M_PI / 180.0f;
    instance->gps.observerLLA.climbRate = (int16_t)_MEISEI_getPayloadHalfWord(packet->fields, 9) / 100.0f;

    instance->_gps10 = _MEISEI_getPayloadHalfWord(packet->fields, 10);
    instance->_gps11 = _MEISEI_getPayloadHalfWord(packet->fields, 11);

    GPS_convertLLA2ECEF(&instance->gps.observerLLA, &instance->gps.observerECEF);

    return LPCLIB_SUCCESS;
}


