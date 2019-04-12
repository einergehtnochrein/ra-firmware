
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



/* Index: 0...11 */
static uint16_t _MEISEI_getPayloadHalfWord (
        const MEISEI_Packet *packet,
        int index)
{
    uint64_t field = packet->fields[index / 2];

    uint32_t x = 0;
    if (index % 2) {
        x = (field & 0x00000001FFFE0000LL) >> 1;
    }
    else {
        x = (field & 0x000000000000FFFFLL) << 16;
    };

    return __RBIT(x);
}



LPCLIB_Result _MEISEI_processGpsFrame (
        const MEISEI_Packet *packet,
        MEISEI_InstanceData *instance)
{
    int32_t i32;

    if (instance == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    i32 = (_MEISEI_getPayloadHalfWord(packet, 1) << 16) | _MEISEI_getPayloadHalfWord(packet, 2);
    instance->gps.observerLLA.lat = (i32 / 1e7) * (M_PI / 180.0);
    i32 = (_MEISEI_getPayloadHalfWord(packet, 3) << 16) | _MEISEI_getPayloadHalfWord(packet, 4);
    instance->gps.observerLLA.lon = (i32 / 1e7) * (M_PI / 180.0);
    i32 = (_MEISEI_getPayloadHalfWord(packet, 5) << 16) | _MEISEI_getPayloadHalfWord(packet, 6);
    instance->gps.observerLLA.alt = (i32 / 1e2);

    GPS_convertLLA2ECEF(&instance->gps.observerLLA, &instance->gps.observerECEF);

    return LPCLIB_SUCCESS;
}


