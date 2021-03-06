
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
    uint8_t satIndex;
    uint8_t satValue;
    float f;
    int i;

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

            f = _MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 10);
            instance->gps.observerLLA.velocity = (f / 200.0f);

            f = _MEISEI_getPayloadHalfWord(instance->gpsPacketEven.fields, 9);
            instance->gps.observerLLA.direction = (f / 100.0f) * (float)(M_PI / 180.0f);

            instance->gps.observerLLA.climbRate = NAN;

            GPS_applyGeoidHeightCorrection(&instance->gps.observerLLA);
            GPS_convertLLA2ECEF(&instance->gps.observerLLA, &instance->gps.observerECEF);
        }

        /* Satellite info */
        uint8_t numSats = 0;
        satIndex = _MEISEI_getPayloadHalfWord(instance->gpsPacketOdd.fields, 2) & 0xFF;
        for (i = 0; i < 16; i++) {
            if (i % 2) {
                satValue = _MEISEI_getPayloadHalfWord(instance->gpsPacketOdd.fields, 3 + (i / 2)) & 0xFF;
            }
            else {
                satValue = (_MEISEI_getPayloadHalfWord(instance->gpsPacketOdd.fields, 3 + (i / 2)) >> 8) & 0xFF;
            }

            switch (satIndex) {
                case 0:
                    instance->gps.sats[i].PRN = satValue;
                    if (satValue != 0) {
                        ++numSats;
                    }
                    break;
                case 1:
                    instance->gps.sats[i].snr = satValue;
                    break;
                case 2:
                    instance->gps.sats[i].xxx1 = satValue;
                    break;
                case 3:
                    instance->gps.sats[i].xxx2 = satValue;
                    break;
            }
        }
        instance->gps.usedSats = numSats;

        instance->gps.milliseconds = _MEISEI_getPayloadHalfWord(instance->configPacketEven.fields, 10);
        instance->gps.count1024 = _MEISEI_getPayloadHalfWord(instance->configPacketOdd.fields, 10);
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


