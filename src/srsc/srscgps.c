
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "srsc.h"
#include "srscprivate.h"


LPCLIB_Result _SRSC_processGpsFrame (
        const SRSC_Packet *rawGps,
        SRSC_CookedGps *cookedGps)
{
    int32_t i32;
    float f;
    uint32_t data = __REV(rawGps->d_bigendian);
    static float lastAltitude[5];
    static float lastAltitudeTime[5];

    switch (rawGps->type) {
//TODO TODO TODO C34/C50 type must be provided as input!
        case SRSC_FRAME_GPS_DATE:
            cookedGps->gpsDate = data;
            break;

        case SRSC_FRAME_GPS_TIME:
            cookedGps->gpsTime = data;
            break;

        case SRSC_FRAME_GPS_LATITUDE:
            i32 = (int32_t)data / 1000000;
            if ((i32 < -90) || (i32 > 90)) {  // C50
                i32 = (int32_t)data / 10000000;
                f = i32;
                f += (data % 10000000) / 6e6f;
            }
            else {
                f = i32;
                f += (data % 1000000) / 6e5f;
            }
            cookedGps->observerLLA.lat = f / 180.0f * M_PI;
            cookedGps->updateFlags |= 1;
            break;

        case SRSC_FRAME_GPS_LONGITUDE:
            i32 = (int32_t)data / 1000000;
            if (i32 > 40) {  // C50  TODO TODO TODO!!!!!
                i32 = (int32_t)data / 10000000;
                f = i32;
                f += (data % 10000000) / 6e6f;
            }
            else {
                f = i32;
                f += (data % 1000000) / 6e5f;
            }
            cookedGps->observerLLA.lon = f / 180.0f * M_PI;
            cookedGps->updateFlags |= 2;
            break;

        case SRSC_FRAME_GPS_ALTITUDE:
            cookedGps->observerLLA.alt = (float)data / 10.0f;
            cookedGps->updateFlags |= 4;

            //TODO Extremely simple climb/sink rate...
            {
                lastAltitude[4] = lastAltitude[3];
                lastAltitude[3] = lastAltitude[2];
                lastAltitude[2] = lastAltitude[1];
                lastAltitude[1] = lastAltitude[0];
                lastAltitude[0] = cookedGps->observerLLA.alt;

                lastAltitudeTime[4] = lastAltitudeTime[3];
                lastAltitudeTime[3] = lastAltitudeTime[2];
                lastAltitudeTime[2] = lastAltitudeTime[1];
                lastAltitudeTime[1] = lastAltitudeTime[0];
                lastAltitudeTime[0] = os_time;

                float tmean = (lastAltitudeTime[4]+lastAltitudeTime[3]+lastAltitudeTime[2]+lastAltitudeTime[1]+lastAltitudeTime[0])/5.0f;
                float amean = (lastAltitude[4]+lastAltitude[3]+lastAltitude[2]+lastAltitude[1]+lastAltitude[0])/5.0f;
                if (tmean > 0.01f) {
                    float s1 = 0;
                    float s2 = 0;

                    f = lastAltitudeTime[4] - tmean;
                    s1 += f * (lastAltitude[4] - amean);
                    s2 += f * f;
                    f = lastAltitudeTime[3] - tmean;
                    s1 += f * (lastAltitude[3] - amean);
                    s2 += f * f;
                    f = lastAltitudeTime[2] - tmean;
                    s1 += f * (lastAltitude[2] - amean);
                    s2 += f * f;
                    f = lastAltitudeTime[1] - tmean;
                    s1 += f * (lastAltitude[1] - amean);
                    s2 += f * f;
                    f = lastAltitudeTime[0] - tmean;
                    s1 += f * (lastAltitude[0] - amean);
                    s2 += f * f;

                    if (s2 > 0.01f) {
                        cookedGps->climbRate = 100.0f * s1 / s2;
                    }
                }
            }
            break;

        case SRSC_FRAME_GPS_QUALITY:
            //TODO not confirmed, just a wild guess...
            cookedGps->hdop = (float)(data / 100) / 100.0f;
            cookedGps->usedSats = data % 100;
            break;

        default:
            break;
    }

    if (cookedGps->usedSats == 0) {
        cookedGps->observerLLA.lat = NAN;
        cookedGps->observerLLA.lon = NAN;
        cookedGps->observerLLA.alt = NAN;
    }

    return LPCLIB_SUCCESS;
}


