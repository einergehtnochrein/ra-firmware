
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "rs41.h"
#include "rs41private.h"
#include "observer.h"
#include "rinex.h"



LPCLIB_Result _RS41_processGpsPositionBlock (
        const RS41_SubFrameGpsPosition *rawGps,
        RS41_CookedGps *cookedGps)
{
    ECEF_Coordinate ecef;
    LLA_Coordinate lla;

    ecef.x = rawGps->ecefX / 100.0;
    ecef.y = rawGps->ecefY / 100.0;
    ecef.z = rawGps->ecefZ / 100.0;
    ecef.vx = rawGps->speedX / 100.0f;
    ecef.vy = rawGps->speedY / 100.0f;
    ecef.vz = rawGps->speedZ / 100.0f;
    GPS_convertECEF2LLA(&ecef, &lla);
    GPS_applyGeoidHeightCorrection(&lla);

    if (lla.alt < -100.0) {
        return LPCLIB_ERROR;
    }

    cookedGps->observerECEF = ecef;
    cookedGps->observerLLA = lla;
    cookedGps->usedSats = rawGps->usedSats;
    cookedGps->pdop = rawGps->pdop / 10.0f;
    cookedGps->sAcc = rawGps->sAcc;
    cookedGps->estimatedPressure = 1013.25f * expf(-1.18575919e-4f * lla.alt);

    /* Invalidate position if this is not a valid position solution */
    if (cookedGps->usedSats == 0) {
        cookedGps->observerLLA.lat = NAN;
        cookedGps->observerLLA.lon = NAN;
        cookedGps->observerLLA.alt = NAN;
        cookedGps->estimatedPressure = NAN;
    }

    return LPCLIB_SUCCESS;
}


/* Process the GPS block (sat info). */
LPCLIB_Result _RS41_processGpsInfoBlock (
        const RS41_SubFrameGpsInfo *rawGps,
        RS41_CookedGps *cookedGps,
        RS41_RawGps *raw)
{
    int i;
    uint8_t nSats = 0;

    cookedGps->gpstime = SECONDS_BETWEEN_UNIX_GPS + (rawGps->gpsWeek * GPS_SECONDS_PER_WEEK) + (rawGps->timeOfWeek / 1000.0);

    for (i = 0; i < 12; i++) {
        raw->sats[i].prn = rawGps->sats[i].prn;

        /* Count number of valid sats */
        if (raw->sats[i].prn != (uint8_t)-1) {
            ++nSats;
        }

        raw->sats[i].cno = rawGps->sats[i].cno_mesQI / 32;
        raw->sats[i].mesQI = rawGps->sats[i].cno_mesQI % 32;
    }

    cookedGps->visibleSats = nSats;

    return LPCLIB_SUCCESS;
}


LPCLIB_Result _RS41_processGpsRawBlock (
        const RS41_SubFrameGpsRaw *p,
        RS41_RawGps *raw,
        RS41_CookedGps *cookedGps)
{
    int i;

    for (i = 0; i < 12; i++) {
        raw->sats[i].pseudorange =  (double)p->minPrMes + p->sats[i].deltaPrMes / 256.0;
        raw->sats[i].doppler = (int32_t)(_RS41_readS24(p->sats[i].doMes) << 8) / 256;
    }

    if (p->mon_jam != 0xFF) {
        cookedGps->jammer = p->mon_jam >> 4;
    }

    return LPCLIB_SUCCESS;
}


