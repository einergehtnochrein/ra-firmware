
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "rs92.h"
#include "rs92private.h"
#include "observer.h"
#include "rinex.h"



GPS_4SatInfo sats;

ECEF_Coordinate observer;


static struct {
    int32_t rang0;
    int32_t rang1;
    int32_t rang3;
    uint8_t snr;
} g[12];


/* Read 24-bit little-endian integer from memory */
static uint32_t _BLOCKRX_read24 (const uint8_t *p24)
{
    return p24[0] + 256 * p24[1] + 65536 * p24[2];
}


LPCLIB_Result _RS92_processGpsBlock (
        const struct _RS92_GpsBlock *rawGps,
        RS92_CookedGps *cookedGps,
        float pressureAltitude)
{
    int i;
    uint32_t numSats;


    /* Initialize default result in case no solution can be found */
    memset(cookedGps, 0, sizeof(RS92_CookedGps));
    cookedGps->observerLLA.lat = NAN;
    cookedGps->observerLLA.lon = NAN;
    cookedGps->observerLLA.alt = NAN;

    /* The timeMilliseconds parameter can be negative to indicate an error */
    if (rawGps->timeMilliseconds < 0) {
        return LPCLIB_ERROR;
    }
    cookedGps->gpstime = rawGps->timeMilliseconds / 1000.0;

    /* Get measurements of all 12 satellite channels */
    for (i = 0; i < 12; i++) {
        g[i].rang0 = rawGps->satData[i].val32;
        if ((g[i].rang0 < -50000000) || (g[i].rang0 > 50000000)) {
            g[i].rang0 = 0x7FFFFFFF;
        }
        g[i].rang1 = _BLOCKRX_read24(rawGps->satData[i].val24) & 0x7FFFFF;
        g[i].rang3 = rawGps->satData[i].val8;
        g[i].snr = rawGps->channel[i].snr;
    }

    /* Extract the satellite PRN's from the bit fields.
     * There are four 16-bit fields with three 5-bit PRN's each.
     * Set PRN to zero if the pseudorange value of that channel is invalid
     * (either 0x7FFFFFFF or a negative value), or in case of a marginal SNR.
     */
    for (i = 0; i < 4; i++) {
        cookedGps->sats[3*i+0].PRN = (rawGps->prn[i] >> 0 ) & 0x1F;     /* [4:0] */
        if ((g[3*i+0].rang0 == 0x7FFFFFFF) || (g[3*i+0].snr < 3)) {
            cookedGps->sats[3*i+0].PRN = 0;
            g[3*i+0].rang0 = 0x7FFFFFFF;
        }
        cookedGps->sats[3*i+1].PRN = (rawGps->prn[i] >> 5 ) & 0x1F;     /* [9:5] */
        if ((g[3*i+1].rang0 == 0x7FFFFFFF) || (g[3*i+1].snr < 3)) {
            cookedGps->sats[3*i+1].PRN = 0;
            g[3*i+1].rang0 = 0x7FFFFFFF;
        }
        cookedGps->sats[3*i+2].PRN = (rawGps->prn[i] >> 10) & 0x1F;     /* [14:10] */
        if ((g[3*i+2].rang0 == 0x7FFFFFFF) || (g[3*i+2].snr < 3)) {
            cookedGps->sats[3*i+2].PRN = 0;
            g[3*i+2].rang0 = 0x7FFFFFFF;
        }
    }

    /* If PRN32 is received (takes six bits to encode), RS92 tries to fit that into
    * the 5-bit fields extracted above, overwriting the LSB of the next channel
    * (if PRN32 occurs in the middle or lower 5-bit field), and leaving PRN=0 where there
    * should be PRN32.
    *
    * This can also happen if the channel receiving PRN32 can't eventually determine a valid
    * pseudorange (so there is PRN=0 and RANGE=0x7FFFFFFF). As this situation cannot be
    * distinguished from the situation where PRN=0 means no satellite has been received,
    * the channel following PRN=0 must always be treated as possibly corrupted.
    *
    * Two things must be fixed:
    * 1) Change PRN for the channel that contains 0 instead of 32:
    *    Walk through the list of PRN's, and replace 0 with PRN32 if that channel contains
    *    a valid pseudorange measurement.
    * 2) Fix the channel following a channel with PRN=0 if it is potentially corrupted by
    *    a leaked MSB of PRN32:
    *    Assume this following channel contains the *ODD* number PRN.
    *    The correct number is then either PRN or PRN-1.
    *    Search the other channels for an occurrence of either PRN or PRN-1:
    *    a) If we find PRN, the affected channel is corrupted, and must be replaced by PRN-1.
    *    b) If we find PRN-1, the PRN in the affected channel is genuine.
    *    c) If we find neither, no decision can be made for the affected channel as to whether
    *       it is corrupted or not, so it must be discarded.
    */

    /* Look for an occurrence of PRN32 */
    bool havePRN32 = false;
    for (i = 0; i < 12; i++) {
        if (cookedGps->sats[i].PRN == 0) {
            if (g[i].rang0 != 0x7FFFFFFF) {
                /* This PRN=0 is in fact PRN32 */
                cookedGps->sats[i].PRN = 32;
                havePRN32 = true; /* PRN32 has been positively identified */
            }
        }
    }

    /* Fix the potentially corrupted channel.
     * Note that unless PRN32 has been positively identified, there can be more than
     * one candidate for corruption!
     */
    for (i = 0; i < 12; i++) {
        if ((cookedGps->sats[i].PRN == 32) ||
            ((cookedGps->sats[i].PRN == 0) && !havePRN32)) {
            if ((i % 3) != 2) {     /* PRN32 in channels 2, 5, 8, 11 doesn't hurt */
                /* Check if affected channel contains a valid pseudorange */
                if (g[i+1].rang0 != 0x7FFFFFFF) {
                    /* An even PRN cannot be corrupted! */
                    if ((cookedGps->sats[i+1].PRN % 2) == 1) {
                        int j;

                        /* See if next channel's PRN (or PRN-1) occurs somewhere else */
                        for (j = 0; j < 12; j++) {
                            if (j != i+1) {
                                if (cookedGps->sats[j].PRN == cookedGps->sats[i+1].PRN) {
                                    /* We know the affected channel can only be PRN-1 */
                                    cookedGps->sats[i+1].PRN -= 1;
                                    break;
                                }
                                if (cookedGps->sats[j].PRN == cookedGps->sats[i+1].PRN-1) {
                                    /* We know the affected channel can only be PRN */
                                    break;
                                }
                            }
                        }
                        if (j >= 12) {
                            /* TODO. Give up, invalidate next channel */
                            cookedGps->sats[i+1].PRN = 0;
                        }
                    }
                }
            }
        }
    }

    /* NOTE: In case PRN32 is not received, the fix above might discard more channels than necessary.
     * This is because there could be several channels for which no decision can be made as to whether
     * there is corruption or not.
     * If only few channels are being received, a more sophisticated trial-and-error procedure could
     * help to find a position solution at all.
     */

    /* For all channels with a valid PRN calculate satellite parameters
     * (position, clock, constellation as seen from approximate observer location)
     */
    numSats = 0;
    for (i = 0; i < 12; i++) { 
        if ((cookedGps->sats[i].PRN < 1) || (cookedGps->sats[i].PRN > 32)) {
            continue;
        }

        /* Ignore SV if flagged as not healthy in broadcast ephemerides */
        if (ephemeris->sats[cookedGps->sats[i].PRN].SV_health > 0) {
            continue;
        }

        ECEF_Coordinate observerPos = {.x = 4642822.0, .y = 1028660.0, .z = 4236692.0};  //TODO Rom...

        GPS_computeSatelliteClockCorrectionAndDrift(
                            &cookedGps->sats[i],
                            cookedGps->gpstime,
                            &ephemeris->sats[cookedGps->sats[i].PRN],
                            EPHEMERIS_getTOC(cookedGps->sats[i].PRN));

        float estimatedRange = 0.07f * 3e8f;

        EPHEMERIS_calculateSatellitePosition(
                            &ephemeris->sats[cookedGps->sats[i].PRN],
                            cookedGps->gpstime,
                            &cookedGps->sats[i].pos,
                            estimatedRange
                            );

        estimatedRange = sqrtf(
                            (observerPos.x - cookedGps->sats[i].pos.x) * (observerPos.x - cookedGps->sats[i].pos.x)
                            + (observerPos.y - cookedGps->sats[i].pos.y) * (observerPos.y - cookedGps->sats[i].pos.y)
                            + (observerPos.z - cookedGps->sats[i].pos.z) * (observerPos.z - cookedGps->sats[i].pos.z)
                            );
        EPHEMERIS_calculateSatellitePosition(
                            &ephemeris->sats[cookedGps->sats[i].PRN],
                            cookedGps->gpstime,
                            &cookedGps->sats[i].pos,
                            estimatedRange
                            );

        cookedGps->sats[i].prange = (5e7 - g[i].rang0) * (GPS_SPEED_OF_LIGHT / (64.0f * 16.368e6f));
        cookedGps->sats[i].prange -= cookedGps->rxClockOffset;

        if (g[i].rang0 != 0x7FFFFFFF) {
            ++numSats;
        }

        {
            GPS_computeSatelliteElevationAndAzimuth(
                                    &observerPos,
                                    &cookedGps->sats[i].pos,
                                    &cookedGps->sats[i].elevation,
                                    &cookedGps->sats[i].azimuth);

float dx, dy, dz, slant;
dx = observerPos.x - cookedGps->sats[i].pos.x;
dy = observerPos.y - cookedGps->sats[i].pos.y;
dz = observerPos.z - cookedGps->sats[i].pos.z;
slant = sqrtf(dx * dx + dy * dy + dz * dz);
cookedGps->sats[i].speed = (slant - cookedGps->sats[i].slantDist);
cookedGps->sats[i].slantDist = slant;
        }

        {
            LLA_Coordinate position;

            GPS_convertECEF2LLA(&observerPos, &position);
            cookedGps->sats[i].ionoDelay = GPS_computeIonosphericTimeDelay(
                    position.lat,
                    position.lon,
                    cookedGps->sats[i].elevation,
                    cookedGps->sats[i].azimuth,
                    ephemeris,
                    cookedGps->gpstime
                    );
        }
    }

    cookedGps->visibleSats = numSats;

    {
        if (cookedGps->visibleSats >= 4) {
            /* Precision solution involving at least four satellites */
            ECEF_Coordinate startPos = {.x = 4642822.0, .y = 1028660.0, .z = 4236692.0};  //TODO... 

            GPS_findPositionSolutionAllSats(
                cookedGps->sats,
                12,
                cookedGps->gpstime,
                &startPos,
                &cookedGps->rxClockOffset,
                &cookedGps->hdop,
                pressureAltitude
                );

cookedGps->valid=false;
if(cookedGps->hdop<20.0f){
    cookedGps->valid=true;
    cookedGps->observerECEF=startPos;

    GPS_convertECEF2LLA(&cookedGps->observerECEF, &cookedGps->observerLLA);
    if((cookedGps->observerLLA.alt > -100.0f) && (cookedGps->observerLLA.alt < 50000.0f)){
        cookedGps->valid=true;
    }
}

#if SEMIHOSTING_RS92
            printf("pos = %f %f %f\r\n", cookedGps->observerLLA.lat*57.2958, cookedGps->observerLLA.lon*57.2958, cookedGps->observerLLA.alt);
#endif
        }
    }

    return LPCLIB_SUCCESS;
}

