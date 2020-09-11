/* Copyright (c) 2015, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 * Neither the name of the author nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "lpclib.h"
#include "rinex.h"
#include "observer.h"

/* Lookup tables for day-of-week calculation */
static uint8_t _dowNM[12] = {0,3,3,6,1,4,6,2,5,0,3,5};


/* Global RINEX data (initialized from EEPROM) */
GPS_Ephemeris *ephemeris;

/* TOC (calculated from RINEX data) */
static uint32_t _ephemerisTOC[32];



/* Initialize ephemeris from RINEX data */
void EPHEMERIS_init (void)
{
    extern uint64_t _binary_rinexdata_bin_start;    // Use uint64_t to make the compiler assume 8-byte alignment
    ephemeris = (GPS_Ephemeris *)&_binary_rinexdata_bin_start;

    /* Calculate TOC value from discrete date/time */
    {
//TODO Sanity check for date/time!
        int prn;
        int W;
        GPS_EphemerisOneSat *ephem;

        for (prn = 1; prn <= 32; prn++) {
            ephem = &ephemeris->sats[prn];

            W = ephem->day;
            W += _dowNM[ephem->month - 1];
            W += ephem->year + ephem->year / 4;
            W += 6;                                     // 21st century
            if ((ephem->month < 3) && ((ephem->year % 4) == 0) && !((ephem->year % 100) == 0)) {
                W -= 1;
            }

            _ephemerisTOC[prn - 1] = ((W % 7) * 86400) + 3600 * ephem->hour + 60 * ephem->minute + ephem->second;
        }
    }
}


/* Calculate satellite position for a given time from ephemeris data */
void EPHEMERIS_calculateSatellitePosition (
    GPS_EphemerisOneSat *ephem,
    double transmissionTimeOfWeek,
    ECEF_Coordinate *coordinate,
    float estimatedRange
    )
{
    double tk;
    double A;
    volatile double n;
    volatile double Mk;
    volatile double Ek;
    double PHIk;
    double vk;
    volatile double uk, rk, ik;
    double xk, yk;
    double OMEGAk;
    int i;


    tk = transmissionTimeOfWeek - ephem->Toe;
    if (tk > 302400.0) {
        tk -= 604800.0;
    }
    if (tk < -302400.0) {
        tk += 604800.0;
    }

    A = ephem->sqrt_A * ephem->sqrt_A;
    n = sqrt(GPS_GRAVITATIONAL_CONSTANT / (A * A * A)) + ephem->Delta_n;
    Mk = ephem->M0 + n * tk;
    Ek = Mk;
    for (i = 0; i < 7; i++) {
        Ek = Mk + ephem->e * sin(Ek);    //TODO: check
    }
    vk = atan2(sqrt(1.0 - ephem->e * ephem->e) * sin(Ek), cos(Ek) - ephem->e);
    PHIk = vk + ephem->omega;
    uk = ephem->Cus * sin(2.0 * PHIk) + ephem->Cuc * cos(2.0 * PHIk) + PHIk;
    rk = ephem->Crs * sin(2.0 * PHIk) + ephem->Crc * cos(2.0 * PHIk) + A * (1.0 - ephem->e * cos(Ek));
    ik = ephem->Cis * sin(2.0 * PHIk) + ephem->Cic * cos(2.0 * PHIk) + ephem->i0 + ephem->IDOT * tk;

    xk = rk * cos(uk);
    yk = rk * sin(uk);

    OMEGAk = ephem->OMEGA + (ephem->OMEGA_DOT - GPS_WGS84_EARTH_ROTATION_RATE) * tk - GPS_WGS84_EARTH_ROTATION_RATE * (ephem->Toe + estimatedRange / GPS_SPEED_OF_LIGHT);

    coordinate->x = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
    coordinate->y = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
    coordinate->z = yk * sin(ik);


    float Ekdot, vkdot;
    float ukdot, rkdot, ikdot;
    Ekdot = n / (1.0f - ephem->e * cosf(Ek));
    vkdot = sinf(Ek) * Ekdot * (1.0f + ephem->e * cosf(vk)) / (sinf(vk) * (1.0f - ephem->e * cosf(vk)));
    ukdot = vkdot + 2.0f * (ephem->Cus * cosf(2.0f * uk) - ephem->Cuc * sinf(2.0f * uk)) * vk;
    rkdot = A * ephem->e * sinf(Ek) * n / (1.0f - ephem->e * cosf(uk)) + 2.0f * (ephem->Crs * cosf(2.0f * uk) - ephem->Crc * sinf(2.0f * uk)) * vk;
    ikdot = ephem->IDOT + (ephem->Cis * cosf(2.0f * uk) - ephem->Cic * sinf(2.0f * uk)) * 2.0f * vk;

    float xkdot, ykdot, OMEGAkdot;
    xkdot = rkdot * cosf(uk) - yk * ukdot;
    ykdot = rkdot * sinf(uk) + xk * ukdot;
    OMEGAkdot = ephem->OMEGA_DOT - GPS_WGS84_EARTH_ROTATION_RATE;

    coordinate->vx = (xkdot - yk * cosf(ik) * OMEGAkdot) * cosf(OMEGAk)
                    - (xk * OMEGAkdot + ykdot * cosf(ik) - yk * sinf(ik) * ikdot) * sinf(OMEGAk);
    coordinate->vy = (xkdot - yk * cosf(ik) * OMEGAkdot) * sinf(OMEGAk)
                    + (xk * OMEGAkdot + ykdot * cosf(ik) - yk * sinf(ik) * ikdot) * cosf(OMEGAk);
    coordinate->vz = ykdot * sinf(ik) + yk * cosf(ik) * ikdot;
}


/* Get "Time of Clock" (computed when loading RINEX data) */
uint32_t EPHEMERIS_getTOC (int prn)
{
    if ((prn < 1) || (prn > 32)) {
        return 0;
    }

    return _ephemerisTOC[prn - 1];
}


/* Determine the oldest valid satellite data.
 * If no valid satellite data exists, the function returns LPCLIB_ERROR, and an empty string
 * is returned in oldestSatTime.
 */
LPCLIB_Result EPHEMERIS_findOldestEntry (const char **oldestSatTime)
{
    if (oldestSatTime == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    const char *_noData = "";
    *oldestSatTime = _noData;

    uint32_t t_now;
    uint32_t t_oldest = (uint32_t)(-1);

    for (int prn = 1; prn <= 32; prn++) {
        /* Ignore SV if flagged as not healthy in broadcast ephemerides */
        if ((ephemeris->sats[prn].SV_health == 0) && (ephemeris->sats[prn].month > 0)) {
            /* Get time of this satellite and convert to an integer for comparison */
            t_now = (ephemeris->sats[prn].minute << 0);
            t_now += (ephemeris->sats[prn].hour << 6);
            t_now += (ephemeris->sats[prn].day << 11);
            t_now += (ephemeris->sats[prn].month << 16);
            t_now += (ephemeris->sats[prn].year << 20);

            /* Remember oldest timestamp */
            if (t_now < t_oldest) {
                t_oldest = t_now;
            }
        }
    }

    /* No satellite data? */
    if (t_oldest == (uint32_t)(-1)) {
        return LPCLIB_ERROR;
    }

    static char str_oldestSat[30];
    snprintf(str_oldestSat, sizeof(str_oldestSat),
             "%04"PRIu32"-%02"PRIu32"-%02"PRIu32"T%02"PRIu32":%02"PRIu32":00+0000",
             2000ul + ((t_oldest >> 20) & 0xFF),
             ((t_oldest >> 16) & 0x0F),
             ((t_oldest >> 11) & 0x1F),
             ((t_oldest >> 6) & 0x1F),
             ((t_oldest >> 0) & 0x3F)
    );
    *oldestSatTime = str_oldestSat;

    return LPCLIB_SUCCESS;
}
