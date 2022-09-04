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


#include <math.h>
#include <string.h>

#include "lpclib.h"
#include "config.h"
#include "gps.h"

#define WGS84_a         6378137.0
#define WGS84_b         6356752.3142
#define WGS84_e         0.0818191909316
#define WGS84_eprime    0.0820944380396



/* Convert from ECEF to LLA */
void GPS_convertECEF2LLA (
        const ECEF_Coordinate *ecef,
        LLA_Coordinate *lla
        )
{
    double p;
    double theta;
    double N;

    p = sqrt(ecef->x * ecef->x + ecef->y * ecef->y);
    theta = atan2(ecef->z * WGS84_a, p * WGS84_b);

    lla->lon = atan2(ecef->y, ecef->x);
    lla->lat = atan2(ecef->z + WGS84_eprime * WGS84_eprime * WGS84_b * sin(theta) * sin(theta) * sin(theta),
                      p - WGS84_e * WGS84_e * WGS84_a * cos(theta) * cos(theta) * cos(theta)
               );
    N = WGS84_a / sqrt(1.0 - WGS84_e * WGS84_e * sin(lla->lat) * sin(lla->lat));
    lla->alt = p / cos(lla->lat) - N;

    float fx = ecef->vx * cosf(lla->lat) * cosf(lla->lon);
    float fy = ecef->vy * cosf(lla->lat) * sinf(lla->lon);
    float fz = ecef->vz * sinf(lla->lat);
    lla->climbRate = fx + fy + fz;

    float ve =
        ecef->vx * (-sin(lla->lon))
      + ecef->vy * cos(lla->lon);
    float vn =
        ecef->vx * (-cos(lla->lon) * sin(lla->lat))
      + ecef->vy * (-sin(lla->lon) * sin(lla->lat))
      + ecef->vz * cos(lla->lat);

    lla->velocity = sqrtf(ve * ve + vn * vn);
    lla->direction = atan2f(ve, vn);    /* North = 0° */
    if (lla->direction <= 0) {
        lla->direction += 2.0f * M_PI;
    }
}



/* Convert from LLA to ECEF */
void GPS_convertLLA2ECEF (
        const LLA_Coordinate *lla,
        ECEF_Coordinate *ecef
        )
{
    double N;


    N = WGS84_a / sqrt(1.0 - WGS84_e * WGS84_e * sin(lla->lat) * sin(lla->lat));
    ecef->x = (N + lla->alt) * cos(lla->lat) * cos(lla->lon);
    ecef->y = (N + lla->alt) * cos(lla->lat) * sin(lla->lon);
    ecef->z = ((1.0 - WGS84_e * WGS84_e) * N + lla->alt) * sin(lla->lat);
}



/* Get ellipsoid radius for given LLA coordinate */
double GPS_getEllipsoidRadius (const LLA_Coordinate *lla)
{
    double N = WGS84_b / sqrt(1.0 - WGS84_e * WGS84_e * cos(lla->lat) * cos(lla->lat));

    return N;
}



void GPS_computeSatelliteClockCorrectionAndDrift (
        GPS_SatInfo *sat,
        double transmissionTimeOfWeek,
        GPS_EphemerisOneSat *ephem,
        uint32_t timeOfClock
     )
{
    double tot;
    double tk;
    volatile double tc;
    double A;
    double n;
    double Mk;
    double Ek;
    volatile double d_tr;
    volatile double d_tsv;
    int i;
    uint32_t transmissionWeek;
    double toc;


    toc = timeOfClock;
    transmissionWeek = lrintf(ephem->GPSWeek) % 1024;

//TODO transmissionWeek+1 ???
    tot = transmissionWeek * GPS_SECONDS_PER_WEEK + transmissionTimeOfWeek;  // TODO  find real week of transmission!
    tk = tot - (transmissionWeek * GPS_SECONDS_PER_WEEK + ephem->Toe);
    tc = tot - (transmissionWeek * GPS_SECONDS_PER_WEEK + toc);

    A = ephem->sqrt_A * ephem->sqrt_A;
    n = sqrt(GPS_GRAVITATIONAL_CONSTANT / (A * A * A)) + ephem->Delta_n;
    Mk = ephem->M0 + n * tk;
    Ek = Mk;
    for (i = 0; i < 7; i++) {
        Ek = Mk + ephem->e * sin(Ek);    //TODO: check
    }

    d_tr = GPS_SPEED_OF_LIGHT * GPS_CLOCK_CORRECTION_RELATIVISTIC_CONSTANT_F * ephem->e * ephem->sqrt_A * sin(Ek);
    d_tsv = ephem->SV_clock_bias
          + ephem->SV_clock_drift * tc
          + ephem->SV_clock_drift_rate * tc * tc
          - ephem->TGD;

    sat->clockcorr = d_tsv * GPS_SPEED_OF_LIGHT + d_tr;
    sat->clockdrift = (ephem->SV_clock_drift + 2.0 * ephem->SV_clock_drift_rate * tc) * GPS_SPEED_OF_LIGHT;
}



double GPS_computeIonosphericTimeDelay (
        float ulat,
        float ulon,
        float elevation,
        float azimuth,
        GPS_Ephemeris *ephem,
        double t
        )
{
    float fm;
    float xilat, xilon;
    float gmlat;
    double tsub;
    double F;
    double alpha, beta;
    double x, y;


    /* Calculate earth angle to ionospheric pierce point.
     * Earth angle is angle between earth center to pierce point and earth center to observer.
     * Formula below assumes h=350 km
     */
    fm = acosf(6370.0f/(6370.0f + 350.0f) * cosf(elevation)) - elevation;

    /* Calculate latitude/longitude of the ionospheric pierce point.
     * These are approximations assuming a flat earth!
     */
    xilat = ulat + fm * cosf(azimuth);
    if (xilat > 75.0f * (M_PI / 180.0f)) {
        xilat = 75.0f * (M_PI / 180.0f);
    }
    if (xilat < -75.0f * (M_PI / 180.0f)) {
        xilat = -75.0f * (M_PI / 180.0f);
    }
    xilon = ulon + fm * sinf(azimuth) / cosf(xilat);

    /* Transform latitude into the geomagnetic reference system (earth-centered dipole),
     * assuming a north pole location of 78.3°N/291°E.
     * Again, this is an approximation to within 1° of the precise solution.
     */
    gmlat = xilat/M_PI + 0.064f * cosf(xilon - 291.0f * (M_PI/180.0f));

    /* Find the time of day at that position.
     * NOTE: we add 86400 to be sure we always have a positive argument for fmodf()
     */
    tsub = fmodf(43200.0f * xilon/M_PI + t + 86400.0f, 86400.0f);

    F = 1.0 + 16.0 * pow(0.53 - elevation / M_PI, 3);

    alpha = ephem->iono_A0 + gmlat * ephem->iono_A1 + gmlat * gmlat * ephem->iono_A2 + gmlat * gmlat * gmlat * ephem->iono_A3;
    beta  = ephem->iono_B0 + gmlat * ephem->iono_B1 + gmlat * gmlat * ephem->iono_B2 + gmlat * gmlat * gmlat * ephem->iono_B3;
    if (alpha < 0.0) {
        alpha = 0.0;
    }
    if (beta < 72000.0) {
        beta = 72000.0;
    }

    x = 2.0 * M_PI * (tsub - 50400.0) / beta;

    if (fabs(x) <= M_PI / 2.0) {
        y = alpha * (1.0 - (x * x) / 2.0 + (x * x * x * x) / 24.0);
    }
    else {
        y = 0.0;
    }

    return F * (5e-9 + y) * GPS_SPEED_OF_LIGHT;
}



void GPS_computeSatelliteElevationAndAzimuth (
        ECEF_Coordinate *observerPos,
        ECEF_Coordinate *satPos,
        float *elevation,
        float *azimuth
     )
{
    LLA_Coordinate localLLA;
    float dX, dY, dZ;
    float sinlat, coslat, sinlon, coslon;
    float easting, northing, up;

    /* Get observer's latitude/longitude from its ECEF coordinates */
    GPS_convertECEF2LLA(observerPos, &localLLA);

    /* The view line from observer to the satellite */
    dX = satPos->x - observerPos->x;
    dY = satPos->y - observerPos->y;
    dZ = satPos->z - observerPos->z;

    /* For the transformation to follow we need sin/cos of both observer's latitude and longitude */
    sinlat = sinf(localLLA.lat);
    coslat = cosf(localLLA.lat);
    sinlon = sinf(localLLA.lon);
    coslon = cosf(localLLA.lon);

    /* Transform view line into coordinates of the tangential plane at the observer's location */
    easting = -sinlon * dX + coslon * dY;
    northing = -sinlat * coslon * dX - sinlat * sinlon * dY + coslat * dZ;
    up = coslat * coslon * dX + coslat * sinlon * dY + sinlat * dZ;

    /* Get elevation from 2-quadrant arctan */
    *elevation = atanf(up / sqrtf(northing * northing + easting * easting));

    /* Get azimuth from 4-quadrant arctan. Note that easting and northing are swapped,
     * so that the angle is zero for direction north, and counting clockwise.
     * As western directions are in the range [-Pi:0], apply a correction to
     * transform them to [Pi:2Pi].
     */
    *azimuth = atan2f(easting, northing);
    if (*azimuth < 0) {
        *azimuth += 2.0f * M_PI;
    }
}


void GPS_applyGeoidHeightCorrection (
        LLA_Coordinate *lla
       )
{
    if (lla) {
        if (!isnan(lla->alt)) {
            lla->alt += CONFIG_getGeoidHeight();
        }
    }
}

