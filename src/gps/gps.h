/* Copyright (c) 2015-2016, DF9DQ
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


#ifndef __GPS_H
#define __GPS_H

#include "lpclib.h"


#define GPS_SECONDS_PER_WEEK                            604800ul
#define GPS_GRAVITATIONAL_CONSTANT                      3.986004418e14
#define GPS_SPEED_OF_LIGHT                              2.99792458e8
#define GPS_WGS84_EARTH_ROTATION_RATE                   7.2921151467e-5
#define GPS_CLOCK_CORRECTION_RELATIVISTIC_CONSTANT_F    -4.442807633e-10


/* ECEF coordinate */
typedef struct ECEF_Coordinate {
    double      x;
    double      y;
    double      z;
    float       vx;
    float       vy;
    float       vz;
} ECEF_Coordinate;



/* LLA coordinate (Latitude, Longitude, Altitude) */
typedef struct LLA_Coordinate {
    double      lat;
    double      lon;
    double      alt;
    float       climbRate;
    float       velocity;
    float       direction;
} LLA_Coordinate;


/* Status of a satellite */
typedef struct GPS_SatInfo {
    uint8_t PRN;
    bool ignore;
    float snr;
    ECEF_Coordinate pos;
    float elevation;
    float azimuth;
    float slantDist;
    float speed;
    float estimatedRangeRate;
    double prange;
    double clockcorr;
    double clockdrift;
    double ionoDelay;
} GPS_SatInfo;


/* Status of a set of four satellites.
 * Maintains the common numbering scheme 1...4
 */
typedef struct GPS_4SatInfo {
    GPS_SatInfo sat1;
    GPS_SatInfo sat2;
    GPS_SatInfo sat3;
    GPS_SatInfo sat4;
} GPS_4SatInfo;


/* Ephemeris for single satellite (extracted from RINEX file) */
typedef __PACKED(struct GPS_EphemerisOneSat {
    uint8_t     year;
    uint8_t     month;
    uint8_t     day;
    uint8_t     hour;
    uint8_t     minute;
    uint8_t     second;
    uint8_t     _fill6_;
    uint8_t     _fill7_;

    double      SV_clock_bias;
    double      SV_clock_drift;
    double      SV_clock_drift_rate;

    double      IODE;
    double      Crs;
    double      Delta_n;
    double      M0;

    double      Cuc;
    double      e;
    double      Cus;
    double      sqrt_A;

    double      Toe;
    double      Cic;
    double      OMEGA;
    double      Cis;

    double      i0;
    double      Crc;
    double      omega;
    double      OMEGA_DOT;

    double      IDOT;
    double      CodesL2;
    double      GPSWeek;
    double      L2P_data_flag;

    double      SV_accuracy;
    double      SV_health;
    double      TGD;
    double      IODC;

    double      TTOM;
    double      spare7_2;
}) GPS_EphemerisOneSat;


/* All content of RINEX file (one time sample only) */
typedef struct GPS_Ephemeris {
    GPS_EphemerisOneSat   sats[33];         /* Allow addressing directly by PRN 1-32 */

    double      iono_A0;
    double      iono_A1;
    double      iono_A2;
    double      iono_A3;
    double      iono_B0;
    double      iono_B1;
    double      iono_B2;
    double      iono_B3;

    double      deltaUTC_A0;
    double      deltaUTC_A1;
    uint32_t    deltaUTC_T;
    uint32_t    deltaUTC_W;

    uint32_t    leapsec;

    uint32_t    __fill__;   // GCC rounds the structure size up to a multiple of 8
} GPS_Ephemeris;


/* Convert from ECEF to LLA */
void GPS_convertECEF2LLA (
        const ECEF_Coordinate *ecef,
        LLA_Coordinate *lla
        );

/* Convert from LLA to ECEF */
void GPS_convertLLA2ECEF (
        const LLA_Coordinate *lla,
        ECEF_Coordinate *ecef
        );

/* Get ellipsoid radius for given LLA coordinate */
double GPS_getEllipsoidRadius (
        const LLA_Coordinate *lla
        );

void GPS_computeSatelliteClockCorrectionAndDrift (
        GPS_SatInfo *sat,
        double transmissionTimeOfWeek,
        GPS_EphemerisOneSat *ephem,
        uint32_t timeOfClock
     );

double GPS_computeIonosphericTimeDelay (
        float ulat,
        float ulon,
        float elevation,
        float azimuth,
        GPS_Ephemeris *ephem,
        double t
        );

void GPS_computeSatelliteElevationAndAzimuth (
        ECEF_Coordinate *observerPos,
        ECEF_Coordinate *satPos,
        float *elevation,
        float *azimuth
     );

void GPS_applyGeoidHeightCorrection (
        LLA_Coordinate *lla
       );

#endif

