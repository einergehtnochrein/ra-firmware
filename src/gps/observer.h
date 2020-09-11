#ifndef __OBSERVER_H
#define __OBSERVER_H

#include "gps.h"

/* Compute the range rate of an SV relative to a user at rest.
 */
bool GPS_computeUserToSVRangeRate (ECEF_Coordinate *observer, ECEF_Coordinate *sv, float *rangeRate);

/** Solve the observer equeations for four satellites in closed form.
 */
bool GPS_PositionSolution (
    GPS_4SatInfo *sats,
    LLA_Coordinate *position
    );

/* Find position solution using all suitable satellites.
 * Uses least square algorithm.
 * Requires a useful initial guess.
 */
void GPS_findPositionSolutionAllSats (
    GPS_SatInfo *sats,
    int numSats,
    double receiverEpoch,
    ECEF_Coordinate *startPos,
    double *rxClockOffset,
    float *hdop,
    float pressureAltitude,
    uint8_t *pUsedSats
    );

/* Determine suitability of satellite selection for a position solution. */
float GPS_checkSatelliteConstellation (GPS_4SatInfo *sats);

#endif  /* __OBSERVER_H */


