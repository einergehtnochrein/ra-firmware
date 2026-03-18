
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include "lpclib.h"
#include "ncar.h"
#include "ncarprivate.h"
#include "observer.h"
#include "rinex.h"



static const double _rd41_coordinateFactor = M_PI / 180e7;

/* Process the GPS block. */
LPCLIB_Result _RD41_processGps1Block (
        const RD41_SubFrameGps1 *rawGps1,
        NCAR_CookedGps *cookedGps)
{
    uint32_t tow = 3600*100 * (rawGps1->xxx_hour & 0x1F)
                 + 60*100 * rawGps1->minute
                 + 100 * rawGps1->second
                 + rawGps1->second_hundredth;
    cookedGps->gpstime = tow / 100.0;   //TODO add GPS week!

    return LPCLIB_SUCCESS;
}


LPCLIB_Result _RD41_processGps2_3Block (
        const RD41_SubFrameGps2 *rawGps2,
        const RD41_SubFrameGps3 *rawGps3,
        NCAR_CookedGps *cookedGps)
{
    ECEF_Coordinate ecef;
    LLA_Coordinate lla;

    lla.lat = __REV(rawGps2->latitude_BE) * _rd41_coordinateFactor;
    lla.lon = __REV(rawGps2->longitude_BE) * _rd41_coordinateFactor;
    lla.velocity = __REV16(rawGps3->speed_BE) / 100.0f;
    lla.direction = 0;
    lla.climbRate = (int32_t)__REV16(rawGps3->climb_rate_BE) / 100.0f;
    lla.alt = (__REVSH(rawGps3->altitude_b2b1_BE) * 256 + rawGps3->altitude_b0) / 100.0f;
    GPS_convertLLA2ECEF(&lla, &ecef);
    /* NOTE: RD41 sends altitude as MSL value! */
    
    if (lla.alt < -100.0) {
        return LPCLIB_ERROR;
    }

    cookedGps->observerECEF = ecef;
    cookedGps->observerLLA = lla;

    return LPCLIB_SUCCESS;
}


LPCLIB_Result _RD41_processGps4Block (
        const RD41_SubFrameGps4 *rawGps4,
        NCAR_CookedGps *cookedGps)
{
(void)rawGps4;
(void)cookedGps;
    /* NOTE: This block contains the 2 Hz component of wind speed and altitude.
     *       We ignore this, the information from GPS block 2 is sufficient.
     */

    return LPCLIB_SUCCESS;
}

