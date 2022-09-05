
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "ht03.h"
#include "ht03private.h"
#include "observer.h"
#include "rinex.h"



LPCLIB_Result _HT03_processPayload (
        const HT03_Payload *payload,
        HT03_CookedGps *cookedGps,
        HT03_CookedMetrology *cookedMetro)
{
(void)cookedMetro;
    LLA_Coordinate lla;

    lla.lat = NAN;
    lla.lon = NAN;
    lla.alt = NAN;
    lla.climbRate = NAN;
    lla.velocity = NAN;
    lla.direction = NAN;

    cookedGps->observerLLA = lla;

    return LPCLIB_SUCCESS;
}

