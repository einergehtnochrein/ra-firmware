
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "gth3.h"
#include "gth3private.h"
#include "observer.h"
#include "rinex.h"



LPCLIB_Result _GTH3_processPayload (
        const GTH3_Payload *payload,
        GTH3_CookedGps *cookedGps,
        GTH3_CookedMetrology *cookedMetro)
{
(void)cookedMetro;
    LLA_Coordinate lla;

    lla.lat = payload->latitude;
    lla.lon = payload->longitude;
    lla.alt = payload->altitude / 10.0f;
    lla.climbRate = payload->speedV / 100.0f;
    float ve = payload->speedE / 100.0f;
    float vn = payload->speedN / 100.0f;
    lla.velocity = sqrtf(ve * ve + vn * vn);
    float direction = atan2f(ve, vn);
    if (direction < 0) {
        direction += 2 * M_PI;
    }
    lla.direction = direction;

    cookedGps->observerLLA = lla;

    return LPCLIB_SUCCESS;
}

