
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "lms6.h"
#include "lms6private.h"
#include "observer.h"
#include "rinex.h"


static const double _lms6_coordinateFactor = M_PI / 2147483648.0;


/* Read 24-bit little-endian signed integer from memory */
static int32_t _LMS6_read24_BigEndian (const uint8_t *p24)
{
    int32_t value = p24[2] + 256 * p24[1] + 65536 * p24[0];
    if (value & (1u << 23)) {
        value |= 0xFF000000;
    }

    return value;
}



LPCLIB_Result _LMS6_processPayload (
        const LMS6_RawFrame *payload,
        LMS6_CookedGps *cookedGps,
        LMS6_CookedMetrology *cookedMetro)
{
    LLA_Coordinate lla;

    lla = cookedGps->observerLLA;

    lla.lat = __REV(payload->latitude) * _lms6_coordinateFactor;
    lla.lon = __REV(payload->longitude) * _lms6_coordinateFactor;
    lla.alt = __REV(payload->altitude) * 0.001f;
    lla.climbRate = _LMS6_read24_BigEndian(payload->reserved20.d) / 100.0f;
    lla.direction = NAN;
    lla.velocity = NAN;

    cookedGps->observerLLA = lla;
    cookedGps->tow = __REV(payload->time_ms);

    cookedMetro->pressure = NAN;
    cookedMetro->temperature = NAN;
    cookedMetro->humidity = NAN;

    return LPCLIB_SUCCESS;
}

