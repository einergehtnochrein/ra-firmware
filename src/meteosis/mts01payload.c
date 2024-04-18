
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "mts01.h"
#include "mts01private.h"
#include "observer.h"


/* Convert raw temperature measurement (resistance) to temperature [°C]
 * MTS-01 sends no calibration values, so we use the standard NTC equation with
 * average parameters (B=3000, R0=15) obtained from previous flights.
 */
static float _MTS01_getTemperature (float raw) {
    return 1.0f / (logf(raw / 15.0f) / 3000.0f + 1.0f / 273.15f) - 273.15f;
}


LPCLIB_Result _MTS01_processPayload (
        MTS01_Packet *payload,
        MTS01_InstanceData *instance)
{
    LPCLIB_Result result = LPCLIB_ERROR;
    LLA_Coordinate lla;

    lla.alt = NAN;
    lla.lat = NAN;
    lla.lon = NAN;
    lla.direction = NAN;
    lla.velocity = NAN;
    lla.climbRate = NAN;

    /* The first few comma-separated ASCII fields are present in all frames */
    char *p = &payload->rawData.dat8[0];

    double latitude = 0;
    double longitude = 0;
    int nFields = 0;
    float temperature = NAN;

    /* Read the first field */
    char *token = strtok(p, ",");
    /* Process up to 18 fields */
    while (token && (nFields < 18)) {
        switch (nFields++) {
            case 0:
                strncpy(instance->name, token, sizeof(instance->name));
                instance->name[sizeof(instance->name) - 1] = 0;
                break;
            case 2:
                instance->frameCounter = atol(token);
                break;
            case 5:
                latitude = atof(token);
                lla.lat = latitude * M_PI / 180.0;
                break;
            case 6:
                longitude = atof(token);
                lla.lon = longitude * M_PI / 180.0;
                break;
            case 7:
                lla.alt = atol(token);
                break;
            case 11:
                temperature = _MTS01_getTemperature(atof(token));
                break;
        }

        /* Read the next field */
        token = strtok(NULL, ",");
    }

    instance->gps.observerLLA = lla;
    instance->gps.usedSats = 0;
    instance->gps.pdop = 0;
    instance->metro.temperature = temperature;

    if (nFields >= 18) {
        result = LPCLIB_SUCCESS;
    }

    return result;
}

