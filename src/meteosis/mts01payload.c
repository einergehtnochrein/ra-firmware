
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "mts01.h"
#include "mts01private.h"
#include "observer.h"


LPCLIB_Result _MTS01_processPayload (
        const MTS01_Packet *payload,
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
    const char *p = &payload->rawData.dat8[0];

    double latitude = 0;
    double longitude = 0;
    int nFields = 0;

    /* Read the first field */
    char *token = strtok(p, ",");
    /* Process up to 18 fields */
    while (token && (nFields < 18)) {
        switch (nFields++) {
            case 0: instance->serialNumber = atol(token);
                    break;
            case 2: instance->frameCounter = atol(token);
                    break;
            case 5: latitude = atof(token);
                    lla.lat = latitude * M_PI / 180.0;
                    break;
            case 6: longitude = atof(token);
                    lla.lon = longitude * M_PI / 180.0;
                    break;
            case 7: lla.alt = atol(token);
                    break;
        }

        /* Read the next field */
        token = strtok(NULL, ",");
    }

    instance->gps.observerLLA = lla;
    instance->gps.usedSats = 0;
    instance->gps.pdop = 0;

    if (nFields >= 18) {
        result = LPCLIB_SUCCESS;
    }

    return result;
}

