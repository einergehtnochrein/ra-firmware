
#include <inttypes.h>
#include <math.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "windsond.h"
#include "windsondprivate.h"
#include "bch.h"


/** Context */
typedef struct WINDSOND_Context {
    WINDSOND_Packet packet;

    WINDSOND_InstanceData *instance;
    float rxFrequencyHz;

} WINDSOND_Context;

static WINDSOND_Context _windsond;


//TODO
LPCLIB_Result WINDSOND_open (WINDSOND_Handle *pHandle)
{
    WINDSOND_Handle handle = &_windsond;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _WINDSOND_sendKiss (WINDSOND_InstanceData *instance)
{
    static char s[160];
    int length = 0;
    float offset;
    char sPressure[10];

    offset = 0;

    /* Pressure as string */
    sPressure[0] = 0;
    if (!isnan(instance->metro.pressure)) {
        snprintf(sPressure, sizeof(sPressure), "%.1f", instance->metro.pressure);
    }

    /* Convert lat/lon from radian to decimal degrees */
    double latitude = instance->gps.observerLLA.lat;
    if (!isnan(latitude)) {
        latitude *= 180.0 / M_PI;
    }
    double longitude = instance->gps.observerLLA.lon;
    if (!isnan(longitude)) {
        longitude *= 180.0 / M_PI;
    }
    float direction = instance->gps.observerLLA.direction;
    if (!isnan(direction)) {
        direction *= 180.0 / M_PI;
    }
    float velocity = instance->gps.observerLLA.velocity;
    if (!isnan(velocity)) {
        velocity *= 3.6f;
    }

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",21,%.3f,,%.5lf,%.5lf,%.0f,,%.1f,%.1f,%.1f,%s,,,%.1f,,%.1f,%.1f,,,,,,,%.2lf",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    direction,                              /* Direction [°] */
                    velocity,                               /* Horizontal speed [km/h] */
                    instance->metro.temperature,            /* Temperature [°C] */
                    sPressure,                              /* Pressure sensor [hPa] */
                    instance->metro.humidity,
                    instance->rssi,
                    offset,                                 /* RX frequency offset [kHz] */
                    instance->realTime * 0.01
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",21,0,%d,%d,%.1f",
                instance->id,
                instance->name_id,
                instance->name_sid,
                instance->metro.illuminance
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


int windsond_crcok;

LPCLIB_Result WINDSOND_processBlock (
        WINDSOND_Handle handle,
        SONDE_Type sondeType,
        void *buffer,
        uint32_t length,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime)
{
    (void)handle;
    (void)sondeType;

    if (length != 8*sizeof(WINDSOND_Packet)) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
    handle->rxFrequencyHz = rxFrequencyHz;

    /* Remove data whitening */
    _WINDSOND_removeWhitening(buffer, length/8);

    /* Error correction */
    int numCodewords;
    if (_WINDSOND_checkFEC(buffer, &numCodewords) != LPCLIB_SUCCESS) {
        return LPCLIB_ERROR;
    }

    /* Look for valid CRC in the frame */
    if (_WINDSOND_checkCRC(buffer, 32)) {
        ++windsond_crcok;

        /* Process the payload */
        _WINDSOND_processPayload(&handle->instance, buffer, 32, rxFrequencyHz / 1e6f);  //TODO get confirmed frame length from CRC detector
        if (handle->instance) {
            handle->instance->rssi = rssi;
            handle->instance->realTime = realTime;
            _WINDSOND_sendKiss(handle->instance);
        }
    }

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result WINDSOND_resendLastPositions (WINDSOND_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    WINDSOND_InstanceData *instance = NULL;
    while (_WINDSOND_iterateInstance(&instance)) {
        _WINDSOND_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result WINDSOND_removeFromList (WINDSOND_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    WINDSOND_InstanceData *instance = NULL;
    while (_WINDSOND_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _WINDSOND_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


