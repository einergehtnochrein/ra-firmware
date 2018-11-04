
#include <math.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "app.h"
#include "bsp.h"
#include "sys.h"
#include "pilot.h"
#include "pilotprivate.h"
#include "reedsolomon.h"
#include "gps.h"


/** Context */
typedef struct PILOT_Context {
    struct _PILOT_Payload payload;

    float rxFrequencyHz;

    PILOT_InstanceData *instance;
} PILOT_Context;


static PILOT_Context _pilot;


/* Check packet CRC */
static bool _PILOT_checkCRC (PILOT_Handle handle)
{
    (void)handle;
//TODO
    return true;
}



static void _PILOT_fromBigEndianGps (struct _PILOT_Payload *gps)
{
    gps->latitude = __REV(gps->latitude);
    gps->longitude = __REV(gps->longitude);
    gps->altitude = __REV(gps->altitude);
    gps->speedEast = __REV16(gps->speedEast);
    gps->speedNorth = __REV16(gps->speedNorth);
    gps->climbRate = __REV16(gps->climbRate);
    gps->daytime = __REV(gps->daytime);
    gps->date = __REV(gps->date);
}


//TODO
LPCLIB_Result PILOT_open (PILOT_Handle *pHandle)
{
    *pHandle = &_pilot;

    return LPCLIB_SUCCESS;
}


/* Send position as a KISS packet */
static void _PILOT_sendKiss (PILOT_InstanceData *instance)
{
    char s[100];
    int length = 0;
    float offset;

    offset = 0;

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

    length = sprintf((char *)s, "%ld,10,%.3f,,%.5lf,%.5lf,%.0f,%.1f,%.1f,%.1f,,,,,,,%.1f,%.1f,%d",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    latitude,  /* Latitude [degrees] */
                    longitude, /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    instance->gps.observerLLA.climbRate,    /* Climb rate [m/s] */
                    direction,                              /* Direction [°] */
                    velocity,                               /* Horizontal speed [km/h] */
                    SYS_getFrameRssi(sys),
                    offset,    /* RX frequency offset [kHz] */
                    0          /* # satellites */
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = sprintf(s, "%ld,10,0,%s",
                instance->id,
                instance->hashName
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


static char _pilot_raw[150];

static void _PILOT_sendRaw (PILOT_InstanceData *instance, uint8_t *buffer, uint32_t length)
{
    uint32_t n;
    int slen = 0;


    slen += snprintf(&_pilot_raw[slen], sizeof(_pilot_raw) - slen, "%ld,10,1,",
                     instance->id
                    );
    
    for (n = 0; n < length; n++) {
        slen += snprintf(&_pilot_raw[slen], sizeof(_pilot_raw) - slen, "%02X", buffer[n]);
    }

    SYS_send2Host(HOST_CHANNEL_INFO, _pilot_raw);
}



LPCLIB_Result PILOT_processBlock (PILOT_Handle handle, void *buffer, uint32_t length, float rxFrequencyHz)
{
    if (length == sizeof(struct _PILOT_Payload)) {
        memcpy(&handle->payload, buffer, sizeof(handle->payload));
        
        _PILOT_sendRaw(handle->instance, (uint8_t *)&handle->payload, sizeof(handle->payload));

        if (_PILOT_checkCRC(handle)) {
            /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
            handle->rxFrequencyHz = rxFrequencyHz;

            _PILOT_processConfigBlock(&handle->payload, &handle->instance);

            if (handle->instance) {
                handle->instance->rxFrequencyMHz = handle->rxFrequencyHz / 1e6f;

                _PILOT_fromBigEndianGps(&handle->payload);
                if (_PILOT_processGpsBlock(&handle->payload, &handle->instance->gps) == LPCLIB_SUCCESS) {
                    _PILOT_sendKiss(handle->instance);
                }
            }
        }
    }

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result PILOT_resendLastPositions (PILOT_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    PILOT_InstanceData *instance = NULL;
    while (_PILOT_iterateInstance(&instance)) {
        _PILOT_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result PILOT_removeFromList (PILOT_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    PILOT_InstanceData *instance = NULL;
    while (_PILOT_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _PILOT_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

