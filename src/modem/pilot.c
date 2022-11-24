
#include <inttypes.h>
#include <math.h>
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
    bool result = false;
    CRC_Handle crc = LPCLIB_INVALID_HANDLE;
    CRC_Mode crcMode = CRC_makeMode(
        CRC_POLY_CRC16,
        CRC_DATAORDER_REVERSE,
        CRC_SUMORDER_REVERSE,
        CRC_DATAPOLARITY_NORMAL,
        CRC_SUMPOLARITY_NORMAL);
    uint8_t syncword[] = {0xAA, 0xAA, 0xAA, 0x01};

    if (CRC_open(crcMode, &crc) == LPCLIB_SUCCESS) {
        CRC_seed(crc, 0xFFFF);
        /* Strange enough the CRC includes the sync word! */
        CRC_write(crc, syncword, sizeof(syncword), NULL, NULL);
        /* Process payload (length excludes the 16-bit CRC) */
        CRC_write(crc, &handle->payload, sizeof(handle->payload) - 2, NULL, NULL);
        /* CRC is transmitted in big endian format */
        result = CRC_read(crc) == __REV16(handle->payload.crc);

        CRC_close(&crc);
    }

    return result;
}


static void _PILOT_fromBigEndianGps (struct _PILOT_Payload *payload)
{
    payload->latitude = __REV(payload->latitude);
    payload->longitude = __REV(payload->longitude);
    payload->altitude = __REV(payload->altitude);
    payload->speedEast = __REV16(payload->speedEast);
    payload->speedNorth = __REV16(payload->speedNorth);
    payload->climbRate = __REV16(payload->climbRate);
    payload->daytime = __REV(payload->daytime);
    payload->date = __REV(payload->date);
    payload->hdop = __REV16(payload->hdop);
    payload->vdop = __REV16(payload->vdop);
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

    length = sprintf((char *)s, "%"PRIu32",10,%.3f,%d,%.5lf,%.5lf,%.0f,%.1f,%.1f,%.1f,,,,,,%.2f,%.1f,%.1f,%d,,,,,,%"PRIu64,
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                     instance->gps.usedSats,
                    latitude,  /* Latitude [degrees] */
                    longitude, /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    instance->gps.observerLLA.climbRate,    /* Climb rate [m/s] */
                    direction,                              /* Direction [°] */
                    velocity,                               /* Horizontal speed [km/h] */
                    instance->gps.hdop,
                    instance->rssi,
                    offset,    /* RX frequency offset [kHz] */
                    instance->gps.visibleSats,              /* # satellites */
                    instance->realTime
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = sprintf(s, "%"PRIu32",10,0,%s",
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


    slen += snprintf(&_pilot_raw[slen], sizeof(_pilot_raw) - slen, "%"PRIu32",10,1,",
                     instance->id
                    );
    
    for (n = 0; n < length; n++) {
        slen += snprintf(&_pilot_raw[slen], sizeof(_pilot_raw) - slen, "%02X", buffer[n]);
    }

    SYS_send2Host(HOST_CHANNEL_INFO, _pilot_raw);
}



LPCLIB_Result PILOT_processBlock (
        PILOT_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime)
{
    if (numBits == 8*sizeof(struct _PILOT_Payload)) {
        memcpy(&handle->payload, buffer, sizeof(handle->payload));
        
        _PILOT_sendRaw(handle->instance, (uint8_t *)&handle->payload, sizeof(handle->payload));

        /* Process packet only if CRC is correct */
        if (_PILOT_checkCRC(handle)) {
            /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
            handle->rxFrequencyHz = rxFrequencyHz;

            _PILOT_processConfigBlock(&handle->payload, &handle->instance);

            if (handle->instance) {
                handle->instance->rssi = rssi;
                handle->instance->realTime = realTime;
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

