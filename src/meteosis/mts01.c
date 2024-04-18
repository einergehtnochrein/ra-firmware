
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "mts01.h"
#include "mts01private.h"
#include "bch.h"

/*
 * MTS-01 frame parameters
 *
 * 2FSK, 1200 sym/s, 1200 bit/s, ? kHz deviation
 * One burst every two seconds.
 * Sync word: 10110100 00101011 10000000 (B4 2B 80)
 *
 * Packet length: 3 sync bytes + 128 data bytes + 2 CRC bytes = 133 bytes (--> 1064 bits, frame duration = 887 ms)
 * Packet structure (all bytes received MSB first):
 *
 * +-----------+-------+
 * | Payload   | CRC16 |
 * | 128 bytes | 2 b.  |
 * +-----------+-------+
 *
 * Regular CRC:  TODO
 * CRC parameters: polynomial=0x8005, seed=0, LSB first, output-XOR=0
 * CRC is sent in big-endian format
 */


/** Context */
typedef struct MTS01_Context {
    MTS01_Packet packet;
    MTS01_InstanceData *instance;
    float rxFrequencyHz;
} MTS01_Context;

static MTS01_Context _mts01;


//TODO
LPCLIB_Result MTS01_open (MTS01_Handle *pHandle)
{
    MTS01_Handle handle = &_mts01;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _MTS01_sendKiss (MTS01_InstanceData *instance)
{
    static char s[160];
    int length = 0;


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

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",22,%.3f,%d,%.5lf,%.5lf,%.0f,,%.1f,%.1f,%.1f,,,,%.1f,,%.1f,,,%d,,,,,%.2lf",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    instance->gps.usedSats,
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    direction,                              /* Direction [°] */
                    velocity,                               /* [km/h] */
                    instance->metro.temperature,
                    instance->metro.humidity,               /* Relative humidity [%] */
                    instance->rssi,
                    instance->frameCounter,
                    instance->realTime * 0.01
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",22,0,%s,%.1f",
                instance->id,
                instance->name,
                instance->metro.innerTemperature
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


static void _MTS01_sendRaw (MTS01_Handle handle, MTS01_Packet *p1)
{
(void)handle;
(void)p1;
}



LPCLIB_Result MTS01_processBlock (
        MTS01_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime)
{
    (void)rxFrequencyHz;
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    if (numBits == 8*sizeof(MTS01_Packet)) {
        memcpy(&handle->packet, buffer, sizeof(handle->packet));

        uint16_t receivedCRC = handle->packet.rawData.crc;

        /* Validate CRC16: Polynomial 0x8005, seed=0xFFFF, output-xor=0 */
        if (_MTS01_checkCRC((uint8_t *)&handle->packet.rawData, 128, receivedCRC)) {
            _MTS01_sendRaw(handle, buffer);

            result = _MTS01_prepare(&handle->packet, &handle->instance, rxFrequencyHz);
            if (result == LPCLIB_SUCCESS) {
                handle->instance->rssi = rssi;
                handle->instance->realTime = realTime;
                _MTS01_processPayload(&handle->packet, handle->instance);
                if (result == LPCLIB_SUCCESS) {
                    _MTS01_sendKiss(handle->instance);
                }
            }
        }
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result MTS01_resendLastPositions (MTS01_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    MTS01_InstanceData *instance = NULL;
    while (_MTS01_iterateInstance(&instance)) {
        _MTS01_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result MTS01_removeFromList (MTS01_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    MTS01_InstanceData *instance = NULL;
    while (_MTS01_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _MTS01_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

