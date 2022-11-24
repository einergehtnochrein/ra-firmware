
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "gth3.h"
#include "gth3private.h"
#include "bch.h"

/*
 * GTH3 frame parameters
 *
 * 2FSK, 2400 bit/s, 2.4 kHz deviation
 * Preamble: 80x  01
 * Sync word: 10110100 00101011 (B4 2B)
 *
 * Packet length: 102 bytes
 * Packet structure:
 *
 * +----------------+ +----------+-----+ +--------+
 * |                | | Payload  |     | | CRC    |
 * | 63 7F FF FF EE | | 93 bytes | CRC | | Magic  |
 * |                | |          |     | |        |
 * +----------------+ +----------+-----+ +--------+
 *                                  |        |
 *      \-----------------------/ <-+        |
 *   \---------------------------------/ <---+
 *
 * Regular CRC
 * CRC parameters: polynomial=0x1021, seed=0, LSB first, output-XOR=0
 * While CRC is a variable value, CRC_magic is always the same (0xEB87)
 * Both CRC and CRC_magic are sent in little-endian format
 */



/** Context */
typedef struct GTH3_Context {
    GTH3_Packet *packet;
    GTH3_InstanceData *instance;
    float rxFrequencyHz;
    int nCorrectedErrors;
} GTH3_Context;

static GTH3_Context _gth3;


//TODO
LPCLIB_Result GTH3_open (GTH3_Handle *pHandle)
{
    GTH3_Handle handle = &_gth3;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _GTH3_sendKiss (GTH3_InstanceData *instance)
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

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",17,%.3f,,%.5lf,%.5lf,%.0f,,%.1f,%.1f,,,,,,,%.1f,,,%d,,,,,%"PRIu64,
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    direction,                              /* GPS direction [degrees] */
                    velocity,                               /* GPS velocity [km/h] */
                    SYS_getFrameRssi(sys),
                    instance->frameCounter,
                    instance->realTime
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",17,0,%s",
                instance->id,
                instance->name
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


static void _GTH3_sendRaw (GTH3_Handle handle, GTH3_Payload *payload)
{
(void)handle;
(void)payload;
#if 0
    char s[200];

    snprintf(s, sizeof(s),
                     "%"PRIu32",17,1,"
                     "%08"PRIX32"%08"PRIX32"%08"PRIX32"%08"PRIX32"%08"PRIX32
                     "%08"PRIX32"%08"PRIX32"%08"PRIX32"%08"PRIX32"%08"PRIX32,
                     handle->instance->id,
                     __REV(p1->rawData.dat32[0]),
                     __REV(p1->rawData.dat32[1]),
                     __REV(p1->rawData.dat32[2]),
                     __REV(p1->rawData.dat32[3]),
                     __REV(p1->rawData.dat32[4]),
                     __REV(p1->rawData.dat32[5]),
                     __REV(p1->rawData.dat32[6]),
                     __REV(p1->rawData.dat32[7]),
                     __REV(p1->rawData.dat32[8]),
                     __REV(p1->rawData.dat32[9])
                    );

    SYS_send2Host(HOST_CHANNEL_INFO, s);
#endif
}



LPCLIB_Result GTH3_processBlock (
        GTH3_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime)
{
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    if (numBits < 8*sizeof(GTH3_Packet)) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    handle->packet = buffer;

    /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
    handle->rxFrequencyHz = rxFrequencyHz;

    /* CRC ok? */
    if (_GTH3_checkCRC((uint8_t *)&handle->packet->payload, sizeof(GTH3_Payload), handle->packet->crc_magic)) {
        _GTH3_prepare(&handle->packet->payload, &handle->instance, rxFrequencyHz);
        if (handle->instance) {
            handle->instance->rssi = rssi;
            handle->instance->realTime = realTime;
            _GTH3_processPayload(&handle->packet->payload, &handle->instance->gps, &handle->instance->metro);
            _GTH3_sendRaw(handle, &handle->packet->payload);
            _GTH3_sendKiss(handle->instance);
        }
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result GTH3_resendLastPositions (GTH3_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    GTH3_InstanceData *instance = NULL;
    while (_GTH3_iterateInstance(&instance)) {
        _GTH3_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result GTH3_removeFromList (GTH3_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    GTH3_InstanceData *instance = NULL;
    while (_GTH3_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _GTH3_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

