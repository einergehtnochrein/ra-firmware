
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "ht03.h"
#include "ht03private.h"
#include "bch.h"

/*
 * HT03G-1U frame parameters
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
typedef struct HT03_Context {
    HT03_Packet *packet;
    HT03_InstanceData *instance;
    float rxFrequencyHz;
    int nCorrectedErrors;
} HT03_Context;

static HT03_Context _ht03;


//TODO
LPCLIB_Result HT03_open (HT03_Handle *pHandle)
{
    HT03_Handle handle = &_ht03;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _HT03_sendKiss (HT03_InstanceData *instance)
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

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",17,%.3f,,%.5lf,%.5lf,%.0f,,%.1f,%.1f,,,,,,,%.1f,,,%d,",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    direction,                              /* GPS direction [degrees] */
                    velocity,                               /* GPS velocity [km/h] */
                    SYS_getFrameRssi(sys),
                    instance->frameCounter
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


static void _HT03_sendRaw (HT03_Handle handle, HT03_Payload *payload)
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



LPCLIB_Result HT03_processBlock (
        HT03_Handle handle,
        SONDE_Type sondeType,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz)
{
    (void)sondeType;
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    if (numBits < 8*sizeof(HT03_Packet)) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    handle->packet = buffer;

    /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
    handle->rxFrequencyHz = rxFrequencyHz;

    /* CRC ok? */
    if (_HT03_checkCRC((uint8_t *)&handle->packet->payload, sizeof(HT03_Payload), handle->packet->crc_magic)) {
        _HT03_prepare(&handle->packet->payload, &handle->instance, rxFrequencyHz);
        if (handle->instance) {
            _HT03_processPayload(&handle->packet->payload, &handle->instance->gps, &handle->instance->metro);
            _HT03_sendRaw(handle, &handle->packet->payload);
            _HT03_sendKiss(handle->instance);
        }
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result HT03_resendLastPositions (HT03_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    HT03_InstanceData *instance = NULL;
    while (_HT03_iterateInstance(&instance)) {
        _HT03_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result HT03_removeFromList (HT03_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    HT03_InstanceData *instance = NULL;
    while (_HT03_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _HT03_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

