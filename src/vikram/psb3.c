
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "psb3.h"
#include "psb3private.h"
#include "bch.h"

/*
 * PS-B3 frame parameters
 *
 * 2FSK, 768 sym/s Manchester encoded, 384 bit/s, 5.0 kHz deviation
 * Continuous transmission (no preamble)
 * Sync word: 11111001 10100100 00101011 10110001 (F9 A4 2B B1)
 *
 * Packet length: 4 sync bytes + 44 data bytes = 48 bytes (--> 384 bits, frame duration = one second)
 * Packet structure (all bytes received MSB first):
 *
 * +---------+-------+----------+---------+
 * | Part 1  | CRC16 | Part 2   | Parity  |
 * | 6 bytes | 2 b.  | 27 bytes | 9 bytes |
 * +---------+-------+----------+---------+
 * \-------------- Codeword --------------/
 * \------------ CRC -----------/
 *
 * Structure of the codeword unknown. According to public documentation the sonde uses a Reed Solomon
 * code, but no luck with various parameter combinations :-(
 * However, a function to validate the codeword has been found (no code correction capability yet)
 *
 * Regular CRC:
 * CRC parameters: polynomial=0x8005, seed=0, LSB first, output-XOR=0
 * CRC is sent in big-endian format
 * NOTE: For CRC calculation the two byte CRC field is set to zero!
 */


/** Context */
typedef struct PSB3_Context {
    PSB3_Packet packet;
    PSB3_InstanceData *instance;
    float rxFrequencyHz;
} PSB3_Context;

static PSB3_Context _psb3;


//TODO
LPCLIB_Result PSB3_open (PSB3_Handle *pHandle)
{
    PSB3_Handle handle = &_psb3;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _PSB3_sendKiss (PSB3_InstanceData *instance)
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
    float kmh = instance->gps.observerLLA.velocity;
    if (!isnan(kmh)) {
        kmh *= 3.6f;
    }

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",15,%.3f,%d,%.5lf,%.5lf,%.0f,,%.1f,%.1f,%.1f,,,,%.1f,,%.1f,,,%d",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    instance->gps.usedSats,
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    direction,                              /* Course [°] */
                    kmh,                                    /* Speed [km/h] */
                    instance->metro.temperature,            /* Temperature [°C] */
                    instance->metro.humidity,               /* Relative humidity [%] */
                    SYS_getFrameRssi(sys),
                    instance->frameCounter
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",15,0,%s,%.1f",
                instance->id,
                instance->name,
                instance->gps.pdop
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


static void _PSB3_sendRaw (PSB3_Handle handle, PSB3_Packet *p1)
{
(void)handle;
(void)p1;
}



LPCLIB_Result PSB3_processBlock (
        PSB3_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz)
{
    (void)rxFrequencyHz;
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    if (numBits == 8*sizeof(PSB3_Packet)) {
        memcpy(&handle->packet, buffer, sizeof(handle->packet));

        /* Validate codeword */
        if (_PSB3_checkParity((uint8_t *)&handle->packet, sizeof(handle->packet))) {
            /* Probably a valid codeword. Check CRC!.
             * CRC is transmitted amid the data block. Read it and set these locations to zero
             * before CRC calculation.
             */
            uint16_t receivedCRC = __REV16(handle->packet.crc);
            handle->packet.crc = 0;

            /* Validate CRC16: Polynomial 0x8005, seed=0, output-xor=0 */
            if (_PSB3_checkCRC((uint8_t *)&handle->packet.rawData, 35, receivedCRC)) {
                _PSB3_sendRaw(handle, buffer);

                result = _PSB3_prepare(&handle->packet, &handle->instance, rxFrequencyHz);
                if (result == LPCLIB_SUCCESS) {
                    _PSB3_processPayload(&handle->packet, handle->instance);
                    if (result == LPCLIB_SUCCESS) {
                        _PSB3_sendKiss(handle->instance);
                    }
                }
            }
        }
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result PSB3_resendLastPositions (PSB3_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    PSB3_InstanceData *instance = NULL;
    while (_PSB3_iterateInstance(&instance)) {
        _PSB3_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result PSB3_removeFromList (PSB3_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    PSB3_InstanceData *instance = NULL;
    while (_PSB3_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _PSB3_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

