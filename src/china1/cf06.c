
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "cf06.h"
#include "cf06private.h"
#include "bch.h"

/*
 * CF-06-AH frame parameters
 *
 * 2FSK, 2400 bit/s, 2.4 kHz deviation
 * Preamble: 36x  01
 * Sync word: 10110100 00101011 (B4 2B)
 *
 * Packet length: 100 bytes (we receive 102 bytes for compatibility with HT03 sonde)
 * Packet structure (all bytes received LSB first):
 *
 * +----------------+ +---------+------+------------+ +---------+------+------------+
 * |                | | Block 1 |      | RS Parity  | | Block 2 |      | RS Parity  |
 * | 63 7F FF AA AA | | 40 bytes| CRC1 | 6 bytes    | | 39 bytes| CRC2 | 6 bytes    |
 * |                | |         |      | Codeword 1 | |         |      | Codeword 2 |
 * +----------------+ +---------+------+------------+ +---------+------+------------+
 *                    \-------- Codeword 1 ---------/
 *                    \---------------------- Codeword 2 ---------------------------/
 *
 * Reed-Solomon code:  shortened RS(255,249), symbols from GF(256) with primitive polynomial 0x11D
 *                     and generator element 2. RS generator polynomial g = (x - a^1)*...*(x - a^6)
 *                     NOTE: This is different from RS41 Reed Solomon code, where the generator
 *                           polynomial starts at alpha^0: g_rs41 = (x - a^0)*...*(x - a^23)
 *
 *                     Codeword 1 is constructed as follows:
 *                     c_0...c_254 (c_0...c_248 = k data symbols, c_249...c_254 = m parity symbols)
 *                     Codeword 1 is transmitted in reverse symbol order: c_41...c_0+c_254...c_249
 *                     (first byte of block 1 is c_41, last byte of CRC1 is c_0).
 *                     Codeword 2 is formed accordingly.
 *
 * Regular CRC:
 * CRC1 parameters: polynomial=0x1021, seed=0, LSB first, output-XOR=0
 * CRC2 parameters: polynomial=0x1021, seed=0, LSB first, output-XOR=0
 * Both CRC1 and CRC2 are sent in big-endian format
 */



/** Context */
typedef struct CF06_Context {
    CF06_Packet *pRawData;
    CF06_InstanceData *instance;
    float rxFrequencyHz;
    int nCorrectedErrors;
} CF06_Context;

static CF06_Context _cf06;


//TODO
LPCLIB_Result CF06_open (CF06_Handle *pHandle)
{
    CF06_Handle handle = &_cf06;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _CF06_sendKiss (CF06_InstanceData *instance)
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

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",16,%.3f,,%.5lf,%.5lf,%.0f,,%.1f,%.1f,,,,,,,%.1f,,,%d,",
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

    length = snprintf(s, sizeof(s), "%"PRIu32",16,0,%s",
                instance->id,
                instance->name
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


static void _CF06_sendRaw (CF06_Handle handle, CF06_Packet *p1)
{
#if 0
    char s[200];

    snprintf(s, sizeof(s),
                     "%"PRIu32",16,1,"
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



LPCLIB_Result CF06_processBlock (
        CF06_Handle handle,
        SONDE_Type sondeType,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz)
{
    (void)sondeType;
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    if (numBits < 8*sizeof(CF06_Packet)) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    handle->pRawData = buffer;

    /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
    handle->rxFrequencyHz = rxFrequencyHz;

    /* Error correction for outer block */
    int errorsOuter = 0;
    _Bool frameOk = false;
    if (_CF06_checkReedSolomonOuter (handle->pRawData->rawData.dat8, &errorsOuter) == LPCLIB_SUCCESS) {
        /* CRC of outer block ok? */
        handle->pRawData->block2.crc = __REV16(handle->pRawData->block2.crc); /* Big endian */
        if (_CF06_checkCRCOuter((uint8_t *)&handle->pRawData->block2, sizeof(CF06_PayloadBlock2)-2, handle->pRawData->block2.crc)) {
            frameOk = true;
            _CF06_prepare(&handle->pRawData->block1, &handle->instance, rxFrequencyHz);
            if (handle->instance) {
                _CF06_processPayloadBlock2(&handle->pRawData->block2, &handle->instance->gps, &handle->instance->metro);
            }
        }
    }

    /* Error correction for inner block */
    int errorsInner = 0;
    if (_CF06_checkReedSolomonInner (handle->pRawData->rawData.dat8, &errorsInner) == LPCLIB_SUCCESS) {
        /* CRC of inner block ok? */
        handle->pRawData->block1.crc = __REV16(handle->pRawData->block1.crc); /* Big endian */
        if (_CF06_checkCRCInner((uint8_t *)&handle->pRawData->block1, sizeof(CF06_PayloadBlock1)-2, handle->pRawData->block1.crc)) {
            if (handle->instance) {
                _CF06_processPayloadBlock1(&handle->pRawData->block1, &handle->instance->gps, &handle->instance->metro);
            }
        }
    }

    handle->nCorrectedErrors = errorsInner + errorsOuter;
    if (frameOk && (handle->instance != NULL)) {
        _CF06_sendRaw(handle, handle->pRawData);
        _CF06_sendKiss(handle->instance);
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result CF06_resendLastPositions (CF06_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    CF06_InstanceData *instance = NULL;
    while (_CF06_iterateInstance(&instance)) {
        _CF06_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result CF06_removeFromList (CF06_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    CF06_InstanceData *instance = NULL;
    while (_CF06_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _CF06_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

