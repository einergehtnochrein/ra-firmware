
#include <inttypes.h>
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
#include "m20.h"
#include "m20private.h"
#include "reedsolomon.h"
#include "gps.h"


/** Context */
typedef struct M20_Context {
    int packetLength;
    M20_Packet packet;

    float rxFrequencyHz;

    M20_InstanceData *instance;
} M20_Context;


static M20_Context _m20;


static const uint8_t manchester2bin[256] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x07,0x06,0x04,0x05,0x00,0x00,
    0x00,0x00,0x01,0x00,0x02,0x03,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x0B,0x0A,0x08,0x09,0x00,0x00,
    0x00,0x00,0x0D,0x0C,0x0E,0x0F,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x0F,0x0E,0x0C,0x0D,0x00,0x00,
    0x00,0x00,0x09,0x08,0x0A,0x0B,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x03,0x02,0x00,0x01,0x00,0x00,
    0x00,0x00,0x05,0x04,0x06,0x07,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};


/* Manchester decoding */
static void _M20_buffer2raw (M20_Handle handle, uint8_t *buffer, uint32_t length)
{
    uint32_t i;
    uint8_t *p = (uint8_t *)&handle->packet;

    for (i = 0; i < length; i++) {
        p[i] =
           (manchester2bin[buffer[2*i+0]] << 4)
         | (manchester2bin[buffer[2*i+1]] << 0);
    }
}


/* Process one byte for CRC.
 * CRC method as published by GitHub project RS: https://github.com/rs1729/RS
 */
static uint16_t _M20_updateCRC (uint8_t data, uint16_t crc)
{
    uint8_t b;

    b = (data >> 1) ^ (data << 7);
    b ^= (b >> 2);
    b ^= (crc >> 7) ^ ((crc >> 9) & 0x3F);
    crc = (crc << 8)
        ^ b
        ^ ((crc << 6) & 0xC0)
        ^ ((crc << 4) & 0xC0)
        ^ ((crc << 2) & 0xC0)
        ^ (crc & 0x3F)
        ;

    return crc;
}



/* Check packet CRC */
static _Bool _M20_checkCRC (const uint8_t *p, int length, uint16_t expectedCRC)
{
    int i;
    uint16_t crc = 0;

    /* Always start CRC calculation with length of following data block */
    crc = _M20_updateCRC(length, crc);

    for (i = 0; i < length - 2; i++) {
        crc = _M20_updateCRC(p[i], crc);
    }

    return (crc == expectedCRC);
}



static void _M20_fromBigEndian (struct _M20_Payload *data)
{
    data->inner.speedE = __REV16(data->inner.speedE);
    data->inner.speedN = __REV16(data->inner.speedN);
    data->climbRate = __REV16(data->climbRate);
    data->week = __REV16(data->week);
    data->latitude = __REV(data->latitude);
    data->longitude = __REV(data->longitude);
}


//TODO
LPCLIB_Result M20_open (M20_Handle *pHandle)
{
    *pHandle = &_m20;

    return LPCLIB_SUCCESS;
}



/* Send position as a KISS packet */
static void _M20_sendKiss (M20_InstanceData *instance)
{
    char s[100];
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

    length = sprintf((char *)s, "%"PRIu32",13,%.3f,,%.5lf,%.5lf,%.0f,%.1f,%.1f,%.1f,%.1f,,,,,,%.1f,,,,,,0",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    instance->gps.observerLLA.climbRate,    /* Climb rate [m/s] */
                    direction,                              /* Direction [degrees} */
                    velocity,                               /* Velocity [km/h] */
                    instance->metro.temperature,            /* Temperature [°C] */
                    SYS_getFrameRssi(sys)
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = sprintf(s, "%"PRIu32",13,0,%s",
                instance->id,
                instance->hashName
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}



static char _m20_raw[300];    //TODO

static void _M20_sendRaw (M20_InstanceData *instance, uint8_t *buffer, uint32_t length)
{
    uint32_t n;
    int slen = 0;


    slen += snprintf(&_m20_raw[slen], sizeof(_m20_raw) - slen, "%"PRIu32",13,1,,",
                     instance->id
                    );

    for (n = 0; n < length; n++) {
        slen += snprintf(&_m20_raw[slen], sizeof(_m20_raw) - slen, "%02X", buffer[n]);
    }

    SYS_send2Host(HOST_CHANNEL_INFO, _m20_raw);
}



LPCLIB_Result M20_processBlock (M20_Handle handle, void *buffer, uint32_t numBits, float rxFrequencyHz)
{
    if (numBits == 70*2*8) {
        handle->packetLength = (numBits/8) / 2;

        /* Convert to byte array */
        _M20_buffer2raw(handle, buffer, handle->packetLength);

        /* There are two CRC's: One for the whole packet, one for an inner block only. */
        volatile _Bool innercrc;
        innercrc = _M20_checkCRC(
                    (uint8_t *)&handle->packet.packet69.inner,
                    sizeof(handle->packet.packet69.inner),
                    __REV16(handle->packet.packet69.inner.crc));
        volatile _Bool outercrc;
        outercrc = _M20_checkCRC(
                    (uint8_t *)&handle->packet.packet69,
                    sizeof(handle->packet.packet69),
                    __REV16(handle->packet.packet69.crc));

        /* At least the inner CRC must be correct */
        if (innercrc) {
            /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
            handle->rxFrequencyHz = rxFrequencyHz;

if(1){//            if (handle->instance->logMode == M20_LOGMODE_RAW) {
                _M20_sendRaw(handle->instance, (uint8_t *)&handle->packet.packet69, handle->packetLength);
            }

            /* Convert big-endian fields to little-endian */
            _M20_fromBigEndian(&handle->packet.packet69);

            /* Get an instance */
            _M20_processConfigBlock(&handle->packet.packet69, &handle->instance);
            if (handle->instance) {
                handle->instance->rxFrequencyMHz = handle->rxFrequencyHz / 1e6f;

                /* Process the inner data block */
                if (_M20_processPayloadInner(&handle->packet.packet69.inner,
                                            &handle->instance->gps,
                                            &handle->instance->metro) == LPCLIB_SUCCESS) {
                    /* Process the outer data block (if CRC is correct) */
                    _M20_processPayload(&handle->packet.packet69,
                                        outercrc,
                                        &handle->instance->gps);
                }

                _M20_sendKiss(handle->instance);
            }
        }
    }

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result M20_resendLastPositions (M20_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    M20_InstanceData *instance = NULL;
    while (_M20_iterateInstance(&instance)) {
        _M20_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result M20_removeFromList (M20_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    M20_InstanceData *instance = NULL;
    while (_M20_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _M20_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

