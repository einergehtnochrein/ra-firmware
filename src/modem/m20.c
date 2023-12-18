
#include <inttypes.h>
#include <math.h>
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
    uint32_t packetLength;
    union {
        M20_Packet packet;
        uint8_t rawPacket[256];     /* Room for maximum length of XDATA packet */
    };

    float rxFrequencyHz;

    M20_InstanceData *instance;
} M20_Context;


static M20_Context _m20;


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



static void _M20_fromBigEndian (M20_Packet *data)
{
    data->inner.speedE = __REV16(data->inner.speedE);
    data->inner.speedN = __REV16(data->inner.speedN);
    data->climbRate = __REV16(data->climbRate);
    data->week = __REV16(data->week);
    data->latitude = __REV(data->latitude);
    data->longitude = __REV(data->longitude);
}



static void _M20_processXdata (char *p, int length, M20_InstanceData *instance)
{
    /* XDATA of minimum two bytes is accepted. */
    if (length >= 2) {
        /* Send raw data to app */
        static char s[240];
        if (snprintf(s, sizeof(s), "%"PRIu32",13,2,%.*s", instance->id, length, p) > 0) {
            SYS_send2Host(HOST_CHANNEL_INFO, s);
        }
    }
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
    char s[160];
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

    length = snprintf((char *)s, sizeof(s),
                "%"PRIu32",13,%.3f,,%.5lf,%.5lf,%.0f,%.1f,%.1f,%.1f,%.1f,%.1f,,,%.1f,,%.1f,,,,,%.1f,%.1f,,%.2lf",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    instance->gps.observerLLA.climbRate,    /* Climb rate [m/s] */
                    direction,                              /* Direction [degrees} */
                    velocity,                               /* Velocity [km/h] */
//                    instance->metro.T,                      /* Temperature main sensor [°C] */
                instance->metro.TU,                      /* Temperature humidity sensor [°C] (main sensor calibration broken) */
                    instance->metro.pressure,               /* Pressure [hPa] */
                    instance->metro.humidity,               /* Relative humidity [%] */
                    instance->rssi,
                    instance->metro.batteryVoltage,         /* Sonde battery voltage [V] */
                    instance->metro.cpuTemperature,         /* CPU temperature [°C] */
                    instance->realTime * 0.01
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",13,0,%s,,,%.1f,%.1f",
                instance->id,
                instance->hashName,
                instance->metro.TU,                         /* Temperature humidity sensor [°C] */
                instance->metro.boardTemperature
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



LPCLIB_Result M20_processBlock (
        M20_Handle handle,
        uint8_t *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime)
{
    handle->packetLength = numBits / 8 - 1; /* -1: ignore the length byte */
    if (handle->packetLength >= sizeof(M20_Packet)) { /* Must have at least minimum packet length */
        memcpy(&handle->packet, &buffer[1], handle->packetLength); /* Skip length byte in buffer[0] */

        /* There are two CRC's: One for the whole packet, one for an inner block only. */
        volatile _Bool innercrc;
        innercrc = _M20_checkCRC(
                    (uint8_t *)&handle->packet.inner,
                    sizeof(handle->packet.inner),
                    __REV16(handle->packet.inner.crc));
        volatile _Bool outercrc;
        uint16_t receivedOuterCrc =
                256 * buffer[1 + handle->packetLength - 2] + buffer[1 + handle->packetLength - 1];
        outercrc = _M20_checkCRC(
                    (uint8_t *)&handle->packet,
                    sizeof(handle->packet),
                    receivedOuterCrc);
        if (outercrc) {
            innercrc = true;    // Firmware 6 transmits an invalid inner CRC (always 0)!
        }

        /* At least the inner CRC must be correct */
        if (innercrc) {
            /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
            handle->rxFrequencyHz = rxFrequencyHz;

if(1){//            if (handle->instance->logMode == M20_LOGMODE_RAW) {
                _M20_sendRaw(handle->instance, (uint8_t *)&handle->packet, handle->packetLength);
            }

            /* Convert big-endian fields to little-endian */
            _M20_fromBigEndian(&handle->packet);

            /* Get an instance */
            _M20_processConfigBlock(&handle->packet, &handle->instance);
            if (handle->instance) {
                handle->instance->rssi = rssi;
                handle->instance->realTime = realTime;
                handle->instance->rxFrequencyMHz = handle->rxFrequencyHz / 1e6f;

                /* Process the inner data block */
                if (_M20_processPayloadInner(&handle->packet.inner,
                                            &handle->instance->gps,
                                            &handle->instance->metro) == LPCLIB_SUCCESS) {
                    /* Process the outer data block (if CRC is correct) */
                    _M20_processPayload(&handle->packet,
                                        outercrc,
                                        &handle->instance->gps,
                                        &handle->instance->metro);

                    /* XDATA */
                    _M20_processXdata(&handle->packet.xdata, handle->packet.xdataLength, handle->instance);
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

