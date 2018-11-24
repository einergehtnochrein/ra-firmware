
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
#include "m10.h"
#include "m10private.h"
#include "reedsolomon.h"
#include "gps.h"


/** Context */
typedef struct M10_Context {
    int packetLength;
    M10_Packet packet;

    float rxFrequencyHz;

    M10_InstanceData *instance;
} M10_Context;


static M10_Context _m10;


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
static void _M10_buffer2raw (M10_Handle handle, uint8_t *buffer, uint32_t length)
{
    uint32_t i;
    uint8_t *p = (uint8_t *)&handle->packet;

    for (i = 0; i < length; i++) {
        p[i] =
           (manchester2bin[buffer[2*i+0]] << 4)
         | (manchester2bin[buffer[2*i+1]] << 0);
    }
}


/* Check packet CRC */
static bool _M10_checkCRC (M10_Handle handle)
{
    int i;
    uint16_t crc = 0;
    uint8_t *p = (uint8_t *)&handle->packet;
    int payloadLength = 1 + sizeof(handle->packet.packet100) - 2;
    uint8_t b;

    for (i = 0; i < payloadLength; i++) {
        b = (p[i] >> 1) ^ (p[i] << 7);
        b ^= (b >> 2);
        b ^= (crc >> 7) ^ ((crc >> 9) & 0x3F);
        crc = (crc << 8)
            ^ b
            ^ ((crc << 6) & 0xC0)
            ^ ((crc << 4) & 0xC0)
            ^ ((crc << 2) & 0xC0)
            ^ (crc & 0x3F)
            ;
    }

    return crc == __REV16(handle->packet.packet100.crc);
}



static void _M10_fromBigEndianGps (struct _M10_GpsBlock *gps)
{
    gps->speedEast = __REV16(gps->speedEast);
    gps->speedNorth = __REV16(gps->speedNorth);
    gps->speedVertical = __REV16(gps->speedVertical);
    gps->tow = __REV(gps->tow);
    gps->latitude = __REV(gps->latitude);
    gps->longitude = __REV(gps->longitude);
    gps->altitude = __REV(gps->altitude);
//                    uint8_t reserved[6];
    gps->week = __REV16(gps->week);
}


//TODO
LPCLIB_Result M10_open (M10_Handle *pHandle)
{
    *pHandle = &_m10;

    return LPCLIB_SUCCESS;
}



/* Send position as a KISS packet */
static void _M10_sendKiss (M10_InstanceData *instance)
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

    length = sprintf((char *)s, "%ld,7,%.3f,,%.5lf,%.5lf,%.0f,%.1f,%.1f,%.1f,%.1f,,,,,,%.1f,%.1f,%d,,,%.3f",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    instance->gps.observerLLA.climbRate,    /* Climb rate [m/s] */
                    direction,                              /* Direction [degrees} */
                    velocity,                               /* Velocity [km/h] */
                    instance->metro.temperature,            /* Temperature [°C] */
                    SYS_getFrameRssi(sys),
                    offset,    /* RX frequency offset [kHz] */
                    instance->gps.visibleSats,              /* # satellites */
                    instance->metro.batteryVoltage
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = sprintf(s, "%ld,7,0,%s",
                instance->id,
                instance->hashName
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}



static char _m10_raw[300];    //TODO

static void _M10_sendRaw (M10_InstanceData *instance, uint8_t *buffer, uint32_t length)
{
    uint32_t n;
    int slen = 0;


    slen += snprintf(&_m10_raw[slen], sizeof(_m10_raw) - slen, "%ld,7,1,,",
                     instance->id
                    );

    for (n = 0; n < length; n++) {
        slen += snprintf(&_m10_raw[slen], sizeof(_m10_raw) - slen, "%02X", buffer[n]);
    }

    SYS_send2Host(HOST_CHANNEL_INFO, _m10_raw);
}



LPCLIB_Result M10_processBlock (M10_Handle handle, void *buffer, uint32_t length, float rxFrequencyHz)
{
    if (length == (100+1)*2) {
        handle->packetLength = length / 2;

        /* Convert to byte array */
        _M10_buffer2raw(handle, buffer, handle->packetLength);

        if (_M10_checkCRC(handle)) {
            /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
            handle->rxFrequencyHz = rxFrequencyHz;

            if (handle->packet.packet100.packetType == 0x9F) {
                _M10_processConfigBlock(&handle->packet.packet100.data.config, &handle->instance);

                if (handle->instance) {
                    handle->instance->rxFrequencyMHz = handle->rxFrequencyHz / 1e6f;

if(1){//                    if (handle->instance->logMode == M10_LOGMODE_RAW) {
                        _M10_sendRaw(handle->instance, (uint8_t *)&handle->packet.packet100, handle->packetLength);
                    }

                    _M10_fromBigEndianGps(&handle->packet.packet100.data.gps);
                    _M10_processGpsBlock(&handle->packet.packet100.data.gps, &handle->instance->gps);
                    _M10_processMetrologyBlock(&handle->packet.packet100.data.config, &handle->instance->metro);
                    _M10_sendKiss(handle->instance);
                }
            }
        }
    }

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result M10_resendLastPositions (M10_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    M10_InstanceData *instance = NULL;
    while (_M10_iterateInstance(&instance)) {
        _M10_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result M10_removeFromList (M10_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    M10_InstanceData *instance = NULL;
    while (_M10_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _M10_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

