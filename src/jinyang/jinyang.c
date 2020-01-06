
#include <inttypes.h>
#include <math.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "jinyang.h"
#include "jinyangprivate.h"
#include "bch.h"


/** Context */
typedef struct JINYANG_Context {
    JINYANG_Packet packet;
    JINYANG_InstanceData *instance;
    float rxFrequencyHz;
uint32_t rawlat;
uint32_t rawlon;
uint32_t rawalt;
} JINYANG_Context;

static JINYANG_Context _jinyang;


static const uint8_t whitening[40] = {
    0x00,0x00,0x00,0x00,0x00,0x85,0x01,0x00,
    0x00,0x79,0xD2,0x39,0x70,0x97,0x57,0x0A,
    0x54,0x7D,0x2E,0x00,0x6D,0x0D,0x00,0x80,
    0x67,0x59,0xC7,0xA2,0x00,0x34,0xCA,0x18,
    0x30,0x53,0x93,0xDF,0x6E,0xB9,0x00,0x00,
};


/* Do bit reversal, and remove data whitening */
static void _JINYANG_removeWhitening(uint8_t *buffer, int length)
{
    int i;

    for (i = 0; i < length; i++) {
        buffer[i] ^= whitening[i % sizeof(whitening)];
    }
}



//TODO
LPCLIB_Result JINYANG_open (JINYANG_Handle *pHandle)
{
    JINYANG_Handle handle = &_jinyang;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _JINYANG_sendKiss (JINYANG_InstanceData *instance)
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

    length = snprintf((char *)s, sizeof(s), "%ld,12,%.3f,,%.5lf,%.5lf,%.0f,,%.1f,%.1f,,,,,,,%.1f,,,%d,",
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

    length = snprintf(s, sizeof(s), "%"PRIu32",12,0,%s",
                instance->id,
                "RSG-20A"
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


static void _JINYANG_sendRaw (JINYANG_Handle handle, JINYANG_Packet *p1)
{
    char s[200];


    snprintf(s, sizeof(s),
                     "%ld,12,1,"
                     "%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8
                     "%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8
                     "%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8
                     "%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8
                     "%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8
                     "%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8
                     "%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8
                     "%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8"%02"PRIX8,
                     handle->instance->id,
                     p1->rawData[0],
                     p1->rawData[1],
                     p1->rawData[2],
                     p1->rawData[3],
                     p1->rawData[4],
                     p1->rawData[5],
                     p1->rawData[6],
                     p1->rawData[7],
                     p1->rawData[8],
                     p1->rawData[9],
                     p1->rawData[10],
                     p1->rawData[11],
                     p1->rawData[12],
                     p1->rawData[13],
                     p1->rawData[14],
                     p1->rawData[15],
                     p1->rawData[16],
                     p1->rawData[17],
                     p1->rawData[18],
                     p1->rawData[19],
                     p1->rawData[20],
                     p1->rawData[21],
                     p1->rawData[22],
                     p1->rawData[23],
                     p1->rawData[24],
                     p1->rawData[25],
                     p1->rawData[26],
                     p1->rawData[27],
                     p1->rawData[28],
                     p1->rawData[29],
                     p1->rawData[30],
                     p1->rawData[31],
                     p1->rawData[32],
                     p1->rawData[33],
                     p1->rawData[34],
                     p1->rawData[35],
                     p1->rawData[36],
                     p1->rawData[37],
                     p1->rawData[38],
                     p1->rawData[39]
                    );

    SYS_send2Host(HOST_CHANNEL_INFO, s);
}



LPCLIB_Result JINYANG_processBlock (
        JINYANG_Handle handle,
        SONDE_Type sondeType,
        void *buffer,
        uint32_t length,
        float rxFrequencyHz)
{
    (void)rxFrequencyHz;
    (void)sondeType;
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    /* Remove data whitening */
    _JINYANG_removeWhitening(buffer, length);

    if (length == sizeof(JINYANG_Packet)) {
        memcpy(&handle->packet, buffer, sizeof(handle->packet));
//        _JINYANG_sendRaw(handle, buffer);

        result = _JINYANG_processConfigFrame(&handle->packet, &handle->instance, rxFrequencyHz);
        if (result == LPCLIB_SUCCESS) {
            /* Process detected subframe type */
            switch (handle->packet.subType) {
                case 0: {
union{
  uint32_t fx;
  float fy;
}xyz;
xyz.fy=handle->packet.frameGps.latitude;
handle->rawlat = xyz.fx;
xyz.fy=handle->packet.frameGps.longitude;
handle->rawlon = xyz.fx;
xyz.fy=handle->packet.frameGps.altitude;
handle->rawalt = xyz.fx;
                    result = _JINYANG_processGpsFrame(&handle->packet.frameGps, handle->instance);
                    if (result == LPCLIB_SUCCESS) {
                        _JINYANG_sendKiss(handle->instance);
                    }
                    handle->instance->frameGps = handle->packet.frameGps;
                    break;}

                case 1:
                    handle->instance->frame1 = handle->packet.frame1;
                    break;

                case 3:
                    handle->instance->frame3 = handle->packet.frame3;
                    break;
            }
        }
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result JINYANG_resendLastPositions (JINYANG_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    JINYANG_InstanceData *instance = NULL;
    while (_JINYANG_iterateInstance(&instance)) {
        _JINYANG_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result JINYANG_removeFromList (JINYANG_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    JINYANG_InstanceData *instance = NULL;
    while (_JINYANG_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _JINYANG_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

