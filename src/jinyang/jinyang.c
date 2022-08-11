
#include <inttypes.h>
#include <math.h>
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
    0xFF,0xE1,0x1D,0x9A,0xED,0x85,0x33,0x24,
    0xEA,0x7A,0xD2,0x39,0x70,0x97,0x57,0x0A,
    0x54,0x7D,0x2D,0xD8,0x6D,0x0D,0xBA,0x8F,
    0x67,0x59,0xC7,0xA2,0xBF,0x34,0xCA,0x18,
    0x30,0x53,0x93,0xDF,0x92,0xEC,0xA7,0x15,
};


/* Do bit reversal, and remove data whitening */
static void _JINYANG_removeWhitening(uint8_t *buffer, int length)
{
    int i;

    for (i = 0; i < length; i++) {
        buffer[i] ^= whitening[i % sizeof(whitening)];
    }
}



/* Check frame CRC */
static bool _JINYANG_checkCRC (JINYANG_Handle handle)
{
    CRC_Handle crc = LPCLIB_INVALID_HANDLE;
    CRC_Mode crcMode;
    bool result = false;

    crcMode = CRC_makeMode(
            CRC_POLY_CRC16,
            CRC_DATAORDER_NORMAL,
            CRC_SUMORDER_NORMAL,
            CRC_DATAPOLARITY_NORMAL,
            CRC_SUMPOLARITY_NORMAL
            );
    if (CRC_open(crcMode, &crc) == LPCLIB_SUCCESS) {
        CRC_seed(crc, 0xFFFF);
        CRC_write(crc, &handle->packet.rawData, 40 - 2, NULL, NULL);
        /* CRC is sent in big-endian format */
        result = (__REV16(handle->packet.crc) == CRC_read(crc));

        CRC_close(&crc);
    }

    return result;
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

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",12,%.3f,,%.5lf,%.5lf,%.0f,,%.1f,%.1f,,,,,,,%.1f,,,%d,",
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
                     "%"PRIu32",12,1,"
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
}



LPCLIB_Result JINYANG_processBlock (
        JINYANG_Handle handle,
        SONDE_Type sondeType,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz)
{
    (void)rxFrequencyHz;
    (void)sondeType;
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    /* Remove data whitening */
    _JINYANG_removeWhitening(buffer, numBits/8);

    if (numBits == 8*sizeof(JINYANG_Packet)) {
        memcpy(&handle->packet, buffer, sizeof(handle->packet));

        /* Check CRC-16 at the end of the frame */
        if (_JINYANG_checkCRC(handle)) {
            _JINYANG_sendRaw(handle, buffer);

            result = _JINYANG_processConfigFrame(&handle->packet, &handle->instance, rxFrequencyHz);
            if (result == LPCLIB_SUCCESS) {
                /* Process subframe type */
                switch (handle->packet.subType) {
                    case 0:
                        handle->instance->frame0 = handle->packet.frame0;
                        break;

                    case 2:
                        handle->instance->frame2 = handle->packet.frame2;
                        break;

                    case 3: {
                        result = _JINYANG_processGpsFrame(&handle->packet.frameGps, handle->instance);
                        if (result == LPCLIB_SUCCESS) {
                            _JINYANG_sendKiss(handle->instance);
                        }
                        handle->instance->frameGps = handle->packet.frameGps;
                        break;}
                }
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

