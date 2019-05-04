
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
#include "meisei.h"
#include "meiseiprivate.h"
#include "bch.h"


/** Context */
typedef struct MEISEI_Context {
    MEISEI_Packet configPacket;
    float configPacketTimestamp;
    MEISEI_Packet gpsPacket;

    MEISEI_InstanceData *instance;
    float rxFrequencyHz;
    float rxOffset;

    uint64_t *pBCH;
} MEISEI_Context;

static MEISEI_Context _meisei;


//TODO
LPCLIB_Result MEISEI_open (MEISEI_Handle *pHandle)
{
    MEISEI_Handle handle = &_meisei;

    *pHandle = handle;

    handle->rxOffset = NAN;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _MEISEI_sendKiss (MEISEI_InstanceData *instance)
{
    static char s[160];
    int length = 0;
    uint32_t special;
    char sSpecial[8];


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

    /* Meisei type indicator */
    special = 0;
    switch (instance->model) {
        case MEISEI_MODEL_IMS100:
            special |= 1u << 2;
            break;
        case MEISEI_MODEL_RS11G:
            special |= 1u << 3;
            break;
        case MEISEI_MODEL_UNKNOWN:
            /* Nothing to do */
            break;
    }
    snprintf(sSpecial, sizeof(sSpecial), "%lu", special);

    length = snprintf((char *)s, sizeof(s), "%ld,11,%.3f,,%.5lf,%.5lf,%.0f,%.1f,%.1f,%.1f,%.1f,,%s,,,,%.1f,,,%d,,,%.1f",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    instance->gps.observerLLA.climbRate,    /* Climb rate [m/s] */
                    direction,                              /* Direction [°] */
                    velocity,                               /* [km/h] */
                    instance->metro.temperature,
                    sSpecial,
                    SYS_getFrameRssi(sys),
                    instance->frameCounter / 2,
                    instance->metro.cpuTemperature
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",11,0,%"PRIu32,
                instance->id,
                instance->serialSonde
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


static void _MEISEI_sendRaw (MEISEI_Handle handle, MEISEI_Packet *p1, MEISEI_Packet *p2)
{
    char s[200];


    snprintf(s, sizeof(s),
                     "%ld,11,1,"
                     "%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16
                     "%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16
                     ","
                     "%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16
                     "%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16,
                     handle->instance->id,
                     _MEISEI_getPayloadHalfWord(p1->fields, 0),
                     _MEISEI_getPayloadHalfWord(p1->fields, 1),
                     _MEISEI_getPayloadHalfWord(p1->fields, 2),
                     _MEISEI_getPayloadHalfWord(p1->fields, 3),
                     _MEISEI_getPayloadHalfWord(p1->fields, 4),
                     _MEISEI_getPayloadHalfWord(p1->fields, 5),
                     _MEISEI_getPayloadHalfWord(p1->fields, 6),
                     _MEISEI_getPayloadHalfWord(p1->fields, 7),
                     _MEISEI_getPayloadHalfWord(p1->fields, 8),
                     _MEISEI_getPayloadHalfWord(p1->fields, 9),
                     _MEISEI_getPayloadHalfWord(p1->fields, 10),
                     _MEISEI_getPayloadHalfWord(p1->fields, 11),
                     _MEISEI_getPayloadHalfWord(p2->fields, 0),
                     _MEISEI_getPayloadHalfWord(p2->fields, 1),
                     _MEISEI_getPayloadHalfWord(p2->fields, 2),
                     _MEISEI_getPayloadHalfWord(p2->fields, 3),
                     _MEISEI_getPayloadHalfWord(p2->fields, 4),
                     _MEISEI_getPayloadHalfWord(p2->fields, 5),
                     _MEISEI_getPayloadHalfWord(p2->fields, 6),
                     _MEISEI_getPayloadHalfWord(p2->fields, 7),
                     _MEISEI_getPayloadHalfWord(p2->fields, 8),
                     _MEISEI_getPayloadHalfWord(p2->fields, 9),
                     _MEISEI_getPayloadHalfWord(p2->fields, 10),
                     _MEISEI_getPayloadHalfWord(p2->fields, 11)
                    );

    SYS_send2Host(HOST_CHANNEL_INFO, s);
}


/* Index: 0...11 */
uint16_t _MEISEI_getPayloadHalfWord (const uint64_t *fields, int index)
{
    uint32_t x = 0;
    if (index % 2) {
        x = (fields[index / 2] & 0x00000001FFFE0000LL) >> 1;
    }
    else {
        x = (fields[index / 2] & 0x000000000000FFFFLL) << 16;
    };

    return __RBIT(x);
}



static int _MEISEI_getDataBCH (int index)
{
    MEISEI_Handle handle = &_meisei;

    if ((handle->pBCH != NULL) && (index < 12+34)) {
        return (*handle->pBCH >> (46 - 1 - index)) & 1;
    }
    else {
        return 0;
    }
}

static void _MEISEI_toggleDataBCH (int index)
{
    MEISEI_Handle handle = &_meisei;

    if ((handle->pBCH != NULL) && (index < 12+34)) {
        *handle->pBCH ^= 1ull << (46 - 1 - index);
    }
}



LPCLIB_Result MEISEI_processBlock (
        MEISEI_Handle handle,
        SONDE_Type sondeType,
        void *buffer,
        uint32_t length,
        float rxFrequencyHz)
{
    (void)rxFrequencyHz;
    int nErrors;
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    if (length == sizeof(MEISEI_Packet)) {
        if (sondeType == SONDE_MEISEI_CONFIG) {
            handle->configPacketTimestamp = 10.0f * os_time;
            memcpy(&handle->configPacket, buffer, sizeof(handle->configPacket));
            return LPCLIB_PENDING;
        }
        else if (sondeType != SONDE_MEISEI_GPS) {
            return LPCLIB_ILLEGAL_PARAMETER;
        }

        memcpy(&handle->gpsPacket, buffer, sizeof(handle->gpsPacket));

        /* Check if we have a matching config frame */
        float diff = fabs(10.0f * os_time - handle->configPacketTimestamp - 250.0f);
        if (diff > 40.0f) {
            return LPCLIB_PENDING;
        }

        nErrors = 0;

        /* First frame */
        /* Do BCH check for all six codewords */

        result = LPCLIB_SUCCESS;
        int i;
        for (i = 0; i < 6; i++) {
            handle->pBCH = &handle->gpsPacket.fields[i];
            result = BCH_63_51_t2_process(_MEISEI_getDataBCH, _MEISEI_toggleDataBCH, &nErrors);
            if (result != LPCLIB_SUCCESS) {
                break;
            }
        }
        if (result == LPCLIB_SUCCESS) {
            for (i = 0; i < 6; i++) {
                handle->pBCH = &handle->configPacket.fields[i];
                result = BCH_63_51_t2_process(_MEISEI_getDataBCH, _MEISEI_toggleDataBCH, &nErrors);
                if (result != LPCLIB_SUCCESS) {
                    break;
                }
            }

            if (result == LPCLIB_SUCCESS) {
                _MEISEI_processConfigFrame(&handle->configPacket, &handle->instance, rxFrequencyHz);

                /* Store GPS data depending on frame number even/odd */
                if ((handle->instance->frameCounter % 2) == 0) {
                    handle->instance->gpsPacketEven = handle->gpsPacket;
                    handle->instance->frameCounterEven = handle->instance->frameCounter;
                }
                else {
                    handle->instance->gpsPacketOdd = handle->gpsPacket;
                }

                //TODO Logging
                _MEISEI_sendRaw(handle, &handle->configPacket, &handle->gpsPacket);

                //TODO Process in odd frames (or even?)
                if ((handle->instance->frameCounter % 2) == 1) {
                    if (handle->instance->frameCounter == handle->instance->frameCounterEven + 1) {
                        _MEISEI_processGpsFrame(handle->instance);
                        _MEISEI_processMetrology(handle->instance);
                        _MEISEI_sendKiss(handle->instance);
                    }
                }
            }
        }
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result MEISEI_resendLastPositions (MEISEI_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    MEISEI_InstanceData *instance = NULL;
    while (_MEISEI_iterateInstance(&instance)) {
        _MEISEI_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result MEISEI_removeFromList (MEISEI_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    MEISEI_InstanceData *instance = NULL;
    while (_MEISEI_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _MEISEI_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


