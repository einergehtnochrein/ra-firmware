
#include <inttypes.h>
#include <math.h>
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
    MEISEI_RawPacket rawConfigPacket;
    float configPacketTimestamp;
    MEISEI_RawPacket rawGpsPacket;
    MEISEI_Packet configPacket;
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
    char sModel[20];


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
    if (instance->metro.temperatureSensorBroken) {
        special |= (1u << 4);
    }
    sModel[0] = 0;
    switch (instance->model) {
        case MEISEI_MODEL_IMS100:
            special |= 1u << 2;
            snprintf(sModel, sizeof(sModel), "iMS-100");
            break;
        case MEISEI_MODEL_RS11G:
            special |= 1u << 3;
            snprintf(sModel, sizeof(sModel), "RS11G");
            break;
        case MEISEI_MODEL_UNKNOWN:
            /* Nothing to do */
            break;
    }
    snprintf(sSpecial, sizeof(sSpecial), "%"PRIu32, special);

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",11,%.3f,%d,%.5lf,%.5lf,%.0f,%.1f,%.1f,%.1f,%.1f,,%s,,%.1f,,%.1f,,%d,%d,,,%.1f",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    instance->gps.usedSats,
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    instance->gps.observerLLA.climbRate,    /* Climb rate [m/s] */
                    direction,                              /* Direction [°] */
                    velocity,                               /* [km/h] */
                    instance->metro.temperature,
                    sSpecial,
                    instance->metro.humidity,               /* Relative humidity [%] */
                    instance->rssi,
                    instance->gps.visibleSats,
                    instance->frameCounter / 2,
                    instance->metro.cpuTemperature
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",11,0,%"PRIu32",%"PRIu32",%"PRIu32",%s,%.0f,%.1f",
                instance->id,
                instance->serialSonde,
                instance->serialSensorBoom,
                instance->serialPcb,
                sModel,
                instance->metro.txTemperature,
                instance->metro.rh_temperature
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


static void _MEISEI_sendRaw (MEISEI_Handle handle, MEISEI_Packet *p1, MEISEI_Packet *p2)
{
    char s[200];


    snprintf(s, sizeof(s),
                     "%"PRIu32",11,1,"
                     "%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16
                     "%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16
                     ","
                     "%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16
                     "%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16"%04"PRIX16,
                     handle->instance->id,
                     p1->w[0],p1->w[1],p1->w[2],p1->w[3],p1->w[4],p1->w[5],
                     p1->w[6],p1->w[7],p1->w[8],p1->w[9],p1->w[10],p1->w[11],
                     p2->w[0],p2->w[1],p2->w[2],p2->w[3],p2->w[4],p2->w[5],
                     p2->w[6],p2->w[7],p2->w[8],p2->w[9],p2->w[10],p2->w[11]
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
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi)
{
    int nErrors;
    int nTotalErrors;
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    if (numBits == 8*sizeof(MEISEI_RawPacket)) {
        if (sondeType == SONDE_MEISEI_CONFIG) {
            handle->configPacketTimestamp = 10.0f * os_time;
            memcpy(&handle->rawConfigPacket, buffer, sizeof(handle->rawConfigPacket));
            return LPCLIB_PENDING;
        }
        else if (sondeType != SONDE_MEISEI_GPS) {
            return LPCLIB_ILLEGAL_PARAMETER;
        }

        memcpy(&handle->rawGpsPacket, buffer, sizeof(handle->rawGpsPacket));

        /* Check if we have a matching config frame */
        float diff = fabs(10.0f * os_time - handle->configPacketTimestamp - 250.0f);
        if (diff > 40.0f) {
            return LPCLIB_PENDING;
        }

        nErrors = 0;
        nTotalErrors = 0;

        /* First frame */
        /* Do BCH check for all six codewords */

        result = LPCLIB_SUCCESS;
        int i;
        for (i = 0; i < 6; i++) {
            handle->pBCH = &handle->rawGpsPacket.fields[i];
            result = BCH_63_51_t2_process(_MEISEI_getDataBCH, _MEISEI_toggleDataBCH, &nErrors);
            nTotalErrors += nErrors;
            if (result != LPCLIB_SUCCESS) {
                break;
            }
        }
        if (result == LPCLIB_SUCCESS) {
            for (i = 0; i < 6; i++) {
                handle->pBCH = &handle->rawConfigPacket.fields[i];
                result = BCH_63_51_t2_process(_MEISEI_getDataBCH, _MEISEI_toggleDataBCH, &nErrors);
                nTotalErrors += nErrors;
                if (result != LPCLIB_SUCCESS) {
                    break;
                }
            }

            /* There is no CRC to help with undetectable frame errors.
             * Set an arbitrary upper limit for the number of corrected errors. Packets with a
             * larger number of errors are rejected, although all BCH codewords appear error-free.
             */
            if ((result == LPCLIB_SUCCESS) && (nTotalErrors < 5)) {
                for (i = 0; i < 12; i++) {
                    handle->configPacket.w[i] = _MEISEI_getPayloadHalfWord(handle->rawConfigPacket.fields, i);
                    handle->gpsPacket.w[i] = _MEISEI_getPayloadHalfWord(handle->rawGpsPacket.fields, i);
                }

                _MEISEI_processConfigFrame(&handle->configPacket, &handle->instance, rxFrequencyHz);
                handle->instance->rssi = rssi;

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


