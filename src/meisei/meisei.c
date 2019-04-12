
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


    /* Convert lat/lon from radian to decimal degrees */
    double latitude = instance->gps.observerLLA.lat;
    if (!isnan(latitude)) {
        latitude *= 180.0 / M_PI;
    }
    double longitude = instance->gps.observerLLA.lon;
    if (!isnan(longitude)) {
        longitude *= 180.0 / M_PI;
    }

    length = snprintf((char *)s, sizeof(s), "%ld,11,%.3f,,%.5lf,%.5lf,%.0f,,,,,,,,,,%.1f,",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    SYS_getFrameRssi(sys)
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%ld,11,0,",
                instance->id
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


static int _MEISEI_getDataBCH_x (MEISEI_Packet *packet, int offset, int index)
{
    MEISEI_Handle handle = &_meisei;

    if (index < 12) {   /* Parity bits */
        return (packet->rawData[(offset + 34 + index) / 8] >> ((offset + 34 + index) % 8)) & 1;
    }
    else if (index < 12+34) {
        return (packet->rawData[(offset - 12 + index) / 8] >> ((offset - 12 + index) % 8)) & 1;
    }
    else {
        return 0;
    }
}

static int _MEISEI_getDataBCH_0_0 (int index)
{
    return _MEISEI_getDataBCH_x(&_meisei.configPacket, 0*46, index);
}

static int _MEISEI_getDataBCH_0_1 (int index)
{
    return _MEISEI_getDataBCH_x(&_meisei.configPacket, 1*46, index);
}

static int _MEISEI_getDataBCH_0_2 (int index)
{
    return _MEISEI_getDataBCH_x(&_meisei.configPacket, 2*46, index);
}

static int _MEISEI_getDataBCH_0_3 (int index)
{
    return _MEISEI_getDataBCH_x(&_meisei.configPacket, 3*46, index);
}

static int _MEISEI_getDataBCH_0_4 (int index)
{
    return _MEISEI_getDataBCH_x(&_meisei.configPacket, 4*46, index);
}

static int _MEISEI_getDataBCH_0_5 (int index)
{
    return _MEISEI_getDataBCH_x(&_meisei.configPacket, 5*46, index);
}

static void _MEISEI_toggleDataBCH (int index)
{
    MEISEI_Handle handle = &_meisei;
#if 0
    if (index < 21+61) {
        uint8_t *p = &handle->packet.rawData[(38 + index) / 8];
        uint8_t bitNo = (38 + index) % 8;
        *p ^= 1u << bitNo;
    }
#endif
}



LPCLIB_Result MEISEI_processBlock (
        MEISEI_Handle handle,
        SONDE_Type sondeType,
        void *buffer,
        uint32_t length,
        float rxFrequencyHz)
{
    (void)rxFrequencyHz;
//    int nErrors;

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

#if 0
        /* Log */
        {
            char log[120];
            snprintf(log, sizeof(log), "%ld,11,1,%02X,%02X",
                        handle->instance->id,
                        handle->packet.rawData[1],
                        handle->packet.rawData[0]);
            SYS_send2Host(HOST_CHANNEL_INFO, log);
        }
#endif

//        nErrors = 0;

        /* First frame */
        /* Do BCH check for all six codewords */
#if 0
        if (BCH_63_51_t2_process(_MEISEI_getDataBCH_0_0, _MEISEI_toggleDataBCH, &nErrors) == LPCLIB_SUCCESS) {
            if (BCH_63_51_t2_process(_MEISEI_getDataBCH_0_1, _MEISEI_toggleDataBCH, &nErrors) == LPCLIB_SUCCESS) {
                if (BCH_63_51_t2_process(_MEISEI_getDataBCH_0_2, _MEISEI_toggleDataBCH, &nErrors) == LPCLIB_SUCCESS) {
                    if (BCH_63_51_t2_process(_MEISEI_getDataBCH_0_3, _MEISEI_toggleDataBCH, &nErrors) == LPCLIB_SUCCESS) {
                        if (BCH_63_51_t2_process(_MEISEI_getDataBCH_0_4, _MEISEI_toggleDataBCH, &nErrors) == LPCLIB_SUCCESS) {
                            if (BCH_63_51_t2_process(_MEISEI_getDataBCH_0_5, _MEISEI_toggleDataBCH, &nErrors) == LPCLIB_SUCCESS) {
#endif
                                _MEISEI_processConfigFrame(&handle->instance, rxFrequencyHz);
                                _MEISEI_processGpsFrame(&handle->gpsPacket, handle->instance);
                                _MEISEI_sendKiss(handle->instance);
#if 0
                            }
                        }
                    }
                }
            }
        }
#endif
    }

    return LPCLIB_SUCCESS;
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


