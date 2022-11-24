
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "mrz.h"
#include "mrzprivate.h"
#include "bch.h"


/** Context */
typedef struct MRZ_Context {
    MRZ_Packet packet;
    MRZ_InstanceData *instance;
    float rxFrequencyHz;
} MRZ_Context;

static MRZ_Context _mrz;


/* Check frame CRC */
static bool _MRZ_checkCRC (MRZ_Handle handle)
{
    CRC_Handle crc = LPCLIB_INVALID_HANDLE;
    CRC_Mode crcMode;
    bool result = false;

    crcMode = CRC_makeMode(
            CRC_POLY_CRC16,
            CRC_DATAORDER_REVERSE,
            CRC_SUMORDER_REVERSE,
            CRC_DATAPOLARITY_NORMAL,
            CRC_SUMPOLARITY_NORMAL
            );
    if (CRC_open(crcMode, &crc) == LPCLIB_SUCCESS) {
        CRC_seed(crc, 0xFFFF);
        CRC_write(crc, &handle->packet.rawData, 47 - 2, NULL, NULL);
        /* CRC is sent in big-endian format */
        result = (handle->packet.crc == CRC_read(crc));

        CRC_close(&crc);
    }

    return result;
}



//TODO
LPCLIB_Result MRZ_open (MRZ_Handle *pHandle)
{
    MRZ_Handle handle = &_mrz;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _MRZ_sendKiss (MRZ_InstanceData *instance)
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

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",14,%.3f,%d,%.5lf,%.5lf,%.0f,,%.1f,%.1f,%.1f,%.1f,,,%.1f,,%.1f,,,%d,,%.1f,,,%"PRIu64,
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    instance->gps.usedSats,
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    direction,                              /* GPS direction [degrees] */
                    velocity,                               /* GPS velocity [km/h] */
                    instance->metro.temperature,            /* Temperature [°C] */
                    instance->metro.pressure,               /* Pressure [hPa] */
                    instance->metro.humidity,               /* Relative humidity [%] */
                    instance->rssi,
                    instance->frameCounter,
                    instance->metro.batteryVoltage,         /* Battery voltage [V] */
                    instance->realTime
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    if (_MRZ_checkValidCalibration(instance, CALIB_SERIALSONDE | CALIB_SERIALSENSOR)) {
        length = snprintf(s, sizeof(s), "%"PRIu32",14,0,%s,%"PRIu32",%.1f",
                    instance->id,
                    instance->name,
                    instance->calib.serialSensor,
                    instance->gps.pAcc
                    );

        if (length > 0) {
            SYS_send2Host(HOST_CHANNEL_INFO, s);
        }
    }
}


static void _MRZ_sendRaw (MRZ_Handle handle, MRZ_Packet *p1)
{
    char s[200];


    snprintf(s, sizeof(s),
                     "%"PRIu32",14,1,"
                     "%08"PRIX32"%08"PRIX32"%08"PRIX32"%08"PRIX32"%08"PRIX32
                     "%08"PRIX32"%08"PRIX32"%08"PRIX32"%08"PRIX32"%08"PRIX32
                     "%08"PRIX32"%02"PRIX32"%02"PRIX32"%02"PRIX32,
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
                     __REV(p1->rawData.dat32[9]),
                     __REV(p1->rawData.dat32[10]),
                     (uint32_t)p1->rawData.dat8[44],
                     (uint32_t)p1->rawData.dat8[45],
                     (uint32_t)p1->rawData.dat8[46]
                    );

    SYS_send2Host(HOST_CHANNEL_INFO, s);
}



LPCLIB_Result MRZ_processBlock (
        MRZ_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime)
{
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    if (numBits == 8*sizeof(MRZ_Packet)) {
        memcpy(&handle->packet, buffer, sizeof(handle->packet));

        /* Check CRC-16 at the end of the frame */
        if (_MRZ_checkCRC(handle)) {
            _MRZ_sendRaw(handle, buffer);

            result = _MRZ_processConfigFrame(&handle->packet, &handle->instance, rxFrequencyHz);
            if (result == LPCLIB_SUCCESS) {
                handle->instance->rssi = rssi;
                handle->instance->realTime = realTime;
                _MRZ_processMetrology(&handle->packet, handle->instance);
                result = _MRZ_processGpsFrame(&handle->packet, handle->instance);
                if (result == LPCLIB_SUCCESS) {
                    _MRZ_sendKiss(handle->instance);
                }
            }
        }
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result MRZ_resendLastPositions (MRZ_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    MRZ_InstanceData *instance = NULL;
    while (_MRZ_iterateInstance(&instance)) {
        _MRZ_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result MRZ_removeFromList (MRZ_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    MRZ_InstanceData *instance = NULL;
    while (_MRZ_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _MRZ_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

