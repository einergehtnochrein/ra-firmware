
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "psb3.h"
#include "psb3private.h"
#include "bch.h"


/** Context */
typedef struct PSB3_Context {
    PSB3_Packet packet;
    PSB3_InstanceData *instance;
    float rxFrequencyHz;
} PSB3_Context;

static PSB3_Context _psb3;


//TODO
LPCLIB_Result PSB3_open (PSB3_Handle *pHandle)
{
    PSB3_Handle handle = &_psb3;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _PSB3_sendKiss (PSB3_InstanceData *instance)
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

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",15,%.3f,,%.5lf,%.5lf,%.0f,,,,%.1f,,,,%.1f,,%.1f,,,%d",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    instance->metro.temperature,            /* Temperature [°C] */
                    instance->metro.humidity,               /* Relative humidity [%] */
                    SYS_getFrameRssi(sys),
                    instance->frameCounter
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",15,0,%s",
                instance->id,
                instance->name
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


static void _PSB3_sendRaw (PSB3_Handle handle, PSB3_Packet *p1)
{
(void)handle;
(void)p1;
}



LPCLIB_Result PSB3_processBlock (
        PSB3_Handle handle,
        SONDE_Type sondeType,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz)
{
    (void)rxFrequencyHz;
    (void)sondeType;
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    if (numBits == 8*sizeof(PSB3_Packet)) {
        memcpy(&handle->packet, buffer, sizeof(handle->packet));

        /* Validate codeword */
        if (_PSB3_checkParity((uint8_t *)&handle->packet, sizeof(handle->packet))) {
            _PSB3_sendRaw(handle, buffer);

            result = _PSB3_prepare(&handle->packet, &handle->instance, rxFrequencyHz);
            if (result == LPCLIB_SUCCESS) {
                _PSB3_processPayload(&handle->packet, handle->instance);
                if (result == LPCLIB_SUCCESS) {
                    _PSB3_sendKiss(handle->instance);
                }
            }
        }
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result PSB3_resendLastPositions (PSB3_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    PSB3_InstanceData *instance = NULL;
    while (_PSB3_iterateInstance(&instance)) {
        _PSB3_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result PSB3_removeFromList (PSB3_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    PSB3_InstanceData *instance = NULL;
    while (_PSB3_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _PSB3_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

