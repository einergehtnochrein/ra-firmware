
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "beacon.h"
#include "beaconprivate.h"
#include "bch.h"


/** Context */
typedef struct BEACON_Context {
    BEACON_Packet packet;

    BEACON_InstanceData *instance;
    float rxFrequencyHz;
    float rxOffset;

} BEACON_Context;

static BEACON_Context _beacon;


//TODO
LPCLIB_Result BEACON_open (BEACON_Handle *pHandle)
{
    BEACON_Handle handle = &_beacon;

    *pHandle = handle;

    handle->rxOffset = NAN;
    BEACON_DSP_initAudio();

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _BEACON_sendKiss (BEACON_InstanceData *instance)
{
    (void)instance;

    char s[100];
    int length = 0;
    double lat = NAN;
    double lon = NAN;
    int special = 0;

    /* Send position if beacon uses a location protocol */
    if (instance->pdf1.isLocationProtocol) {
        lat = instance->pdf1.latitudeCoarse;
        lon = instance->pdf1.longitudeCoarse;
        if (instance->pdf2.valid) {
            lat = instance->pdf2.latitude;
            lon = instance->pdf2.longitude;
        }
    }

    /* Flags */
    if (instance->emergency) {
        special |= 2;
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",9,%.3f,,%.5lf,%.5lf,,,,,,,%d,,,,%.1f,,,,,,,,%.1lf",
                    instance->id,
                    instance->rxFrequencyMHz,           /* Frequency [MHz] */
                    lat,
                    lon,
                    special,
                    instance->rssi,
                    instance->realTime / 10.0
                    );
    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    /* Beacon 15 Hex ID */
    snprintf(s, sizeof(s), "%"PRIu32",9,0,%s,%"PRIu32,
                instance->id,
                instance->pdf1.hexID,
                instance->pdf1.serialNumber
            );
    SYS_send2Host(HOST_CHANNEL_INFO, s);
}


static int _BEACON_getDataBCH1 (int index)
{
    BEACON_Handle handle = &_beacon;

    if (index < 21+61) {
        return (handle->packet.rawData[(38 + index) / 8] >> ((38 + index) % 8)) & 1;
    }
    else {
        return 0;
    }

    return 0;
}

static void _BEACON_toggleDataBCH1 (int index)
{
    BEACON_Handle handle = &_beacon;

    if (index < 21+61) {
        uint8_t *p = &handle->packet.rawData[(38 + index) / 8];
        uint8_t bitNo = (38 + index) % 8;
        *p ^= 1u << bitNo;
    }
}

static int _BEACON_getDataBCH2 (int index)
{
    BEACON_Handle handle = &_beacon;

    if (index < 12+26) {
        return (handle->packet.rawData[index / 8] >> (index % 8)) & 1;
    }
    else {
        return 0;
    }

    return 0;
}

static void _BEACON_toggleDataBCH2 (int index)
{
    BEACON_Handle handle = &_beacon;

    if (index < 12+26) {
        uint8_t *p = &handle->packet.rawData[index / 8];
        uint8_t bitNo = index % 8;
        *p ^= 1u << bitNo;
    }
}

void _BEACON_getField (uint32_t fieldDescriptor, uint8_t *dest)
{
    BEACON_Handle handle = &_beacon;

    int index = fieldDescriptor / 256;
    int length = fieldDescriptor % 256;

    if ((index >= 25) && (index + length <= 144+1)) {
        dest[(length - 1) / 8] = 0;
        for (int i = 0; i < length; i++) {
            uint8_t byte = handle->packet.rawData[14 - (index - 25 + i) / 8];
            uint8_t bit = (byte >> (7 - ((index - 25 + i) % 8))) & 1;
            dest[(length - 1 - i) / 8] = (dest[(length - 1 - i) / 8] << 1) | bit;
        }
    }
}



LPCLIB_Result BEACON_processBlock (
        BEACON_Handle handle,
        void *buffer,
        float rxFrequencyHz,
        uint8_t emergency,
        float rssi,
        uint64_t realTime)
{
    int nErrors;

    memcpy(&handle->packet, buffer, sizeof(handle->packet));

    /* Log */
    {
        char log[60];
        snprintf(log, sizeof(log), "%s,9,1,%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                    "none", //TODO
                    handle->packet.rawData[14],
                    handle->packet.rawData[13],
                    handle->packet.rawData[12],
                    handle->packet.rawData[11],
                    handle->packet.rawData[10],
                    handle->packet.rawData[9],
                    handle->packet.rawData[8],
                    handle->packet.rawData[7],
                    handle->packet.rawData[6],
                    handle->packet.rawData[5],
                    handle->packet.rawData[4],
                    handle->packet.rawData[3],
                    handle->packet.rawData[2],
                    handle->packet.rawData[1],
                    handle->packet.rawData[0]);
        SYS_send2Host(HOST_CHANNEL_INFO, log);
    }

    nErrors = 0;

    /* Check BCH-1 */
    if (BCH_127_106_t3_process(_BEACON_getDataBCH1, _BEACON_toggleDataBCH1, &nErrors) == LPCLIB_SUCCESS) {
        /* Process protected data field 1 (also updates 'longMessage' flag) */
        if (_BEACON_processConfigFrame(&handle->instance) == LPCLIB_SUCCESS) {
            handle->instance->rssi = rssi;
            handle->instance->rxFrequencyMHz = rxFrequencyHz / 1e6f;
            handle->instance->realTime = realTime;
            handle->instance->emergency = (emergency != 0);
            if (handle->instance->pdf1.longMessage) {
                /* Check BCH-2 */
                if (BCH_63_51_t2_process(_BEACON_getDataBCH2, _BEACON_toggleDataBCH2, &nErrors) == LPCLIB_SUCCESS) {
                    /* Process protected data field 2 */
                    if (_BEACON_processPDF2(&handle->instance->pdf1, &handle->instance->pdf2) != LPCLIB_SUCCESS) {
                        handle->instance->pdf2.valid = false;
                    }
                    else {
                        handle->instance->pdf2.valid = true;
                    }
                }
            }
            else {
                /* Process non-protected field following PDF-1 */
                ;
                ;
                ;
            }

            _BEACON_sendKiss(handle->instance);
        }
    }

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result BEACON_resendLastPositions (BEACON_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    BEACON_InstanceData *instance = NULL;
    while (_BEACON_iterateInstance(&instance)) {
        _BEACON_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result BEACON_removeFromList (BEACON_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    BEACON_InstanceData *instance = NULL;
    while (_BEACON_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _BEACON_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


