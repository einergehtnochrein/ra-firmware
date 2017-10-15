
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
#include "srsc.h"
#include "srscprivate.h"



/** Context */
typedef struct SRSC_Context {
    SRSC_Packet packet;

    SRSC_InstanceData *instance;
    float rxFrequencyHz;

#if SEMIHOSTING
    FILE *fpLog;
#endif
} SRSC_Context;

static SRSC_Context _srsc;


/* Verify the packet checksum.
 * C34: The buffer consists of five payload bytes plus two checksum bytes.
 *      
 */
static bool _SRSC_doParityCheck (uint8_t *buffer, uint8_t length)
{
    bool result = false;
    int i;
    uint8_t check1, check2;

    if (length == 7) {
        check1 = check2 = 0;
        for (i = 0; i < 5; i++) {
            check1 += buffer[i];
            check2 += check1;
        }
        check2 ^= 0xFF;

        result = (check1 == buffer[5]) && (check2 == buffer[6]);
    }

    return result;
}


//TODO
LPCLIB_Result SRSC_open (SRSC_Handle *pHandle)
{
    *pHandle = &_srsc;
    SRSC_DSP_initAudio();

#if 0 //#if SEMIHOSTING
    _srsc.fpLog = fopen("srsc.txt", "w");
#endif

    return LPCLIB_SUCCESS;
}



/* Send position report */
static void _SRSC_sendKiss (SRSC_InstanceData *instance)
{
    char s[120];
    char sAltitude[20];
    int length = 0;
    float f;

    /* Get frequency */
    f = instance->config.frequencyKhz / 1000.0f;

    /* Convert lat/lon from radian to decimal degrees */
    double latitude = instance->gps.observerLLA.lat;
    double longitude = instance->gps.observerLLA.lon;
    if (!isnan(latitude) && !isnan(longitude)) {
        latitude *= 180.0 / M_PI;
        longitude *= 180.0 / M_PI;
    }

    /* Print altitude string first (empty for an invalid altitude) */
    sAltitude[0] = 0;
    if (!isnan(instance->gps.observerLLA.alt)) {
        sprintf(sAltitude, "%.0f", instance->gps.observerLLA.alt);
    }

    /* Avoid sending the position if any of the values is undefined */
    if (isnan(latitude) || isnan(longitude)) {
        length = sprintf((char *)s, "%s,%s,%.3f,,,,%s,%.1f,%.1f,%.1f,,,%s,,,,%.1f,",
                        instance->config.name,
                        instance->config.isC50 ? "9" : "5",
                        f,         /* Frequency [MHz] */
                        sAltitude, /* Altitude [m] */
                        instance->gps.climbRate,            /* Climb rate [m/s] */
                        0.0f,                               /* Direction [°] */
                        instance->gps.groundSpeed,          /* Horizontal speed [km/h] */
                        instance->config.hasO3 ? "1" : "",
                        SYS_getFrameRssi(sys)
                        );
    }
    else {
        length = sprintf((char *)s, "%s,%s,%.3f,%d,%.5lf,%.5lf,%s,%.1f,%.1f,%.1f,,,%s,,,%.2f,%.1f,,%d",
                        instance->config.name,
                        instance->config.isC50 ? "9" : "5",
                        f,                                  /* Frequency [MHz] */
                        instance->gps.usedSats,
                        latitude,                           /* Latitude [degrees] */
                        longitude,                          /* Longitude [degrees] */
                        sAltitude,                          /* Altitude [m] */
                        instance->gps.climbRate,            /* Climb rate [m/s] */
                        0.0f,                               /* Direction [°] */
                        instance->gps.groundSpeed,          /* Horizontal speed [km/h] */
                        instance->config.hasO3 ? "1" : "",
                        instance->gps.hdop,
                        SYS_getFrameRssi(sys),
                        instance->gps.usedSats
                        );
    }

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }
}



LPCLIB_Result SRSC_processBlock (SRSC_Handle handle, void *buffer, uint32_t length, float rxFrequencyHz)
{
    if (length == 7) {
        if (_SRSC_doParityCheck(buffer, length)) {
            memcpy(&handle->packet, buffer, sizeof(handle->packet));

#if 0 //#if SEMIHOSTING
            fprintf(handle->fpLog, "00FF %02X %04lX %02X\n",
                    handle->packet.type,
                    __REV(handle->packet.d_bigendian),
                    handle->packet.parity);
            fflush(handle->fpLog);
#endif

            /* Always call config handler first to obtain an instance */
            if (_SRSC_processConfigFrame(&handle->packet, &handle->instance, rxFrequencyHz) == LPCLIB_SUCCESS) {
                if (SRSC_isGpsType(handle->packet.type)) {
                    _SRSC_processGpsFrame(&handle->packet, &handle->instance->gps);
                }
            /* Remember RX frequency (difference to nominal sonde frequency will be reported as frequency offset) */
//            handle->rxFrequencyHz = rxFrequencyHz;

                /* If there is a complete position update, send it out (once sonde type is known) */
                if (handle->instance->gps.updateFlags == 7) {
                    handle->instance->gps.updateFlags = 0;
                    if (handle->instance->detectorState >= SRSC_DETECTOR_FIND_NAME) {
                        _SRSC_sendKiss(handle->instance);

                        LPCLIB_Event event;
                        LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
                        event.opcode = APP_EVENT_HEARD_SONDE;
                        event.block = SONDE_DECODER_C34_C50;
                        event.parameter = (void *)((uint32_t)lrintf(rxFrequencyHz));
                        SYS_handleEvent(event);
                    }
                }
            }
        }
    }

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result SRSC_resendLastPositions (SRSC_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    SRSC_InstanceData *instance = NULL;
    while (_SRSC_iterateInstance(&instance)) {
        _SRSC_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list (select by frequency) */
LPCLIB_Result SRSC_removeFromList (SRSC_Handle handle, float rxFrequencyMHz)
{
    (void)handle;

    float rxKhz = roundf(rxFrequencyMHz * 1000.0f);
    SRSC_InstanceData *instance = NULL;
    while (_SRSC_iterateInstance(&instance)) {
        if (roundf(instance->rxFrequencyMHz * 1000.0f) == rxKhz) {
            _SRSC_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


