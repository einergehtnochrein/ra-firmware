
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


#define REPORT_INTERVAL                     (3000/10)


/** Context */
typedef struct SRSC_Context {
    SRSC_Packet packet;

    SRSC_InstanceData *instance;
    float rxFrequencyHz;
    float rxOffset;
    uint32_t timeOfLastReport;

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
    SRSC_Handle handle = &_srsc;

    *pHandle = handle;

    handle->rxOffset = NAN;
    SRSC_DSP_initAudio();

#if 0 //#if SEMIHOSTING
    handle->fpLog = fopen("srsc.txt", "w");
#endif

    return LPCLIB_SUCCESS;
}



/* Inform decoder about RX offset */
LPCLIB_Result SRSC_setRxOffset (SRSC_Handle handle, float rxOffset)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    handle->rxOffset = rxOffset;

    return LPCLIB_SUCCESS;
}



/* Send position report */
static void _SRSC_sendKiss (SRSC_InstanceData *instance)
{
    char s[140];
    char sAltitude[20];
    char sOffset[8];
    char sTemperature[8];
    int length = 0;
    float f;
    uint32_t special;
    char sSpecial[8];

    /* Get frequency */
    f = instance->config.frequencyKhz / 1000.0f;

    /* Sensors */
    sTemperature[0] = 0;
    if (!isnan(instance->metro.temperatureAir)) {
        snprintf(sTemperature, sizeof(sTemperature), "%.1f", instance->metro.temperatureAir);
    }

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

    /* Frequency offset */
    sOffset[0] = 0;
    if (!isnan(instance->rxOffset)) {
        snprintf(sOffset, sizeof(sOffset), "%.2f", instance->rxOffset / 1e3f);
    }

    /* Sonde type */
    special = 0;
    if (instance->config.hasO3) {
        special += 1;
    }
    if (instance->config.isC34) {
        special += 4;
    }
    if (instance->config.isC50) {
        special += 8;
    }
    snprintf(sSpecial, sizeof(sSpecial), "%lu", special);

    /* Avoid sending the position if any of the values is undefined */
    if (isnan(latitude) || isnan(longitude)) {
        length = sprintf((char *)s, "%s,8,%.3f,,,,%s,%.1f,,,%s,,%s,%.3f,,,%.1f,%s,,,,%.3f",
                        instance->config.name,
                        f,         /* Frequency [MHz] */
                        sAltitude, /* Altitude [m] */
                        instance->gps.climbRate,            /* Climb rate [m/s] */
                        sTemperature,
                        sSpecial,
                        instance->config.rfPwrDetect,       /* RF power detector [V] */
                        SYS_getFrameRssi(sys),
                        sOffset,                            /* RX signal offset [kHz] */
                        instance->config.batteryVoltage     /* Battery voltage [V] */
                        );
    }
    else {
        length = sprintf((char *)s, "%s,8,%.3f,%d,%.5lf,%.5lf,%s,%.1f,,,%s,,%s,%.3f,,%.2f,%.1f,%s,%d,,,%.3f",
                        instance->config.name,
                        f,                                  /* Frequency [MHz] */
                        instance->gps.usedSats,
                        latitude,                           /* Latitude [degrees] */
                        longitude,                          /* Longitude [degrees] */
                        sAltitude,                          /* Altitude [m] */
                        instance->gps.climbRate,            /* Climb rate [m/s] */
                        sTemperature,
                        sSpecial,
                        instance->config.rfPwrDetect,       /* RF power detector [V] */
                        instance->gps.hdop,
                        SYS_getFrameRssi(sys),
                        sOffset,                            /* RX signal offset [kHz] */
                        instance->gps.usedSats,
                        instance->config.batteryVoltage     /* Battery voltage [V] */
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

            /* Log */
            if (handle->instance->detectorState == SRSC_DETECTOR_READY) {
                char log[40];
                snprintf(log, sizeof(log), "%s,8,1,%d,%08lX",
                            handle->instance->name,
                            handle->packet.type,
                            handle->packet.d_bigendian);
                SYS_send2Host(HOST_CHANNEL_INFO, log);
            }

#if 0 //#if SEMIHOSTING
            fprintf(handle->fpLog, "00FF %02X %04lX %02X\n",
                    handle->packet.type,
                    __REV(handle->packet.d_bigendian),
                    handle->packet.parity);
            fflush(handle->fpLog);
#endif

            /* Always call config handler first to obtain an instance */
            if (_SRSC_processConfigFrame(&handle->packet, &handle->instance, rxFrequencyHz) == LPCLIB_SUCCESS) {
                handle->instance->rxOffset = handle->rxOffset;

                if (SRSC_isGpsType(handle->packet.type)) {
                    _SRSC_processGpsFrame(&handle->packet, &handle->instance->gps);
                }
            /* Remember RX frequency (difference to nominal sonde frequency will be reported as frequency offset) */
//            handle->rxFrequencyHz = rxFrequencyHz;

                /* Send updated information to host. Either immediately if there is a new position,
                 * or after a minimum interval.
                 */
                if ((handle->instance->gps.updateFlags == 7) ||
                    (os_time - handle->timeOfLastReport > REPORT_INTERVAL)) {

                    handle->timeOfLastReport = os_time;
                    if (handle->instance->gps.updateFlags == 7) {
                        handle->instance->gps.updateFlags = 0;
                    }

                    _SRSC_sendKiss(handle->instance);

                    LPCLIB_Event event;
                    LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
                    event.opcode = APP_EVENT_HEARD_SONDE;
                    event.block = SONDE_DETECTOR_C34_C50;
                    event.parameter = (void *)((uint32_t)lrintf(rxFrequencyHz));
                    SYS_handleEvent(event);
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
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }
            /* Remove sonde */
            _SRSC_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


