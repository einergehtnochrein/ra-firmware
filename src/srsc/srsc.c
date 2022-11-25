
#include <inttypes.h>
#include <math.h>
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

    SRSC_DSP_initAudio();

#if 0 //#if SEMIHOSTING
    handle->fpLog = fopen("srsc.txt", "w");
#endif

    return LPCLIB_SUCCESS;
}



/* Send position report */
static void _SRSC_sendKiss (SRSC_InstanceData *instance)
{
    char s[160];
    char sAltitude[20];
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

    /* Sonde type */
    special = 0;
    if (instance->config.hasO3) {
        special += (1u << 0);
    }
    if (instance->config.isC34) {
        special += (1u << 2);
    }
    if (instance->config.isC50) {
        special += (1u << 3);
    }
    if (instance->config.state >= 8) {
        special += (1u << 8);
    }
    snprintf(sSpecial, sizeof(sSpecial), "%"PRIu32, special);

    length = sprintf((char *)s, "%"PRIu32",8,%.3f,%d,%.5lf,%.5lf,%s,%.1f,,,%s,,%s,%.3f,%.1f,%.2f,%.1f,%.2f,%d,,,%.3f,,,%.2lf",
                    instance->id,
                    f,                                  /* Frequency [MHz] */
                    instance->gps.usedSats,
                    latitude,                           /* Latitude [degrees] */
                    longitude,                          /* Longitude [degrees] */
                    sAltitude,                          /* Altitude [m] */
                    instance->gps.climbRate,            /* Climb rate [m/s] */
                    sTemperature,
                    sSpecial,
                    instance->config.rfPwrDetect,       /* RF power detector [V] */
                    instance->metro.humidity,           /* Humidity [%] */
                    instance->gps.hdop,
                    instance->rssi,
                    instance->rxOffset / 1e3f,
                    instance->gps.usedSats,
                    instance->config.batteryVoltage,    /* Battery voltage [V] */
                    instance->realTime * 0.01
                    );
    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    if (1) { //TODO must check for C50 (NOTE: Every sonde must send a HOST_CHANNEL_INFO packet!)
        length = sprintf(s, "%"PRIu32",8,0,%s,,%.1f,%.1f,%.1f,%.1f,%.3f,,%ld,%ld,%d,%"PRIu32",%.2f,%d,%"PRIu32",%"PRIu32",%"PRIu32,
                    instance->id,
                    instance->name,
                    instance->metro.temperatureRefBlock,    /* Reference temperature [°C] */
                    instance->metro.temperatureHuSensor,    /* Temperature near humidity sensor [°C] */
                    instance->metro.temperatureChamber,
                    instance->metro.temperatureO3Inlet,     /* Temperature of air inlet of ECC ozone sensor [°C] */
                    instance->metro.currentO3Cell,          /* Cell current of ECC ozone sensor [µA] */
                    lrintf(instance->metro.USensorHeating),
                    lrintf(instance->metro.USensorFrequency),
                    instance->config.state,
                    instance->config.errorFlags,
                    instance->config.vdop / 100.0f,
                    (int)instance->config.sondeType,
                    instance->config.serialSensorboom,
                    instance->config.firmwareVersion,
                    instance->config.gpsInterferer
                 );

        if (length > 0) {
            SYS_send2Host(HOST_CHANNEL_INFO, s);
        }
    }
}



LPCLIB_Result SRSC_processBlock (
        SRSC_Handle handle,
        void *buffer,
        float rxSetFrequencyHz,
        float rxOffset,
        float rssi,
        uint64_t realTime)
{
    if (_SRSC_doParityCheck(buffer, 7)) {
        memcpy(&handle->packet, buffer, sizeof(handle->packet));

        /* Log */
        if (handle->instance->detectorState == SRSC_DETECTOR_READY) {
            char log[40];
            snprintf(log, sizeof(log), "%"PRIu32",8,1,%d,%08"PRIX32,
                        handle->instance->id,
                        handle->packet.type,
                        handle->packet.d_bigendian);
            SYS_send2Host(HOST_CHANNEL_INFO, log);
        }

        /* Remove obfuscation */
        handle->packet.rawData[1] ^= handle->instance->obfuscation;
        handle->packet.rawData[2] ^= handle->instance->obfuscation;
        handle->packet.rawData[3] ^= handle->instance->obfuscation;
        handle->packet.rawData[4] ^= handle->instance->obfuscation;

        /* Always call config handler first to obtain an instance */
        if (_SRSC_processConfigFrame(&handle->packet, &handle->instance, rxSetFrequencyHz) == LPCLIB_SUCCESS) {
            handle->instance->rssi = rssi;
            handle->instance->rxOffset = rxOffset;
            handle->instance->realTime = realTime;

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
                event.parameter = (void *)((uintptr_t)lrintf(rxSetFrequencyHz));
                SYS_handleEvent(event);
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
LPCLIB_Result SRSC_removeFromList (SRSC_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    SRSC_InstanceData *instance = NULL;
    while (_SRSC_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _SRSC_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


LPCLIB_Result SRSC_pauseResume (SRSC_Handle handle, bool pause)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    (void)pause;

    /* Simple approach: Reset decode pause begins/ends */
    SRSC_DSP_reset();
    
    return LPCLIB_SUCCESS;
}

