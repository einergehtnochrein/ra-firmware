
#include <math.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "gps.h"
#include "rs41.h"
#include "rs41private.h"
#include "sys.h"
#include "xdata.h"


/** Context */
typedef struct RS41_Context {
    uint8_t *pRawData;

    bool thisFrameNumberValid;

    float rxFrequencyHz;
    int nCorrectedErrors;

    RS41_InstanceData *instance;
    RS41_RawGps rawGps;
} RS41_Context;


//static LPCLIB_Result _RS41_handleEvent (LPCLIB_Event event);


static RS41_Context _rs41;



/* Check CRC of a sub-block */
static bool _RS41_checkCRC (uint8_t *buffer, int length, uint16_t receivedCRC)
{
    CRC_Handle crc = LPCLIB_INVALID_HANDLE;
    CRC_Mode crcMode;
    bool result = false;

    crcMode = CRC_makeMode(
            CRC_POLY_CRCCCITT,
            CRC_DATAORDER_NORMAL,
            CRC_SUMORDER_NORMAL,
            CRC_DATAPOLARITY_NORMAL,
            CRC_SUMPOLARITY_NORMAL
            );
    if (CRC_open(crcMode, &crc) == LPCLIB_SUCCESS) {
        CRC_seed(crc, 0xFFFF);
        CRC_write(crc, buffer, length, NULL, NULL);

        result = receivedCRC == CRC_read(crc);

        CRC_close(&crc);
    }

    return result;
}


static void _RS41_readSubFrameXdata (char *p, int length, RS41_InstanceData *instance)
{
    /* Minimum required data in XDATA subframe:
     * 1 byte timestamp (skipped, p points to instrument ID)
     * Instrument ID (type of instrument), 2 ASCII digits
     * Instrument number (position in chain), 2 ASCII digits
     */
    if (length >= 4) {
        /* We leave decoding XDATA packets to the app.
         * However, we want to know if an ozone unit is present,
         * so we check for that specific instrument ID.
         */
        int instrumentID;
        int daisyChainNumber;
        if (sscanf(p, "%02X%02X", &instrumentID, &daisyChainNumber) == 2) {
            { /* Send raw data to app */
                static char s[160];
                if (snprintf(s, sizeof(s), "%ld,1,2,%.*s", instance->id, length, p) > 0) {
                    SYS_send2Host(HOST_CHANNEL_INFO, s);
                }
            }

            p += 4;
            ++instance->metro.numXdataInstruments;

            /* TODO: Daisy chain numbers must be consecutive */
            switch (instrumentID) {
                case XDATA_INSTRUMENT_OIF411_OZONE:
                    instance->metro.hasO3 = 1;
                    break;

                default:
                    break;
            }
        }
    }
}


//TODO
LPCLIB_Result RS41_open (RS41_Handle *pHandle)
{
    *pHandle = &_rs41;

    return LPCLIB_SUCCESS;
}


/* Send position as a KISS packet */
static void _RS41_sendKiss (RS41_InstanceData *instance)
{
    static char s[160];
    int length = 0;
    float offset;
    char sPressure[10];
    char sFlightKillTimer[6];
    char sBurstKillTimer[6];
    uint32_t special;
    char sModelName[10+1];

    offset = 0;
    special = 0;

    /* Pressure as string */
    sPressure[0] = 0;
    if (!isnan(instance->metro.pressure)) {
        snprintf(sPressure, sizeof(sPressure), "%.1f", instance->metro.pressure);
    }

    /* Kill timers (if active) */
    sFlightKillTimer[0] = 0;
    if (_RS41_checkValidCalibration(instance, CALIB_FLIGHTKILLTIMER)) {
        if (instance->params.flightKillFrames != -1) {
            snprintf(sFlightKillTimer, sizeof(sFlightKillTimer), "%d", instance->params.flightKillFrames);
        }
    }
    sBurstKillTimer[0] = 0;
    if (_RS41_checkValidCalibration(instance, CALIB_BURSTKILLTIMER)) {
        if (instance->params.burstKillFrames != -1) {
            snprintf(sBurstKillTimer, sizeof(sBurstKillTimer), "%d", instance->params.burstKillFrames);
        }
    }
    int16_t killer = -1;
    if (_RS41_checkValidCalibration(instance, CALIB_KILLCOUNTDOWN)) {
        if (instance->params.killCountdown != -1) {
            killer = instance->killCounterRefCount - (instance->frameCounter - instance->killCounterRefFrame);
        }
    }

    /* Flags */
    if (instance->metro.hasO3) {
        special += (1u << 0);
    }
    if (instance->logMode != RS41_LOGMODE_NONE) {
        special += (1u << 2);
    }
    if (instance->encrypted) {
        special += (1u << 1);
        special += (1u << 7);               /* Inexact location information... */
    }
    if (instance->onDescent) {
        special += (1u << 8);
    }

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

    if (instance->encrypted) {
        length = snprintf((char *)s, sizeof(s), "%ld,1,%.3f,,,,,,,,,,%ld,,,,%.1f,%.1f,%d,%d,%s,%.1f",
                        instance->id,
                        instance->rxFrequencyMHz,               /* RX frequency [MHz] */
                        special,
                        SYS_getFrameRssi(sys),
                        offset,    /* RX frequency offset [kHz] */
                        instance->gps.visibleSats,              /* # satellites */
                        instance->frameCounter,                 /* Current frame number */
                        sFlightKillTimer,                       /* Flight kill timer (frames) */
                        instance->batteryVoltage                /* Battery voltage [V] */
                        );
    }
    else {
        length = snprintf((char *)s, sizeof(s), "%ld,1,%.3f,%d,%.5lf,%.5lf,%.0f,%.1f,%.1f,%.1f,%.1f,%s,%ld,,%.1f,,%.1f,%.1f,%d,%d,%s,%.1f",
                        instance->id,
                        instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                        instance->gps.usedSats,                 /* # sats in position solution */
                        latitude,                               /* Latitude [degrees] */
                        longitude,                              /* Longitude [degrees] */
                        instance->gps.observerLLA.alt,          /* Altitude [m] */
                        instance->gps.observerLLA.climbRate,    /* Climb rate [m/s] */
                        direction,                              /* Direction [°] */
                        velocity,                               /* Horizontal speed [km/h] */
                        instance->metro.T,                      /* Temperature [°C] */
                        sPressure,                              /* Pressure sensor [hPa] */
                        special,
                        instance->metro.RH,
                        SYS_getFrameRssi(sys),
                        offset,                                 /* RX frequency offset [kHz] */
                        instance->gps.visibleSats,              /* # satellites */
                        instance->frameCounter,                 /* Current frame number */
                        sFlightKillTimer,                       /* Kill timer (frame) */
                        instance->batteryVoltage                /* Battery voltage [V] */
                        );
    }

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    sModelName[0] = 0;
    if (_RS41_checkValidCalibration(instance, CALIB_MODELNAME)) {
        memcpy(sModelName, instance->params.names.variant, 10);
        sModelName[10] = 0;
    }
    length = snprintf(s, sizeof(s), "%ld,1,0,%s,%.1f,%s,%.0f,%.0f,%.1f,%d,%s,%d,%.1f,,,,%d",
                instance->id,
                instance->name,
                instance->metro.TU,
                instance->params.names.variant,
                instance->temperatureTx,
                instance->temperatureRef,
                instance->metro.dewpoint,
                instance->gps.sAcc,
                sBurstKillTimer,                        /* Burst kill timer (frames) */
                killer,                                 /* Kill countdown (frames remaining) */
                instance->gps.pdop,
                instance->metro.numXdataInstruments
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}



static char _rs41_rawCompressed[680+42];    /* 680: Max UUENCODE length, 42: some overhead... */

static void _RS41_sendRaw (RS41_InstanceData *instance, uint8_t *buffer, uint32_t length)
{
    uint32_t i = 0;
    int j;
    uint8_t uu[4+1];
    int N = ((length + 1) / 3) * 3;
    int n;
    int slen = 0;
    char *s = _rs41_rawCompressed;


    slen += snprintf(&s[slen], sizeof(_rs41_rawCompressed) - slen, "%ld,1,1,%ld,%ld,",
                     instance->id,
                     length,
                     0l
                    );
    
    for (n = 0; n < N; n += 3) {
        uu[0] = uu[1] = uu[2] = uu[3] = uu[4] = 0;

        if (i < length)  {
            uu[0] = buffer[i] & 0x3F;
            uu[1] = (buffer[i] >> 6) & 0x03;
            ++i;
        }
        if (i < length)  {
            uu[1] |= (buffer[i] << 2) & 0x3C;
            uu[2] = (buffer[i] >> 4) & 0x0F;
            ++i;
        }
        if (i < length)  {
            uu[2] |= (buffer[i] << 4) & 0x30;
            uu[3] = (buffer[i] >> 2) & 0x3F;
            ++i;
        }

        for (j = 0; j < 4; j++) {
            uu[j] ^= 0x20;
            if (uu[j] <= 0x20) {
                uu[j] |= 0x40;
            }
            /* Deviation from UUENCODE: Avoid comma, replace by space */
            if (uu[j] == ',') {
                uu[j] = ' ';
            }
        }

        slen += snprintf(&s[slen], sizeof(_rs41_rawCompressed) - slen, "%s", uu);
    }

    SYS_send2Host(HOST_CHANNEL_INFO, s);
}


LPCLIB_Result RS41_processBlock (RS41_Handle handle, void *buffer, uint32_t length, float rxFrequencyHz)
{
    bool longFrame = false;


    /* Return if raw data is shorter than a short frame. */
    if (length < 312) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    handle->pRawData = buffer;

    /* Remove data whitening */
    _RS41_removeWhitening(buffer, length);

    /* Error correction */
    handle->nCorrectedErrors = 0;
    if (_RS41_checkReedSolomon (handle->pRawData, &handle->nCorrectedErrors, &longFrame) != LPCLIB_SUCCESS) {
        return LPCLIB_ERROR;
    }

    /* Verify received packet length indicator matches detected packet length. */
    uint8_t lengthIndicator = ((uint8_t *)buffer)[48];
    if ((longFrame && (lengthIndicator != 0xF0)) || (!longFrame && (lengthIndicator != 0x0F))) {
        return LPCLIB_ERROR;
    }

    /* Limit buffer length in case of short frame */
    length = longFrame ? 510 : 312;

    /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
    handle->rxFrequencyHz = rxFrequencyHz;

    /* Run through buffer and process all sub-frames */
    {
        uint32_t i = 49;                /* Start after Reed-Solomon parity bytes and packet type indicator */
        uint8_t *p;
        uint8_t subFrameLength;
        uint16_t crc;

        handle->thisFrameNumberValid = false;

        while (i < length) {
            p = (uint8_t *)buffer + i;

            /* Is the next sub-frame valid? Check if it is complete. */
            subFrameLength = p[1] + 4;
            if (subFrameLength <= 4) {
                /* Zero-length. Probably garbage. */
                break;
            }
            if (i + subFrameLength > length) {
                /* Doesn't fit. Probably garbage at packet end. */
                break;
            }

            /* Check sub-frame CRC */
            crc = p[subFrameLength - 2] | (p[subFrameLength - 1] << 8);
            if (_RS41_checkCRC(p + 2, subFrameLength - 4, crc)) {
                /* Check which frame type it is */
                switch (p[0]) {
                case RS41_SUBFRAME_GAPFILL:
                    /* This sub frame of varying length contains all zeros, and is (probably) used
                     * to fill the gap to the end of the frame.
                     */
                    break;
                case RS41_SUBFRAME_CALIB_CONFIG:
                    _RS41_processConfigBlock((RS41_SubFrameCalibConfig *)(p + 2), &handle->instance);
                    if (handle->instance) {
                        handle->instance->rxFrequencyMHz = handle->rxFrequencyHz / 1e6f;
                        handle->instance->metro.numXdataInstruments = 0;
                        if (handle->instance->metro.hasO3) {
                            --handle->instance->metro.hasO3;
                        }
                    }
                    break;
                case RS41_SUBFRAME_METROLOGY:
                    if (handle->instance) {
                        _RS41_processMetrologyBlock((RS41_SubFrameMetrology *)(p + 2), handle->instance);
                    }
                    break;
                case RS41_SUBFRAME_METROLOGY_SHORT:
                    if (handle->instance) {
                        _RS41_processMetrologyShortBlock((RS41_SubFrameMetrologyShort *)(p + 2), handle->instance);
                    }
                    break;
                case RS41_SUBFRAME_GPS_POSITION:
                    if (handle->instance) {
                        _RS41_processGpsPositionBlock((RS41_SubFrameGpsPosition *)(p + 2), &handle->instance->gps);

                        LPCLIB_Event event;
                        LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
                        event.opcode = APP_EVENT_HEARD_SONDE;
                        event.block = SONDE_DETECTOR_RS41_RS92;
                        event.parameter = (void *)((uint32_t)lrintf(rxFrequencyHz));
                        SYS_handleEvent(event);
                    }
                    break;
                case RS41_SUBFRAME_GPS_INFO:
                    if (handle->instance) {
                        _RS41_processGpsInfoBlock(
                                (RS41_SubFrameGpsInfo *)(p + 2),
                                &handle->instance->gps,
                                &handle->rawGps);
                    }
                    break;
                case RS41_SUBFRAME_GPS_RAW:
                    _RS41_processGpsRawBlock((RS41_SubFrameGpsRaw *)(p + 2), &handle->rawGps);
                    break;
                case RS41_SUBFRAME_CRYPT78:
                case RS41_SUBFRAME_CRYPT80:
                    if (handle->instance) {
                        /* RS41-SGM */
                        handle->instance->gps.observerLLA.lat = NAN;
                        handle->instance->gps.observerLLA.lon = NAN;
                        handle->instance->encrypted = true;
                        snprintf(handle->instance->params.names.variant,
                                sizeof(handle->instance->params.names.variant), "%s", "RS41-SGM");
                        handle->instance->logMode = RS41_LOGMODE_RAW;
                    }
                    break;
                case RS41_SUBFRAME_XDATA:
                    if (handle->instance) {
                        _RS41_readSubFrameXdata((char *)(p + 3), subFrameLength - 5, handle->instance);
                    }
                    break;
                default:
                    /* Ignore unknown (or unhandled) frame types */
                    break;
                }
            }

            /* Advance to next sub-frame */
            i += subFrameLength;
        }

        _RS41_sendKiss(handle->instance);
        if (handle->instance->logMode == RS41_LOGMODE_RAW) {
            _RS41_sendRaw(handle->instance, buffer, length);
        }
    }

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result RS41_resendLastPositions (RS41_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    RS41_InstanceData *instance = NULL;
    while (_RS41_iterateInstance(&instance)) {
        _RS41_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result RS41_removeFromList (RS41_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    RS41_InstanceData *instance = NULL;
    while (_RS41_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;
            
            /* Remove sonde */
            _RS41_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


/* Control logging */
LPCLIB_Result RS41_setLogMode (RS41_Handle handle, uint32_t id, RS41_LogMode mode)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;
    RS41_InstanceData *instance = NULL;
    while (_RS41_iterateInstance(&instance)) {
        if (instance->id == id) {
            instance->logMode = mode;
            result = LPCLIB_SUCCESS;
            break;
        }
    }

    return result;
}

