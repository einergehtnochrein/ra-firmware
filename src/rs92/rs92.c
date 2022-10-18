
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "bsp.h"
#include "app.h"
#include "sys.h"
#include "rs92.h"
#include "rs92private.h"
#include "reedsolomon.h"


/** Context */
typedef struct RS92_Context {
    RS92_Packet packet;
    bool crcOK[4];

    float rxFrequencyHz;
    int nSymbolErrors;

    RS92_InstanceData *instance;
} RS92_Context;


static RS92_Context _rs92;


/* Check CRC of sub-blocks */
static void _RS92_checkCRC (RS92_Handle handle)
{
    CRC_Handle crc = LPCLIB_INVALID_HANDLE;
    CRC_Mode crcMode;
    uint16_t receivedCRC;

    crcMode = CRC_makeMode(
            CRC_POLY_CRCCCITT,
            CRC_DATAORDER_NORMAL,
            CRC_SUMORDER_NORMAL,
            CRC_DATAPOLARITY_NORMAL,
            CRC_SUMPOLARITY_NORMAL
            );
    if (CRC_open(crcMode, &crc) == LPCLIB_SUCCESS) {
        /* Check CRC of block 1 */
        CRC_seed(crc, 0xFFFF);
        CRC_write(crc, &handle->packet.rawData[2], 32, NULL, NULL);
        receivedCRC = (handle->packet.rawData[35] << 8) | handle->packet.rawData[34];
        handle->crcOK[0] = receivedCRC == CRC_read(crc);

        /* Check CRC of block 2 */
        CRC_seed(crc, 0xFFFF);
        CRC_write(crc, &handle->packet.rawData[38], 24, NULL, NULL);
        receivedCRC = (handle->packet.rawData[63] << 8) | handle->packet.rawData[62];
        handle->crcOK[1] = receivedCRC == CRC_read(crc);

        /* Check CRC of block 3 */
        CRC_seed(crc, 0xFFFF);
        CRC_write(crc, &handle->packet.rawData[66], 122, NULL, NULL);
        receivedCRC = (handle->packet.rawData[189] << 8) | handle->packet.rawData[188];
        handle->crcOK[2] = receivedCRC == CRC_read(crc);

        /* Check CRC of block 4 */
        CRC_seed(crc, 0xFFFF);
        CRC_write(crc, &handle->packet.rawData[192], 10, NULL, NULL);
        receivedCRC = (handle->packet.rawData[203] << 8) | handle->packet.rawData[202];
        handle->crcOK[3] = receivedCRC == CRC_read(crc);

        CRC_close(&crc);
    }
}


/* Estimate climb rate from altitude measurements (Doppler readings from RS are currently ignored) */
static float _RS92_estimateClimbRate (RS92_Handle handle)
{
    float climbRate = NAN;

    if (!isnan(handle->instance->gps.observerLLA.alt)) {
        uint32_t timeDiff = handle->instance->frameCounter - handle->instance->lastAltitude.frame;
        float altitudeDiff = handle->instance->gps.observerLLA.alt - handle->instance->lastAltitude.alt;
        if (timeDiff > 0) {
            climbRate = altitudeDiff / timeDiff;

            handle->instance->lastAltitude.frame = handle->instance->frameCounter;
            handle->instance->lastAltitude.alt = handle->instance->gps.observerLLA.alt;
        }
    }

    return climbRate;
}



/* Send position as a KISS packet */
static void _RS92_sendKiss (RS92_InstanceData *instance)
{
    char s[150];
    char sAltitude[20];
    char sClimbRate[16];
    char sPressure[10];
    char sTemperature[10];
    char sKillTimer[6];
    int length = 0;
    float offset;
    uint32_t special = 0;

    offset = 0;

    /* Pressure/temperature/humidity sensors */
    sPressure[0] = 0;
    if (!isnan(instance->metro.pressure)) {
        snprintf(sPressure, sizeof(sPressure), "%.1f", instance->metro.pressure);
    }
    sTemperature[0] = 0;
    if (!isnan(instance->metro.temperature)) {
        snprintf(sTemperature, sizeof(sTemperature), "%.1f", instance->metro.temperature);
    }

    /* Convert lat/lon from radian to decimal degrees */
    double latitude = instance->gps.observerLLA.lat;
    double longitude = instance->gps.observerLLA.lon;
    if (!isnan(latitude) && !isnan(longitude)) {
        latitude *= 180.0 / M_PI;
        longitude *= 180.0 / M_PI;
    }

    /* Print altitude string. Use GPS altitude if available, otherwise use altitude
     * derived from pressure measurement. Empty for an invalid altitude.
     */
    sAltitude[0] = 0;
    if (!isnan(instance->gps.observerLLA.alt)) {
        sprintf(sAltitude, "%.0f", instance->gps.observerLLA.alt);
    }
    else if (!isnan(instance->metro.pressureAltitude)) {
        sprintf(sAltitude, "%.0f", instance->metro.pressureAltitude);
    }

    /* Don't send climb rate if it is undefined */
    sClimbRate[0] = 0;
    if (!isnan(instance->gps.climbRate)) {
        sprintf(sClimbRate, "%.1f", instance->gps.climbRate);
    }

    /* Kill timer (if active) */
    sKillTimer[0] = 0;
    if (_RS92_checkValidCalibration(instance, CALIB_KILLTIMER)) {
        if (instance->killTimer != (uint16_t)-1) {
            snprintf(sKillTimer, sizeof(sKillTimer), "%d", instance->killTimer);
        }
    }

    /* Info flags */
    if (instance->metro.hasO3) {
        special += 1;
    }

    length = sprintf((char *)s, "%"PRIu32",0,%.3f,%d,%.5lf,%.5lf,%s,%s,,,%s,%s,%"PRIu32",,%.1f,%.2f,%.1f,%.1f,%d,%d,%s,",
                    instance->id,
                    instance->rxFrequencyMHz,   /* Frequency [MHz] */
                    instance->gps.usedSats,
                    latitude,                   /* Latitude [degrees] */
                    longitude,                  /* Longitude [degrees] */
                    sAltitude,                  /* Altitude [m] */
                    sClimbRate,                 /* Climb rate [m/s] */
                    sTemperature,               /* Temperature [°C] */
                    sPressure,                  /* Pressure [hPa] */
                    special,                    /* Flags (Ozone, ...) */
                    instance->metro.humidity,   /* RH [%] */
                    instance->gps.hdop,
                    instance->rssi,
                    offset,                     /* RX frequency offset [kHz] */
                    instance->gps.visibleSats,
                    instance->frameCounter,     /* Current frame number */
                    sKillTimer                  /* Kill timer (frame) */
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = sprintf(s, "%"PRIu32",0,0,%s",
                instance->id,
                instance->name
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}



//TODO
LPCLIB_Result RS92_open (RS92_Handle *pHandle)
{
    *pHandle = &_rs92;

    return LPCLIB_SUCCESS;
}


static uint8_t _RS92_null = 0;

static uint8_t * _RS92_getDataAddress(int index)
{
    RS92_Handle handle = &_rs92;

    if (index < 210) {
        return &handle->packet.rawData[index];
    }
    else if (index >= 231) {
        return &handle->packet.rawData[index - 21];
    }
    else {
        return &_RS92_null;
    }
}


LPCLIB_Result RS92_setSatelliteSnrThreshold (RS92_Handle handle, float threshold)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if ((threshold < 0) || (threshold > 15.0f)) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (handle->instance != NULL) {
        handle->instance->gps.satelliteSnrThreshold = threshold;
    }

    return LPCLIB_SUCCESS;
}



static char _rs92_raw[2*234+42];

static void _RS92_sendRaw (RS92_InstanceData *instance, uint8_t *buffer, uint32_t length)
{
    int slen = 0;
    char *s = _rs92_raw;

    if (length <= 234) {
        slen += snprintf(&s[slen], sizeof(_rs92_raw) - slen, "%"PRIu32",0,1,%"PRIu32",%d,",
                        instance->id,
                        length,
                        0
                        );

        while (length) {
            slen += snprintf(&s[slen], sizeof(_rs92_raw) - slen, "%02"PRIX32, (uint32_t)*buffer);

            ++buffer;
            --length;
        }

        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


LPCLIB_Result RS92_processBlock (
        RS92_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi)
{
    if (numBits == 234*8) {
        /* Copy RX frame */
        memcpy(handle->packet.rawData, buffer, sizeof(handle->packet.rawData));

        /* Reed-Solomon decoder */
        LPCLIB_Result result = REEDSOLOMON_process(24, 0, _RS92_getDataAddress, &handle->nSymbolErrors);
        if (result != LPCLIB_SUCCESS) {
            /* Try harder. Guess some elements of the code word and try again. */
            handle->packet.config.frameType = RS92_SUBFRAME_CALIB_CONFIG;
            handle->packet.config.length = (sizeof(handle->packet.config) - 4) / 2;
            handle->packet.config.name[0] = ' ';
            handle->packet.config.name[1] = ' ';
            if (handle->instance) {
                int i;
                for (i = 0; i < 8; i++) {
                    handle->packet.config.name[2 + i] = handle->instance->name[i];
                }
            }
            handle->packet.metrology.frameType = RS92_SUBFRAME_METROLOGY;
            handle->packet.metrology.length = (sizeof(handle->packet.metrology) - 4) / 2;
            handle->packet.gps.frameType = RS92_SUBFRAME_GPS;
            handle->packet.gps.length = (sizeof(handle->packet.gps) - 4) / 2;
            handle->packet.aux.frameType = RS92_SUBFRAME_AUX;
            handle->packet.aux.length = (sizeof(handle->packet.aux) - 4) / 2;
            handle->packet.unknown.frameType = RS92_SUBFRAME_PADDING;
            handle->packet.unknown.length = 2;
            handle->packet.unknown.nn[0] = 2;
            handle->packet.unknown.nn[1] = 2;
            result = REEDSOLOMON_process(24, 0, _RS92_getDataAddress, &handle->nSymbolErrors);
        }

        if (result == LPCLIB_SUCCESS) {
            /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
            handle->rxFrequencyHz = rxFrequencyHz;

            /* Check sub-block CRCs.
             * There's a very low probability for any of them being wrong (we only process valid code words!),
             * but you never know...
             */
            _RS92_checkCRC(handle);

            /* Ignore everything else if the config block is invalid */
            if (handle->crcOK[0]) {
                /* Process the config/calib block, and obtain the sonde name and the calibration data */
                if (_RS92_processConfigBlock(&handle->packet.config, &handle->instance) == LPCLIB_SUCCESS) {
                    handle->instance->rxFrequencyMHz = handle->rxFrequencyHz / 1e6f;
                    handle->instance->rssi = rssi;

                    /* Process the (valid) metrology block only if there is calibration data available. */
                    if (handle->crcOK[1] && handle->instance) {
                        _RS92_processMetrologyBlock(&handle->packet.metrology, &handle->instance->metro, handle->instance);
                    }

                    /* Process the (valid) GPS block */
                    if (handle->crcOK[2]) {
                        _RS92_processGpsBlock(&handle->packet.gps,
                                              &handle->instance->gps,
                                              handle->instance->metro.pressureAltitude);
                    }

                    /* Process the (valid) aux block. */
                    if (handle->crcOK[3] && handle->instance) {
                        _RS92_processAuxBlock(&handle->packet.aux, &handle->instance->metro, handle->instance);
                    }

                    /* Send out the results (if they make sense...) */
                    handle->instance->gps.climbRate = _RS92_estimateClimbRate(handle);
if(1){//                    if (handle->instance->gps.valid) {
                        _RS92_sendKiss(handle->instance);
                    }

                    if (handle->instance->logMode == RS92_LOGMODE_RAW) {
                        _RS92_sendRaw(handle->instance, handle->packet.rawData, sizeof(handle->packet.rawData));
                    }

                    LPCLIB_Event event;
                    LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
                    event.opcode = APP_EVENT_HEARD_SONDE;
                    event.block = SONDE_DETECTOR_RS41_RS92;
                    event.parameter = (void *)((uintptr_t)lrintf(rxFrequencyHz));
                    SYS_handleEvent(event);
                }
            }
        }
    }

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result RS92_resendLastPositions (RS92_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    RS92_InstanceData *instance = NULL;
    while (_RS92_iterateInstance(&instance)) {
        _RS92_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result RS92_removeFromList (RS92_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    RS92_InstanceData *instance = NULL;
    while (_RS92_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _RS92_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


/* Control logging */
LPCLIB_Result RS92_setLogMode (RS92_Handle handle, uint32_t id, RS92_LogMode mode)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;
    RS92_InstanceData *instance = NULL;
    while (_RS92_iterateInstance(&instance)) {
        if (instance->id == id) {
            instance->logMode = mode;
            result = LPCLIB_SUCCESS;
            break;
        }
    }

    return result;
}

