
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "bsp.h"
#include "app.h"
#include "sys.h"
#include "rs92.h"
#include "rs92private.h"
#include "reedsolomon.h"
#include "linregression.h"


/** Context */
typedef struct RS92_Context {
    RS92_Packet packet;
    bool crcOK[4];

    float rxFrequencyHz;

    RS92_InstanceData *instance;

#if SEMIHOSTING_RS92
    FILE *fpAnalog;
    FILE *fpLog;
#endif
} RS92_Context;


static RS92_Context _rs92;

static const uint8_t manchester2bin[256] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x0F,0x07,0x00, 0x00,0x0B,0x03,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x0D,0x05,0x00, 0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x0E,0x06,0x00, 0x00,0x0A,0x02,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x0C,0x04,0x00, 0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};



/* Manchester decoding and stripping of start and stop bits */
static void _RS92_buffer2raw (RS92_Handle handle, uint8_t *buffer)
{
    uint32_t i, j;

    for (i = 0; i < sizeof(RS92_RawData); i+=2) {
        j = (5 * i) / 2;
        handle->packet.rawData[i+0] =
            manchester2bin[((buffer[j+0] << 2) | (buffer[j+1] >> 6)) & 0xFF]
         | (manchester2bin[((buffer[j+1] << 2) | (buffer[j+2] >> 6)) & 0xFF] << 4);
        handle->packet.rawData[i+1] =
            manchester2bin[((buffer[j+2] << 6) | (buffer[j+3] >> 2)) & 0xFF]
         | (manchester2bin[((buffer[j+3] << 6) | (buffer[j+4] >> 2)) & 0xFF] << 4);
    }
}


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


static void _RS92_getClimbSample (void *context, int index, float *x, float *y)
{
    RS92_Handle handle = (RS92_Handle)context;

    int idx = ((handle->instance->lastAltitudesWrIndex + RS92_CLIMB_HISTORY_LENGTH - 1) - index) % RS92_CLIMB_HISTORY_LENGTH;
    *x = handle->instance->lastAltitudes[idx].frame;
    *y = handle->instance->lastAltitudes[idx].alt;
}


static float _RS92_estimateClimbRate (RS92_Handle handle)
{
    float climbRate = NAN;

    if (!isnan(handle->instance->gps.observerLLA.alt)) {
        handle->instance->lastAltitudes[handle->instance->lastAltitudesWrIndex].alt = handle->instance->gps.observerLLA.alt;
        handle->instance->lastAltitudes[handle->instance->lastAltitudesWrIndex].frame = handle->instance->frameCounter;
        if (++handle->instance->lastAltitudesWrIndex >= RS92_CLIMB_HISTORY_LENGTH) {
            handle->instance->lastAltitudesWrIndex = 0;
        }

        // Linear regression.
        const LINREG_Input linreg_in = {
            .N = RS92_CLIMB_HISTORY_LENGTH,
            .get_data = _RS92_getClimbSample,
        };
        LINREG_Output linreg_out;
        LINREG_computeRegression(handle, &linreg_in, &linreg_out);

        climbRate = linreg_out.slope;
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

    /* Avoid sending the position if any of the values is undefined */
    if (isnan(latitude) || isnan(longitude)) {
        length = sprintf((char *)s, "%ld,0,%.3f,,,,%s,%s,,,%s,%s,,,,,%.1f,%.1f,%d,%d,%s,",
                        instance->id,
                        instance->rxFrequencyMHz,   /* Frequency [MHz] */
                        sAltitude,                  /* Altitude [m] */
                        sClimbRate,                 /* Climb rate [m/s] */
                        sTemperature,               /* Temperature [°C] */
                        sPressure,                  /* Pressure [hPa] */
                        SYS_getFrameRssi(sys),
                        offset,                     /* RX frequency offset [kHz] */
                        instance->gps.visibleSats,
                        instance->frameCounter,     /* Current frame number */
                        sKillTimer                  /* Kill timer (frame) */
                        );
    }
    else {
        length = sprintf((char *)s, "%ld,0,%.3f,%d,%.5lf,%.5lf,%s,%s,,,%s,%s,,,,%.2f,%.1f,%.1f,%d,%d,%s,",
                        instance->id,
                        instance->rxFrequencyMHz,   /* Frequency [MHz] */
                        instance->gps.usedSats,
                        latitude,                   /* Latitude [degrees] */
                        longitude,                  /* Longitude [degrees] */
                        sAltitude,                  /* Altitude [m] */
                        sClimbRate,                 /* Climb rate [m/s] */
                        sTemperature,               /* Temperature [°C] */
                        sPressure,                  /* Pressure [hPa] */
                        instance->gps.hdop,
                        SYS_getFrameRssi(sys),
                        offset,                     /* RX frequency offset [kHz] */
                        instance->gps.visibleSats,
                        instance->frameCounter,     /* Current frame number */
                        sKillTimer                  /* Kill timer (frame) */
                        );
    }

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = sprintf(s, "%ld,0,0,%s",
                instance->id,
                instance->hashName
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}



//TODO
LPCLIB_Result RS92_open (RS92_Handle *pHandle)
{
    *pHandle = &_rs92;

#if SEMIHOSTING_RS92
    _rs92.fpAnalog = fopen("rs92_analog.csv", "w");
    _rs92.fpLog = fopen("rs92.csv", "w");
#endif

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


/* Read 24-bit little-endian integer from memory */
#if SEMIHOSTING_RS92
static uint32_t _RS92_read24 (const uint8_t *p24)
{
    return p24[0] + 256 * p24[1] + 65536 * p24[2];
}
#endif


LPCLIB_Result RS92_processBlock (RS92_Handle handle, void *buffer, uint32_t length, float rxFrequencyHz)
{
    int nErrors;

    if (length > 1) {  //TODO
        /* Convert to byte array */
        _RS92_buffer2raw(handle, buffer);

        /* Reed-Solomon decoder */
        LPCLIB_Result result = REEDSOLOMON_process(_RS92_getDataAddress, &nErrors); 
        if (result != LPCLIB_SUCCESS) {
            /* Try harder. Guess some elements of the code word and try again. */
            handle->packet.config.frameType = RS92_SUBFRAME_CALIB_CONFIG;
            handle->packet.config.length = (sizeof(handle->packet.config) - 4) / 2;
            handle->packet.config.name[0] = ' ';
            handle->packet.config.name[1] = ' ';
            if (handle->instance) {
                int i;
                for (i = 0; i < 8; i++) {
                    handle->packet.config.name[2 + i] = handle->instance->hashName[i];
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
            result = REEDSOLOMON_process(_RS92_getDataAddress, &nErrors);
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

                    /* Process the (valid) metrology block only if there is calibration data available. */
                    if (handle->crcOK[1] && handle->instance) {
                        _RS92_processMetrologyBlock(&handle->packet.metrology, &handle->instance->metro, handle->instance);
                    }

#if SEMIHOSTING_RS92
                    if (!isnan(handle->instance->metro.pressureAltitude)) {
                        fprintf(handle->fpAnalog, "%u, %.0f, ",
                                handle->instance->frameCounter,
                                handle->instance->metro.pressureAltitude);
                        struct _RS92_MetrologyBlock *pm = &handle->packet.metrology;
                        fprintf(handle->fpAnalog, "%lu, %lu, %lu, ",
                                _RS92_read24(pm->T),
                                _RS92_read24(pm->U1),
                                _RS92_read24(pm->U2));
                        fprintf(handle->fpAnalog, "%lu, %lu, ",
                                _RS92_read24(pm->REF1),
                                _RS92_read24(pm->REF2));
                        fprintf(handle->fpAnalog, "%lu, %lu, %lu ",
                                _RS92_read24(pm->P),
                                _RS92_read24(pm->REF3),
                                _RS92_read24(pm->REF4));
                        fprintf(handle->fpAnalog, "\n");
                        fflush(handle->fpAnalog);
                    }
#endif

                    /* Process the (valid) GPS block */
                    if (handle->crcOK[2]) {
                        _RS92_processGpsBlock(&handle->packet.gps,
                                              &handle->instance->gps,
                                              handle->instance->metro.pressureAltitude);
                    }

                    /* Send out the results (if they make sense...) */

float climbRate = _RS92_estimateClimbRate(handle);
if (!isnan(climbRate)) {
    handle->instance->gps.climbRate = climbRate;
}
if(1){//                    if (handle->instance->gps.valid) {
                        _RS92_sendKiss(handle->instance);
                    }

#if SEMIHOSTING_RS92
                    {
                        char s[40];
                        sprintf(s, "%d,%f\n", handle->instance->frameCounter, handle->instance->gps.observerLLA.alt);
                        fwrite(s, strlen(s), 1, handle->fpLog);
                    }
#endif

                    LPCLIB_Event event;
                    LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
                    event.opcode = APP_EVENT_HEARD_SONDE;
                    event.block = SONDE_DETECTOR_RS41_RS92;
                    event.parameter = (void *)((uint32_t)lrintf(rxFrequencyHz));
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
LPCLIB_Result RS92_removeFromList (RS92_Handle handle, uint32_t id)
{
    (void)handle;

    RS92_InstanceData *instance = NULL;
    while (_RS92_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }
            /* Remove sonde */
            _RS92_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

