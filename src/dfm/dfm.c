
#include <math.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "config.h"
#include "sys.h"
#include "dfm.h"
#include "dfmprivate.h"




static const uint8_t _DFM_manchester2bin[256] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00, 0x00,0x02,0x03,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x04,0x05,0x00, 0x00,0x06,0x07,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x08,0x09,0x00, 0x00,0x0A,0x0B,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x0C,0x0D,0x00, 0x00,0x0E,0x0F,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};


/* Generator matrix used for Hamming(8,4) code:
 *
 *     /  1 0 0 0 0 1 1 1  \
 * G = |  0 1 0 0 1 0 1 1  |
 *     |  0 0 1 0 1 1 0 1  |
 *     \  0 0 0 1 1 1 1 0  /
 */

/* Table based parity check.
 * Table index is the received code word. If the entry has bit 7 set,
 * the code word is invalid (two bit errors).
 * Otherwise the table entry is the decoded message (up to 1 corrected bit error).
 */
static const uint8_t _DFM_parityCheckTable[256] = {
    0x00,0x00,0x00,0x80,0x00,0x80,0x80,0x08,
    0x00,0x80,0x80,0x04,0x80,0x02,0x01,0x80,
    0x00,0x80,0x80,0x03,0x80,0x05,0x01,0x80,
    0x80,0x09,0x01,0x80,0x01,0x80,0x01,0x01,
    0x00,0x80,0x80,0x03,0x80,0x02,0x06,0x80,
    0x80,0x02,0x0A,0x80,0x02,0x02,0x80,0x02,
    0x80,0x03,0x03,0x03,0x0B,0x80,0x80,0x03,
    0x07,0x80,0x80,0x03,0x80,0x02,0x01,0x80,
    0x00,0x80,0x80,0x04,0x80,0x05,0x06,0x80,
    0x80,0x04,0x04,0x04,0x0C,0x80,0x80,0x04,
    0x80,0x05,0x0D,0x80,0x05,0x05,0x80,0x05,
    0x07,0x80,0x80,0x04,0x80,0x05,0x01,0x80,
    0x80,0x0E,0x06,0x80,0x06,0x80,0x06,0x06,
    0x07,0x80,0x80,0x04,0x80,0x02,0x06,0x80,
    0x07,0x80,0x80,0x03,0x80,0x05,0x06,0x80,
    0x07,0x07,0x07,0x80,0x07,0x80,0x80,0x0F,
    0x00,0x80,0x80,0x08,0x80,0x08,0x08,0x08,
    0x80,0x09,0x0A,0x80,0x0C,0x80,0x80,0x08,
    0x80,0x09,0x0D,0x80,0x0B,0x80,0x80,0x08,
    0x09,0x09,0x80,0x09,0x80,0x09,0x01,0x80,
    0x80,0x0E,0x0A,0x80,0x0B,0x80,0x80,0x08,
    0x0A,0x80,0x0A,0x0A,0x80,0x02,0x0A,0x80,
    0x0B,0x80,0x80,0x03,0x0B,0x0B,0x0B,0x80,
    0x80,0x09,0x0A,0x80,0x0B,0x80,0x80,0x0F,
    0x80,0x0E,0x0D,0x80,0x0C,0x80,0x80,0x08,
    0x0C,0x80,0x80,0x04,0x0C,0x0C,0x0C,0x80,
    0x0D,0x80,0x0D,0x0D,0x80,0x05,0x0D,0x80,
    0x80,0x09,0x0D,0x80,0x0C,0x80,0x80,0x0F,
    0x0E,0x0E,0x80,0x0E,0x80,0x0E,0x06,0x80,
    0x80,0x0E,0x0A,0x80,0x0C,0x80,0x80,0x0F,
    0x80,0x0E,0x0D,0x80,0x0B,0x80,0x80,0x0F,
    0x07,0x80,0x80,0x0F,0x80,0x0F,0x0F,0x0F
};


/** Context */
typedef struct DFM_Context {
    DFM_Packet packet;

    DFM_InstanceData *instance;
    float rxFrequencyHz;
} DFM_Context;

static DFM_Context _dfm;


/* Manchester decoding */
static void _DFM_buffer2raw (uint8_t *buffer, uint8_t *out, uint8_t length)
{
    uint16_t i, j;

    memset(out, 0, length);

    for (i = 0; i < length; i++) {
        /* Get next two RX bytes and Manchester decode into a data byte */
        uint8_t byte = (_DFM_manchester2bin[buffer[2 * i + 0]] << 4) | _DFM_manchester2bin[buffer[2 * i + 1]];

        /* Deinterleave */
        for (j = 0; j < 8; j++) {
            int offset = (8 * i + j) % length;
            int bitpos = 7 - ((8 * i + j) / length);

            if (byte & (1 << (7 - j))) {
                out[offset] |= (1 << bitpos);
            }
        }
    }
}



/* Do a parity check on the Hamming (8,4) code words in the buffer.
 * Replace them by the decoded nibbles.
 * Return false if any one code word is not correctable.
 */
static bool _DFM_doParityCheck (uint8_t *buffer, int length)
{
    bool result = true;
    int n;
    uint8_t val;


    for (n = 0; n < length; n++) {
        val = _DFM_parityCheckTable[buffer[n]];
        if (val & 0x80) {
            result = false;
            break;
        }

        buffer[n] = val;
    }

    return result;
}


//TODO
LPCLIB_Result DFM_open (DFM_Handle *pHandle)
{
    *pHandle = &_dfm;

    return LPCLIB_SUCCESS;
}



/* Send position report */
static void _DFM_sendKiss (DFM_InstanceData *instance)
{
    char s[120];
    char sAltitude[20];
    char sClimbRate[20];
    char sVelocity[8];
    char sDirection[8];
    char sVbat[8];
    char sTemperature[8];
    uint32_t special;
    char sSpecial[8];
    int length = 0;
    float f;

    /* Pressure/temperature/humidity sensors */
    sTemperature[0] = 0;
    if (!isnan(instance->metro.temperature)) {
        snprintf(sTemperature, sizeof(sTemperature), "%.1f", instance->metro.temperature);
    }

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

    /* Battery voltage [V] */
    sVbat[0] = 0;
    if (!isnan(instance->metro.batteryVoltage)) {
        snprintf(sVbat, sizeof(sVbat), "%.3f", instance->metro.batteryVoltage);
    }

    /* DFM type indicator */
    special = 0;
    switch (instance->model) {
        case DFM_MODEL_DFM09_AFRICA:
            special |= (1u << 2);
            break;
        case DFM_MODEL_DFM09_OLD:
        case DFM_MODEL_DFM09_NEW:
            special |= (1u << 3);
            break;
        case DFM_MODEL_DFM06_OLD:
        case DFM_MODEL_DFM06_NEW:
            special |= (1u << 4);
            break;
        case DFM_MODEL_PS15:
            special |= (1u << 5);
            break;
        case DFM_MODEL_DFM17:
            special |= (1u << 9);
            break;
        case DFM_MODEL_UNKNOWN:
            /* Nothing to do */
            break;
    }
    snprintf(sSpecial, sizeof(sSpecial), "%lu", special);

    /* Climb rate and ground speed may not be available (Burkina Faso version) */
    sClimbRate[0] = 0;
    if (!isnan(instance->gps.climbRate)) {
        sprintf(sClimbRate, "%.1f", instance->gps.climbRate);
    }
    sVelocity[0] = 0;
    if (!isnan(instance->gps.observerLLA.velocity)) {
        snprintf(sVelocity, sizeof(sVelocity), "%.1f", instance->gps.observerLLA.velocity * 3.6f);
    }
    sDirection[0] = 0;
    if (!isnan(instance->gps.observerLLA.direction)) {
        snprintf(sDirection, sizeof(sDirection), "%.1f", instance->gps.observerLLA.direction * (180.0f / M_PI));
    }

    /* Avoid sending the position if any of the values is undefined */
    if (isnan(latitude) || isnan(longitude)) {
        length = sprintf((char *)s, "%ld,2,%.3f,,,,%s,%s,,,%s,,%s,,,,%.1f,,,,,",
                        instance->id,
                        f,                          /* Frequency [MHz] */
                        sAltitude,                  /* Altitude [m] */
                        sClimbRate,                 /* Climb rate [m/s] */
                        sTemperature,               /* Temperature [°C] */
                        sSpecial,
                        SYS_getFrameRssi(sys)
                        );
    }
    else {
        length = sprintf((char *)s, "%ld,2,%.3f,,%.5lf,%.5lf,%s,%s,%s,%s,%s,,%s,,,,%.1f,,%d,,,%s",
                        instance->id,
                        f,                          /* Frequency [MHz] */
                        latitude,                   /* Latitude [degrees] */
                        longitude,                  /* Longitude [degrees] */
                        sAltitude,                  /* Altitude [m] */
                        sClimbRate,                 /* Climb rate [m/s] */
                        sDirection,                 /* Direction [°] */
                        sVelocity,                  /* Horizontal speed [km/h] */
                        sTemperature,               /* Temperature [°C] */
                        sSpecial,
                        SYS_getFrameRssi(sys),
                        instance->gps.usedSats,
                        sVbat                       /* Battery voltage [V] */
                        );
    }

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = sprintf(s, "%ld,2,0,%s,%.1f",
                instance->id,
                instance->name,
                instance->gps.ehpe
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}



LPCLIB_Result DFM_processBlock (
        DFM_Handle handle,
        SONDE_Type type,
        void *buffer,
        uint32_t length,
        float rxFrequencyHz,
        uint32_t rxTime)
{
    LPCLIB_Result result = LPCLIB_ERROR;

    if (length >= 66) {  //TODO
        /* Convert to byte array */
        _DFM_buffer2raw((uint8_t *)buffer + 2*0, (uint8_t *)&handle->packet.config, 7);
        _DFM_buffer2raw((uint8_t *)buffer + 2*7, (uint8_t *)&handle->packet.gps[0], 13);
        _DFM_buffer2raw((uint8_t *)buffer + 2*(7+13), (uint8_t *)&handle->packet.gps[1], 13);

        DFM_InstanceData *instance = NULL;

        if (_DFM_doParityCheck((uint8_t *)&handle->packet.gps[0].raw, 13)) {
            _DFM_processGpsBlock(&handle->packet.gps[0].raw, &instance, rxFrequencyHz, rxTime, type);
        }

        if (_DFM_doParityCheck((uint8_t *)&handle->packet.gps[1].raw, 13)) {
            _DFM_processGpsBlock(&handle->packet.gps[1].raw, &instance, rxFrequencyHz, rxTime, type);
        }

        if (_DFM_doParityCheck((uint8_t *)&handle->packet.config.raw, 7)) {
            _DFM_processConfigBlock(&handle->packet.config.raw, instance, rxFrequencyHz, rxTime);
        }

        if (instance) {
            handle->instance = instance;
        }

        if (handle->instance) {
            handle->instance->platform = type;

            /* Log */
            char log[60];
            uint32_t confval = 0
                    | (handle->packet.config.h[0] << 24)
                    | (handle->packet.config.h[1] << 20)
                    | (handle->packet.config.h[2] << 16)
                    | (handle->packet.config.h[3] << 12)
                    | (handle->packet.config.h[4] <<  8)
                    | (handle->packet.config.h[5] <<  4)
                    | (handle->packet.config.h[6] <<  0)
                    ;
            uint32_t gps0val1 = 0
                    | (handle->packet.gps[0].d1[0] << 16)
                    | (handle->packet.gps[0].d1[1] << 12)
                    | (handle->packet.gps[0].d1[2] <<  8)
                    | (handle->packet.gps[0].d1[3] <<  4)
                    | (handle->packet.gps[0].d1[4] <<  0)
                    ;
            uint32_t gps0val2 = 0
                    | (handle->packet.gps[0].d1[5]  << 28)
                    | (handle->packet.gps[0].d1[6]  << 24)
                    | (handle->packet.gps[0].d1[7]  << 20)
                    | (handle->packet.gps[0].d1[8]  << 16)
                    | (handle->packet.gps[0].d1[9]  << 12)
                    | (handle->packet.gps[0].d1[10] <<  8)
                    | (handle->packet.gps[0].d1[11] <<  4)
                    | (handle->packet.gps[0].d1[12] <<  0)
                    ;
            uint32_t gps1val1 = 0
                    | (handle->packet.gps[1].d1[0] << 16)
                    | (handle->packet.gps[1].d1[1] << 12)
                    | (handle->packet.gps[1].d1[2] <<  8)
                    | (handle->packet.gps[1].d1[3] <<  4)
                    | (handle->packet.gps[1].d1[4] <<  0)
                    ;
            uint32_t gps1val2 = 0
                    | (handle->packet.gps[1].d1[5]  << 28)
                    | (handle->packet.gps[1].d1[6]  << 24)
                    | (handle->packet.gps[1].d1[7]  << 20)
                    | (handle->packet.gps[1].d1[8]  << 16)
                    | (handle->packet.gps[1].d1[9]  << 12)
                    | (handle->packet.gps[1].d1[10] <<  8)
                    | (handle->packet.gps[1].d1[11] <<  4)
                    | (handle->packet.gps[1].d1[12] <<  0)
                    ;
            snprintf(log, sizeof(log), "%s,2,1,%07lX%05lX%08lX%05lX%08lX",
                        handle->instance->name,
                        confval,
                        gps0val1,
                        gps0val2,
                        gps1val1,
                        gps1val2);
            SYS_send2Host(HOST_CHANNEL_INFO, log);

            /* If there is a complete position update, send it out */
            if (handle->instance->gps.newPosition) {
                handle->instance->gps.newPosition = false;
                _DFM_sendKiss(handle->instance);

                LPCLIB_Event event;
                LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
                event.opcode = APP_EVENT_HEARD_SONDE;
                event.block = SONDE_DETECTOR_DFM;
                event.parameter = (void *)((uint32_t)lrintf(rxFrequencyHz));
                SYS_handleEvent(event);

                result = LPCLIB_SUCCESS;
            }
        }

        /* Remember RX frequency (difference to nominal sonde frequency will be reported as frequency offset) */
        handle->rxFrequencyHz = rxFrequencyHz;
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result DFM_resendLastPositions (DFM_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    DFM_InstanceData *instance = NULL;
    while (_DFM_iterateInstance(&instance)) {
        _DFM_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result DFM_removeFromList (DFM_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    DFM_InstanceData *instance = NULL;
    while (_DFM_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _DFM_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


