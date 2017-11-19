
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
    uint32_t special;
    char sSpecial[8];
    int length = 0;
    float f, offset;

    /* Get frequency */
    f = instance->config.frequencyKhz / 1000.0f;
    offset = SYS_getFrameOffsetKhz(sys);

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
        sprintf(sAltitude, "%.0f", instance->gps.observerLLA.alt + CONFIG_getGeoidHeight());
    }

    /* Battery voltage [V] */
    sVbat[0] = 0;
    if (!isnan(instance->metro.batteryVoltage)) {
        snprintf(sVbat, sizeof(sVbat), "%.3f", instance->metro.batteryVoltage);
    }

    /* DFM type indicator */
    special = 0;
    if (instance->platform == SONDE_DFM09) {
        if (instance->gps.inBurkinaFaso) {
            special += 4;
        }
        else {
            special += 8;
        }
    }
    else {
        if (instance->config.isPS15) {
            special += 32;
        }
        if (instance->config.isDFM06) {
            special += 16;
        }
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
        length = sprintf((char *)s, "%s,2,%.3f,,,,%s,%s,,,,,%s,,,,%.1f,%.1f,,,,",
                        instance->name,
                        f,                          /* Frequency [MHz] */
                        sAltitude,                  /* Altitude [m] */
                        sClimbRate,                 /* Climb rate [m/s] */
                        sSpecial,
                        SYS_getFrameRssi(sys),
                        offset                      /* RX frequency offset [kHz] */
                        );
    }
    else {
        length = sprintf((char *)s, "%s,2,%.3f,,%.5lf,%.5lf,%s,%s,%s,%s,,,%s,,,%.2f,%.1f,%.1f,%d,,,%s",
                        instance->name,
                        f,                          /* Frequency [MHz] */
                        latitude,                   /* Latitude [degrees] */
                        longitude,                  /* Longitude [degrees] */
                        sAltitude,                  /* Altitude [m] */
                        sClimbRate,                 /* Climb rate [m/s] */
                        sDirection,                 /* Direction [°] */
                        sVelocity,                  /* Horizontal speed [km/h] */
                        sSpecial,
                        instance->gps.hdop,
                        SYS_getFrameRssi(sys),
                        offset,                     /* RX frequency offset [kHz] */
                        instance->gps.usedSats,
                        sVbat                       /* Battery voltage [V] */
                        );
    }

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }
}



LPCLIB_Result DFM_processBlock (DFM_Handle handle, SONDE_Type type, void *buffer, uint32_t length, float rxFrequencyHz)
{
    LPCLIB_Result result = LPCLIB_ERROR;

    if (length >= 66) {  //TODO
        /* Convert to byte array */
        _DFM_buffer2raw((uint8_t *)buffer + 2*0, (uint8_t *)&handle->packet.config, 7);
        _DFM_buffer2raw((uint8_t *)buffer + 2*7, (uint8_t *)&handle->packet.gps[0], 13);
        _DFM_buffer2raw((uint8_t *)buffer + 2*(7+13), (uint8_t *)&handle->packet.gps[1], 13);

        if (_DFM_doParityCheck((uint8_t *)&handle->packet.config.raw, 7)) {
            _DFM_processConfigBlock(&handle->packet.config.raw, &handle->instance, rxFrequencyHz);
        }

        if (handle->instance) {
            handle->instance->platform = type;

            if (_DFM_doParityCheck((uint8_t *)&handle->packet.gps[0].raw, 13)) {
                _DFM_processGpsBlock(&handle->packet.gps[0].raw, handle->instance);
            }

            if (_DFM_doParityCheck((uint8_t *)&handle->packet.gps[1].raw, 13)) {
                _DFM_processGpsBlock(&handle->packet.gps[1].raw, handle->instance);
            }


            /* If there is a complete position update, send it out (once sonde name is known) */
            if (handle->instance->gps.newPosition) {
                handle->instance->gps.newPosition = false;
if(1){//                if (handle->instance->config.sondeNameKnown) {
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


/* Remove entries from heard list (select by frequency) */
LPCLIB_Result DFM_removeFromList (DFM_Handle handle, float rxFrequencyMHz)
{
    (void)handle;

    float rxKhz = roundf(rxFrequencyMHz * 1000.0f);
    DFM_InstanceData *instance = NULL;
    while (_DFM_iterateInstance(&instance)) {
        if (roundf(instance->rxFrequencyMHz * 1000.0f) == rxKhz) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }
            /* Remove sonde */
            _DFM_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


