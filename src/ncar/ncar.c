
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "gps.h"
#include "ncar.h"
#include "ncarprivate.h"
#include "sys.h"

/*
 * NCAR RD41/RD94 frame parameters
 *
 * 2FSK, 4800 symbols/s, 2.4 kHz deviation, continuous framed transmission.
 * Frame duration is 500ms, i.e. 2400 symbols/frame. Manchester encoding (IEEE 802.2) is used
 * --> bit rate = 1200 bits/s.
 * For unknown reasons, all data is wrapped as 8N1 UART characters (10 bits), so there are
 * 120 characters (bytes) in a frame. Four characters are reserved as a frame sync pattern, and that
 * pattern can either be detected as four bytes after UART deframing or as a raw 40-bit
 * (80 symbols) sequence.
 *
 * The payload (116 characters) is divided into several blocks of different but known length.
 * Each block carries a 16-bit checksum to verify its integrity. There is no forward error correction.
 * The number of blocks, their size, and the implementation of the checksum are different for
 * different sonde models (RD41/RD94):
 * 
 * -----+ +-------------+ +--------+---------+--- ... ---+---------+ +-----
 *      | |     Sync    | |   B1   |   B2    |           |   Bn    | |
 *  ... | | 1A CF FC 1D | |        |         |           |         | | ...
 *      | | 80 symbols  | | 5 chrs | x chrs  |           | y chrs  | |
 * -----+ +-------------+ +--------+---------+--- ... ---+---------+ +-----
 *                        \---  116 characters = 2320 symbols  ----/
 *        \----------  2400 symbols = 500 milliseconds  -----------/
 *
 * The sync pattern as 80 raw symbols:
 *   10 1001100101101010 01 10 0101010110100101 01 10 1010010101010101 01 10 0110010101101010 01
 * The sync pattern as 40 bits after Manchester decoding:
 *   0 01011000 1 0 11110011 1 0 00111111 1 0 10111000 1
 * The sync pattern as 4 bytes after UART unwrapping:
 *   1A CF FC 1D
 *
 * Ra uses symbol level detection of the sync pattern.
 *
 * RD41 payload format:
 * -----+ +-------------+ +--------+---------+---------+---------+---------+---------+---------+ +-----
 *      | |     Sync    | |   B1   |   B2    |   B3    |   B4    |   B5    |   B6    |   B7    | |
 *  ... | | 1A CF FC 1D | |        |         |         |         |         |         |         | | ...
 *      | | 80 symbols  | | 5 chrs | 18 chrs | 19 chrs | 14 chrs | 15 chrs | 29 chrs | 16 chrs | |
 * -----+ +-------------+ +--------+---------+---------+---------+---------+---------+---------+ +-----
 *                        \-----------  116 characters = 1160 bits = 2320 symbols  ------------/
 *        \------------------------  2400 symbols = 500 milliseconds  -------------------------/
 *
 * Payload blocks:
 * B1: Sonde type; Frame number
 * B2: PTU
 * B3: GPS
 * B4: GPS
 * B5: GPS
 * B6: GPS
 * B7: Serial number; inner temperature; ...
 *
 * The last two characters (bytes) in each N-byte block contain a 16-bit big endian CRC of the
 * first N-2 bytes of the block. The CRC uses the CCITT polynomial (x^16 + x^12 + x^5 + 1), a seed
 * of 0, and an output-XOR of 0.
 * CRC verification: Calculate the CRC over the first N-2 bytes of a block, then compare with the
 * value received in the last two bytes of the block. For the CRC parameters used here, a simpler
 * approach is possible: Calculate the CRC over all N bytes of the block (including the received
 * CRC), and the result for a valid block must always be zero.
 *
 * RD94 payload format:
 * -----+ +-------------+ +--------+---------+---------+---------+---------+ +-----
 *      | |     Sync    | |   B1   |   B2    |   B3    |   B4    |   B5    | |
 *  ... | | 1A CF FC 1D | |        |         |         |         |         | | ...
 *      | | 80 symbols  | | 5 chrs | 19 chrs | 49 chrs | 20 chrs | 23 chrs | |
 * -----+ +-------------+ +--------+---------+---------+---------+---------+ +-----
 *                        \--  116 characters = 1160 bits = 2320 symbols  -/
 *        \--------------  2400 symbols = 500 milliseconds  ---------------/
 *
 * Payload blocks:
 * B1: Sonde type; Frame number
 * B2: PTU
 * B3: GPS
 * B4: GPS
 * B5: Serial number; inner temperature; ...
 *
 * The last two characters (bytes) in each N-byte block contain a 16-bit big endian CRC of the
 * first N-2 bytes of the block. The CRC uses the CCITT polynomial (x^16 + x^12 + x^5 + 1), a seed
 * of 0, and an output-XOR of 0.
 * CRC verification: Calculate the CRC over the first N-2 bytes of a block, then compare with the
 * value received in the last two bytes of the block. For the CRC parameters used here, a simpler
 * approach is possible: Calculate the CRC over all N bytes of the block (including the received
 * CRC), and the result for a valid block must always be zero.
 *
 * Notes:
 * - No calibration data is transmitted, all PTU values are final physical units.
 * - Wind data (horizontal wind vector, vertical speed, altitude) are sent in both B3 and B5. The
 *   effective wind sample rate is 2 samples/frame or 4 samples/second.
 * - GPS altitude is not the WGS84 ellipsoid height, but the height above mean sea level (MSL).
 *   The local difference between WGS84 and the geoid ("undulation") is estimated by the sonde
 *   and included in the result.
 */

/** Context */
typedef struct NCAR_Context {
    RD41_RawFrame *pRawData;

    float rxFrequencyHz;

    NCAR_InstanceData *instance;
} NCAR_Context;


static NCAR_Context _ncar;


//TODO
LPCLIB_Result NCAR_open (NCAR_Handle *pHandle)
{
    *pHandle = &_ncar;

    return LPCLIB_SUCCESS;
}


/* Send position as a KISS packet */
static void _NCAR_sendKiss (NCAR_InstanceData *instance)
{
    static char s[160];
    int length = 0;
    char sPressure[10];
    uint32_t special;

    special = 0;

    /* Pressure as string */
    sPressure[0] = 0;
    if (!isnan(instance->metro.pressure)) {
        snprintf(sPressure, sizeof(sPressure), "%.1f", instance->metro.pressure);
    }

    /* Flags */
    //special += 1u << 2;     /* Logging active */
    special += 1u << 3;     /* RD41 */

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

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",24,%.3f,%d,%.5lf,%.5lf,%.0f,%.1f,%.1f,%.1f,%.1f,%s,%"PRIu32",,%.1f,,%.1f,,%d,%d,,%.1f,,%.3lf,%.2lf",
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
                    instance->rssi,
                    instance->gps.visibleSats,              /* # satellites */
                    instance->frameCounter,                 /* Current frame number */
                    instance->batteryVoltage,               /* Battery voltage [V] */
                    instance->gps.gpstime,                  /* GPS time */
                    instance->realTime * 0.01
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",24,0,%s",
                instance->id,
                instance->name
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}



static char _ncar_rawCompressed[156+42];    /* 156: Max UUENCODE length, 42: some overhead... */

static void _NCAR_sendRaw (NCAR_InstanceData *instance, uint8_t *buffer, uint32_t length)
{
    uint32_t i = 0;
    int j;
    uint8_t uu[4+1];
    int N = ((length + 1) / 3) * 3;
    int n;
    int slen = 0;
    char *s = _ncar_rawCompressed;


    slen += snprintf(&s[slen], sizeof(_ncar_rawCompressed) - slen, "%"PRIu32",24,1,%"PRIu32",%d,",
                     instance->id,
                     length,
                     0
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

        slen += snprintf(&s[slen], sizeof(_ncar_rawCompressed) - slen, "%s", uu);
    }

    SYS_send2Host(HOST_CHANNEL_INFO, s);
}


LPCLIB_Result NCAR_processBlock (
        NCAR_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime)
{
    /* Expect 116 bytes after UART unwrapping */
    if (numBits != 116*8) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    handle->pRawData = buffer;

    /* Limit buffer length in case of short frame */
    uint32_t length = 116;

    /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
    handle->rxFrequencyHz = rxFrequencyHz;

    /* Run through buffer and process all sub-frames */
    {
        RD41_RawFrame *p = handle->pRawData;

        /* Check the first block to determine the sonde type. Sonde type indicator and checksum
         * must match.
         */
        if (_RD41_checkCRC(&p->frameNumberRD41, sizeof(p->frameNumberRD41))) {
            if (p->frameNumberRD41.sondeType == 3) {
                if (_RD41_checkCRC(&p->config, sizeof(p->config))) {
                    _RD41_processConfigBlocks(&p->frameNumberRD41, &p->config, &handle->instance);
                }
                if (handle->instance) {
                    if (_RD41_checkCRC(&p->metrology, sizeof(p->metrology))) {
                        _RD41_processMetrologyBlock(&p->metrology, handle->instance);
                    }
                    if (_RD41_checkCRC(&p->gps1, sizeof(p->gps1))) {
                        _RD41_processGps1Block(&p->gps1, &handle->instance->gps);
                    }
                    if (_RD41_checkCRC(&p->gps2, sizeof(p->gps2)) && 
                        _RD41_checkCRC(&p->gps3, sizeof(p->gps3))) {
                        _RD41_processGps2_3Block(&p->gps2, &p->gps3, &handle->instance->gps);
                    }
                    if (_RD41_checkCRC(&p->gps4, sizeof(p->gps4))) {
                        _RD41_processGps4Block(&p->gps4, &handle->instance->gps);
                    }
                    if (_RD41_checkCRC(&p->config, sizeof(p->config))) {
                        _RD41_processMetrologyBlock(&p->metrology, handle->instance);
                    }

                    handle->instance->rssi = rssi;
                    handle->instance->realTime = realTime;
                    handle->instance->rxFrequencyMHz = handle->rxFrequencyHz / 1e6f;

                    _NCAR_sendKiss(handle->instance);
                    if (handle->instance->logMode == NCAR_LOGMODE_RAW) {
                        _NCAR_sendRaw(handle->instance, buffer, length);
                    }
                }
            }
        }
        if (_RD94_checkChecksum(&p->frameNumberRD94, sizeof(p->frameNumberRD94), p->frameNumberRD94.sum1, p->frameNumberRD94.sum2)) {
            if (p->frameNumberRD94.sondeType == 1) {
            }
        }
    }

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result NCAR_resendLastPositions (NCAR_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    NCAR_InstanceData *instance = NULL;
    while (_NCAR_iterateInstance(&instance)) {
        _NCAR_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result NCAR_removeFromList (NCAR_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    NCAR_InstanceData *instance = NULL;
    while (_NCAR_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;
            
            /* Remove sonde */
            _NCAR_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


/* Control logging */
LPCLIB_Result NCAR_setLogMode (NCAR_Handle handle, uint32_t id, NCAR_LogMode mode)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;
    NCAR_InstanceData *instance = NULL;
    while (_NCAR_iterateInstance(&instance)) {
        if (instance->id == id) {
            instance->logMode = mode;
            result = LPCLIB_SUCCESS;
            break;
        }
    }

    return result;
}

