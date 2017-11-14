
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
#include "rs41.h"
#include "rs41private.h"
#include "reedsolomon.h"
#include "gps.h"



/** Context */
typedef struct RS41_Context {
    uint8_t *pRawData;

    bool thisFrameNumberValid;
    bool thisGpsInfoValid;

    float rxFrequencyHz;
    int nCorrectedErrors;

    RS41_InstanceData *instance;

    RS41_SatInfo sats[12];

#if SEMIHOSTING_RS41
    FILE *fpAnalog;
    FILE *fpGps;
#endif
} RS41_Context;


//static LPCLIB_Result _RS41_handleEvent (LPCLIB_Event event);


static RS41_Context _rs41;


static const uint8_t bitreversal[256] = {
    0x00,0x80,0x40,0xC0,0x20,0xA0,0x60,0xE0,0x10,0x90,0x50,0xD0,0x30,0xB0,0x70,0xF0,
    0x08,0x88,0x48,0xC8,0x28,0xA8,0x68,0xE8,0x18,0x98,0x58,0xD8,0x38,0xB8,0x78,0xF8,
    0x04,0x84,0x44,0xC4,0x24,0xA4,0x64,0xE4,0x14,0x94,0x54,0xD4,0x34,0xB4,0x74,0xF4,
    0x0C,0x8C,0x4C,0xCC,0x2C,0xAC,0x6C,0xEC,0x1C,0x9C,0x5C,0xDC,0x3C,0xBC,0x7C,0xFC,
    0x02,0x82,0x42,0xC2,0x22,0xA2,0x62,0xE2,0x12,0x92,0x52,0xD2,0x32,0xB2,0x72,0xF2,
    0x0A,0x8A,0x4A,0xCA,0x2A,0xAA,0x6A,0xEA,0x1A,0x9A,0x5A,0xDA,0x3A,0xBA,0x7A,0xFA,
    0x06,0x86,0x46,0xC6,0x26,0xA6,0x66,0xE6,0x16,0x96,0x56,0xD6,0x36,0xB6,0x76,0xF6,
    0x0E,0x8E,0x4E,0xCE,0x2E,0xAE,0x6E,0xEE,0x1E,0x9E,0x5E,0xDE,0x3E,0xBE,0x7E,0xFE,
    0x01,0x81,0x41,0xC1,0x21,0xA1,0x61,0xE1,0x11,0x91,0x51,0xD1,0x31,0xB1,0x71,0xF1,
    0x09,0x89,0x49,0xC9,0x29,0xA9,0x69,0xE9,0x19,0x99,0x59,0xD9,0x39,0xB9,0x79,0xF9,
    0x05,0x85,0x45,0xC5,0x25,0xA5,0x65,0xE5,0x15,0x95,0x55,0xD5,0x35,0xB5,0x75,0xF5,
    0x0D,0x8D,0x4D,0xCD,0x2D,0xAD,0x6D,0xED,0x1D,0x9D,0x5D,0xDD,0x3D,0xBD,0x7D,0xFD,
    0x03,0x83,0x43,0xC3,0x23,0xA3,0x63,0xE3,0x13,0x93,0x53,0xD3,0x33,0xB3,0x73,0xF3,
    0x0B,0x8B,0x4B,0xCB,0x2B,0xAB,0x6B,0xEB,0x1B,0x9B,0x5B,0xDB,0x3B,0xBB,0x7B,0xFB,
    0x07,0x87,0x47,0xC7,0x27,0xA7,0x67,0xE7,0x17,0x97,0x57,0xD7,0x37,0xB7,0x77,0xF7,
    0x0F,0x8F,0x4F,0xCF,0x2F,0xAF,0x6F,0xEF,0x1F,0x9F,0x5F,0xDF,0x3F,0xBF,0x7F,0xFF,
};
static const uint8_t whitening[64] = {
    0x96,0x83,0x3E,0x51,0xB1,0x49,0x08,0x98,
    0x32,0x05,0x59,0x0E,0xF9,0x44,0xC6,0x26,
    0x21,0x60,0xC2,0xEA,0x79,0x5D,0x6D,0xA1,
    0x54,0x69,0x47,0x0C,0xDC,0xE8,0x5C,0xF1,
    0xF7,0x76,0x82,0x7F,0x07,0x99,0xA2,0x2C,
    0x93,0x7C,0x30,0x63,0xF5,0x10,0x2E,0x61,
    0xD0,0xBC,0xB4,0xB6,0x06,0xAA,0xF4,0x23,
    0x78,0x6E,0x3B,0xAE,0xBF,0x7B,0x4C,0xC1,
};


/* Do bit reversal, and remove data whitening */
static void _RS41_removeWhitening(uint8_t *buffer, int length)
{
    int i;

    /* Extract the two code words */
    for (i = 0; i < length; i++) {
        buffer[i] = bitreversal[buffer[i]] ^ whitening[(i + 8) % 64];
    }
}



/* Read 24-bit little-endian integer from memory */
static uint32_t _RS41_read24 (const uint8_t *p24)
{
    return p24[0] + 256 * p24[1] + 65536 * p24[2];
}



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



static void _RS41_readSubFrame7D (RS41_Handle handle, RS41_SubFrame7D *p)
{
    int i;

    /* Only use this subframe if we have previously decoded a GPS info subframe in the same frame. */
    if (handle->thisGpsInfoValid) {
        for (i = 0; i < 12; i++) {
            handle->sats[i].reserved000 = p->unknown005[i].reservedX0;
            handle->sats[i].reserved001 = (uint32_t)(_RS41_read24(p->unknown005[i].reservedX1));
            handle->sats[i].reserved004 = (int32_t)(_RS41_read24(p->unknown005[i].reservedX4) << 8) / 256;
        }
    }
}



//TODO
LPCLIB_Result RS41_open (RS41_Handle *pHandle)
{
    *pHandle = &_rs41;

#if SEMIHOSTING_RS41
    _rs41.fpAnalog = fopen("rs41_analog.csv", "w");
//    _rs41.fpGps = fopen("gps.csv", "w");
#endif

    return LPCLIB_SUCCESS;
}



/* Send position as a KISS packet */
static void _RS41_sendKiss (RS41_InstanceData *instance)
{
    char s[100];
    int length = 0;
    float offset;
    char sPressure[10];
    char sKillTimer[6];

    offset = 0;

    /* Pressure as string */
    sPressure[0] = 0;
    if (!isnan(instance->metro.pressure)) {
        snprintf(sPressure, sizeof(sPressure), "%.1f", instance->metro.pressure);
    }

    /* Kill timer (if active) */
    sKillTimer[0] = 0;
    if (_RS41_checkValidCalibration(instance, CALIB_KILLTIMER)) {
        if (instance->killTimer != (uint16_t)-1) {
            snprintf(sKillTimer, sizeof(sKillTimer), "%d", instance->killTimer);
        }
    }

    /* Convert lat/lon from radian to decimal degrees */
    double latitude = instance->gps.observerLLA.lat;
    double longitude = instance->gps.observerLLA.lon;
    float direction = instance->gps.observerLLA.direction;
    float velocity = instance->gps.observerLLA.velocity;
    if (!isnan(latitude) && !isnan(longitude)) {
        latitude *= 180.0 / M_PI;
        longitude *= 180.0 / M_PI;
        direction *= 180.0 / M_PI;
        velocity *= 3.6f;

        length = sprintf((char *)s, "%s,1,%.3f,,%.5lf,%.5lf,%.0f,%.1f,%.1f,%.1f,,%s,,,,,%.1f,%.1f,%d,%d,%s,",
                        instance->hashName,
                        instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                        latitude,                               /* Latitude [degrees] */
                        longitude,                              /* Longitude [degrees] */
                        instance->gps.observerLLA.alt,          /* Altitude [m] */
                        instance->gps.observerLLA.climbRate,    /* Climb rate [m/s] */
                        direction,                              /* Direction [°] */
                        velocity,                               /* Horizontal speed [km/h] */
                        sPressure,                              /* Pressure sensor [hPa] */
                        SYS_getFrameRssi(sys),
                        offset,                                 /* RX frequency offset [kHz] */
                        instance->gps.visibleSats,              /* # satellites */
                        instance->frameCounter,                 /* Current frame number */
                        sKillTimer                              /* Kill timer (frame) */
                        );
    }
    else if (instance->encrypted) {
        length = sprintf((char *)s, "%s,1,%.3f,,,,,,,,,,2,,,,%.1f,%.1f,%d,%d,%s,",
                        instance->hashName,
                        instance->rxFrequencyMHz,               /* RX frequency [MHz] */
                        SYS_getFrameRssi(sys),
                        offset,    /* RX frequency offset [kHz] */
                        instance->gps.visibleSats,              /* # satellites */
                        instance->frameCounter,                 /* Current frame number */
                        sKillTimer                              /* Kill timer (frame) */
                        );
    }

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }
}



static uint8_t _RS41_null;

//TODO: valid for short blocks only. Check 0F/F0 before decoding, or run both versions...
static uint8_t * _RS41_getDataAddressShort1 (int index)
{
    RS41_Handle handle = &_rs41;

    if (index < 132) {
        return &handle->pRawData[48 + 2 * index];
    }
    else if (index >= 231) {
        return &handle->pRawData[index - 231];
    }
    else {
        _RS41_null = 0;
        return &_RS41_null;
    }
}

static uint8_t * _RS41_getDataAddressShort2 (int index)
{
    RS41_Handle handle = &_rs41;

    if (index < 132) {
        return &handle->pRawData[49 + 2 * index];
    }
    else if (index >= 231) {
        return &handle->pRawData[index - 207];
    }
    else {
        _RS41_null = 0;
        return &_RS41_null;
    }
}

static uint8_t * _RS41_getDataAddressLong1 (int index)
{
    RS41_Handle handle = &_rs41;

    if (index < 231) {
        return &handle->pRawData[48 + 2 * index];
    }
    else {
        return &handle->pRawData[index - 231];
    }
}

static uint8_t * _RS41_getDataAddressLong2 (int index)
{
    RS41_Handle handle = &_rs41;

    if (index < 231) {
        return &handle->pRawData[49 + 2 * index];
    }
    else {
        return &handle->pRawData[index - 207];
    }
}



LPCLIB_Result RS41_processBlock (RS41_Handle handle, void *buffer, uint32_t length, float rxFrequencyHz)
{
    int numErrors;
    bool longFrame = false;


    handle->pRawData = buffer;

    if (length >= 312) {  //TODO
        /* Remove data whitening */
        _RS41_removeWhitening(buffer, length);

        /* Reed-Solomon error correction */
        if (REEDSOLOMON_process(_RS41_getDataAddressShort1, &numErrors) != LPCLIB_SUCCESS) {
            if (REEDSOLOMON_process(_RS41_getDataAddressLong1, &numErrors) != LPCLIB_SUCCESS) {
                return LPCLIB_ERROR;
            }
            longFrame = true;
        }
        handle->nCorrectedErrors = numErrors;
        if (REEDSOLOMON_process(longFrame ? _RS41_getDataAddressLong2 : _RS41_getDataAddressShort2, &numErrors) != LPCLIB_SUCCESS) {
            return LPCLIB_ERROR;
        }
        handle->nCorrectedErrors += numErrors;

        /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
        handle->rxFrequencyHz = rxFrequencyHz;

        /* Dump raw packet (SondeMonitor format) */
        if (0) {
            uint32_t i;
            char str[10];

            for (i = 0; i < length; i++) {
                sprintf(str, "%02X ", ((uint8_t *)buffer)[i]);
//                USBSerial_write(1, str, 3);
            }
//            USBSerial_write(1, "\r\n", 2);
        }

        /* Read packet type (pos 48, 0F or F0) */
        ;
        ;
        ;

        /* Run through buffer and process all sub-frames */
        {
            uint32_t i = 49;                /* Start after Reed-Solomon parity bytes and packet type indicator */
            uint8_t *p;
            uint8_t subFrameLength;
            uint16_t crc;

            handle->thisFrameNumberValid = false;
            handle->thisGpsInfoValid = false;

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
/*
                        if (handle->instance->rxFrequencyMHz == 0) {
                            handle->instance->rxFrequencyMHz = handle->rxFrequencyHz / 1e6f;
                        }
*/
                        handle->instance->rxFrequencyMHz = handle->rxFrequencyHz / 1e6f;
                        break;
                    case RS41_SUBFRAME_METROLOGY:
                        _RS41_processMetrologyBlock((RS41_SubFrameMetrology *)(p + 2), &handle->instance->metro, handle->instance);
#if SEMIHOSTING_RS41
                        if (handle->instance->gps.observerLLA.alt > 100) {
                            fprintf(handle->fpAnalog, "%u, %.0lf, ",
                                    handle->instance->frameCounter,
                                    handle->instance->gps.observerLLA.alt);
                            RS41_SubFrameMetrology *pm = (RS41_SubFrameMetrology *)(p + 2);
                            fprintf(handle->fpAnalog, "%lu, %lu, %lu, ",
                                    _RS41_read24(pm->adc[0].current),
                                    _RS41_read24(pm->adc[0].refmin),
                                    _RS41_read24(pm->adc[0].refmax));
                            fprintf(handle->fpAnalog, "%lu, %lu, %lu, ",
                                    _RS41_read24(pm->adc[1].current),
                                    _RS41_read24(pm->adc[1].refmin),
                                    _RS41_read24(pm->adc[1].refmax));
                            fprintf(handle->fpAnalog, "%lu, %lu, %lu, ",
                                    _RS41_read24(pm->adc[2].current),
                                    _RS41_read24(pm->adc[2].refmin),
                                    _RS41_read24(pm->adc[2].refmax));
                            fprintf(handle->fpAnalog, "%lu, %lu, %lu, ",
                                    _RS41_read24(pm->adc[3].current),
                                    _RS41_read24(pm->adc[3].refmin),
                                    _RS41_read24(pm->adc[3].refmax));
                            fprintf(handle->fpAnalog, "%u, %u, %u, ",
                                    pm->val12_16,
                                    pm->pressurePolyTwist,
                                    pm->val14_16);
                            fprintf(handle->fpAnalog, "%.7f\n",
                                    (float)(_RS41_read24(pm->adc[0].refmax) - _RS41_read24(pm->adc[0].current))
                                   /(float)(_RS41_read24(pm->adc[0].refmax) - _RS41_read24(pm->adc[0].refmin)));
                            fflush(handle->fpAnalog);
                        }
#endif
                        break;
                    case RS41_SUBFRAME_GPS_POSITION:
                        _RS41_processGpsPositionBlock((RS41_SubFrameGpsPosition *)(p + 2), &handle->instance->gps);

                        LPCLIB_Event event;
                        LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
                        event.opcode = APP_EVENT_HEARD_SONDE;
                        event.block = SONDE_DETECTOR_RS41_RS92;
                        event.parameter = (void *)((uint32_t)lrintf(rxFrequencyHz));
                        SYS_handleEvent(event);
                        break;
                    case RS41_SUBFRAME_GPS_INFO:
                        handle->thisGpsInfoValid = _RS41_processGpsInfoBlock(
                            (RS41_SubFrameGpsInfo *)(p + 2),
                            &handle->instance->gps);
                        break;
                    case RS41_SUBFRAME_7D:
                        _RS41_readSubFrame7D(handle, (RS41_SubFrame7D *)(p + 2));
                        break;
                    case RS41_SUBFRAME_CRYPT78:
                    case RS41_SUBFRAME_CRYPT80:
                        /* RS41-SGM */
                        handle->instance->gps.observerLLA.lat = NAN;
                        handle->instance->gps.observerLLA.lon = NAN;
                        handle->instance->encrypted = true;
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
        }

        /* Dump cooked data */
        if (0) {
            uint32_t i;
            char str[10];

            for (i = 0; i < length; i++) {
                sprintf(str, "%02X ", ((uint8_t *)buffer)[i]);
//                USBSerial_write(1, str, 3);
            }
//            USBSerial_write(1, "\r\n", 2);
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


/* Remove entries from heard list (select by frequency) */
LPCLIB_Result RS41_removeFromList (RS41_Handle handle, float rxFrequencyMHz)
{
    (void)handle;

    float rxKhz = roundf(rxFrequencyMHz * 1000.0f);
    RS41_InstanceData *instance = NULL;
    while (_RS41_iterateInstance(&instance)) {
        if (roundf(instance->rxFrequencyMHz * 1000.0f) == rxKhz) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }
            /* Remove sonde */
            _RS41_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

