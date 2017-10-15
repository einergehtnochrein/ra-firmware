
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
#include "imet.h"
#include "imetprivate.h"



/** Context */
typedef struct IMET_Context {
    IMET_InstanceData *instance;
    float rxFrequencyHz;
} IMET_Context;

static IMET_Context _imet;


/* Verify the packet checksum.
 * The last two payload bytes contain the CRC.
 */
static bool _IMET_doParityCheck (uint8_t *buffer, uint8_t length)
{
    CRC_Handle crc = LPCLIB_INVALID_HANDLE;
    CRC_Mode crcMode;
    uint16_t receivedCRC;
    bool result = false;

    crcMode = CRC_makeMode(
            CRC_POLY_CRCCCITT,
            CRC_DATAORDER_NORMAL,
            CRC_SUMORDER_NORMAL,
            CRC_DATAPOLARITY_NORMAL,
            CRC_SUMPOLARITY_NORMAL
            );
    if (CRC_open(crcMode, &crc) == LPCLIB_SUCCESS) {
//        CRC_seed(crc, 0x1D0F);
        CRC_seed(crc, 0xDCBD);  /* Seed=0x1D0F plus SOH byte (0x01) */
        CRC_write(crc, buffer, length - 2, NULL, NULL);
        receivedCRC = (buffer[length-2] << 8) | buffer[length-1];
        result = receivedCRC == CRC_read(crc);
printf("c %04X %04lX\r\n", receivedCRC, CRC_read(crc));
        CRC_close(&crc);
    }

    return result;
}


//TODO
LPCLIB_Result IMET_open (IMET_Handle *pHandle)
{
    *pHandle = &_imet;
    IMET_DSP_initAudio();

    return LPCLIB_SUCCESS;
}



/* Send position report */
static void _IMET_sendKiss (IMET_InstanceData *instance)
{
    char s[120];
    char sAltitude[20];
    char sClimbRate[16];
    char *sType;
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
        sprintf(sAltitude, "%.0f", instance->gps.observerLLA.alt);
    }

    /* iMet-1 type indicator */
    sType = "6";

    /* Don't send climb rate if it is undefined */
    sClimbRate[0] = 0;
    if (!isnan(instance->gps.climbRate)) {
        sprintf(sClimbRate, "%.1f", instance->gps.climbRate);
    }

    /* Avoid sending the position if any of the values is undefined */
    if (isnan(latitude) || isnan(longitude)) {
        length = sprintf((char *)s, "%s,%s,%.3f,,,,%s,%s,%.1f,%.1f,,,,,,,%.1f,%.1f,0",
                        instance->name,
                        sType,
                        f,                          /* Frequency [MHz] */
                        sAltitude,                  /* Altitude [m] */
                        sClimbRate,                 /* Climb rate [m/s] */
                        0.0f,                       /* Direction [°] */
 0.0f,//                       instance->gps.groundSpeed,  /* Horizontal speed [km/h] */
                        SYS_getFrameRssi(sys),
                        offset                      /* RX frequency offset [kHz] */
                        );
    }
    else {
        length = sprintf((char *)s, "%s,%s,%.3f,,%.5lf,%.5lf,%s,%s,%.1f,%.1f,,,,,,,%.1f,%.1f,%d",
                        instance->name,
                        sType,
                        f,                          /* Frequency [MHz] */
                        latitude,                   /* Latitude [degrees] */
                        longitude,                  /* Longitude [degrees] */
                        sAltitude,                  /* Altitude [m] */
                        sClimbRate,                 /* Climb rate [m/s] */
                        0.0f,                       /* Direction [°] */
 0.0f,//                       instance->gps.groundSpeed,  /* Horizontal speed [km/h] */
                        SYS_getFrameRssi(sys),
                        offset,                     /* RX frequency offset [kHz] */
                        instance->gps.usedSats
                        );
    }

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }
}



LPCLIB_Result IMET_processBlock (IMET_Handle handle, void *buffer, uint32_t length, float rxFrequencyHz)
{
    /* Determine length from frame type */
    uint8_t *p = (uint8_t *)buffer;
    int frameType = p[0];
    switch (frameType) {
        case 0x01:
            length = 13;
            break;
        case 0x02:
            length = 17;
            break;
        case 0x03:
            length = p[1] + 4;
            break;
        case 0x04:
            length = 19;
            break;
        case 0x05:
            length = 29;
            break;
        default:
            length = 0;
            break;
    }

    if (length >= 3) {
        if (_IMET_doParityCheck(p, length)) {
printf("* %02X\r\n",frameType);
            /* Get/create an instance */
            handle->instance = _IMET_getInstanceDataStructure(rxFrequencyHz);
            if (handle->instance) {
                if (frameType == IMET_FRAME_GPS) {
                    _IMET_processGpsFrame((IMET_FrameGps *)p, &handle->instance->gps);
                }
                if (frameType == IMET_FRAME_GPSX) {
                    _IMET_processGpsxFrame((IMET_FrameGpsx *)p, &handle->instance->gps);
                }

                /* Remember RX frequency (difference to nominal sonde frequency will be reported as frequency offset) */
                handle->rxFrequencyHz = rxFrequencyHz;

                /* If there is a position update, send it out */
                if (handle->instance->gps.updated) {
                    handle->instance->gps.updated = false;

                    _IMET_sendKiss(handle->instance);

                    LPCLIB_Event event;
                    LPCLIB_initEvent(&event, LPCLIB_EVENTID_APPLICATION);
                    event.opcode = APP_EVENT_HEARD_SONDE;
                    event.block = SONDE_DECODER_IMET;
                    event.parameter = (void *)((uint32_t)lrintf(rxFrequencyHz));
                    SYS_handleEvent(event);
                }
            }
        }
    }

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result IMET_resendLastPositions (IMET_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    IMET_InstanceData *instance = NULL;
    while (_IMET_iterateInstance(&instance)) {
        _IMET_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}

