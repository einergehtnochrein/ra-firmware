
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "gps.h"
#include "lms6.h"
#include "lms6private.h"
#include "sys.h"
#include "xdata.h"

/*
 * LMS-6 frame parameters
 *
 * 2FSK, 4800 symbols/s, 2.4 kHz deviation, continuous framed transmission.
 * Follows the CCSDS standard (CCSDS 131.0-B-5, concatenated coding):
 *   Rate 1/2, constraint length k=7, convolutional inner code (-> bit rate 2400 bit/s).
 *   Reed Solomon RS(255,223) outer code.
 * Continuous transmission of RS(255,223) codewords, followed by a tail byte, and separated by
 * a 32-bit sync pattern (ASM = Attached Sync Marker):
 *
 * -----+ +-------------+ +----------------------+ +---------+ +-----
 *      | |     ASM     | | RS(255,223) codeword | |  Tail   | |
 *  ... | | 1A CF FC 1D | |    223+32 bytes      | |   00    | | ...
 *      | |   64 syms   | |     4080 syms        | | 16 syms | |
 * -----+ +-------------+ +----------------------+ +---------+ +-----
 *                        \---- Reed Solomon ----/
 * -----/ \-------------- Convolutional coding --------------/ \-----
 *
 * Total frame duration: 4160 symbols = 866.667 ms. The 223 bytes of data in the RS codeword form
 * a continuous data stream, completely independent of the framing applied by the concatenated
 * CCSDS coding. The data rate is 223 bytes / 866.667 ms > 257 bytes/s.
 *
 * The tail byte (eight bits of zero) forces the convolutional encoder into a guaranteed all-zero
 * state before the transmission of the next ASM begins. This means that the ASM is always
 * transmitted as a well known fixed symbol sequence, and the Ra receiver is configured to detect
 * the sync marker on symbol level *before* convolutional decoding.
 *
 * ASM bit sequence:  0001 1010 1100 1111 1111 1100 0001 1101
 * ASM symbol sequence:  0101 0110 0000 1000 0001 1100 1001 0111 0001 1010 1010 0111 0011 1101 0011 1110
 *
 * Reed-Solomon code:  RS(255,223), symbols from GF(256) with primitive polynomial 0x187
 *                     and generator element alpha^11. RS generator polynomial
 *                     g = (x - (alpha^11)^112)*...*(x - (alpha^11)^143)
 *                     TODO It looks like RS codeword bytes are sent LSB first into
 *                          the convolutional encoder
 *
 * (For a detailed description see https://public.ccsds.org/Publications/BlueBooks.aspx,
 *  CCSDS 131.0-B-x  TM Synchronization and Channel Coding)
 */

/*
 * User data is sent transparently as packets of 223 bytes. As the capacity of the radio channel
 * is about 257 bytes/s, a variable number of fill bytes is used between packets to achieve an
 * average transmission rate of one packet per second. The fill byte used is 0xCA.
 *
 * Packets are made of 4+217 bytes of payload, followed by a 16-bit CRC to ensure packet integrity.
 * The first four bytes of the payload contain a fixed pattern (24 54 00 00, ASCII: "$T") to detect
 * the beginning of a packet.
 *
 * -----------+ +-------------+-----------+---------+ +-----------
 *            | |     Sync    | User data |   CRC   | |
 *  ... CA CA | | 24 54 00 00 | 217 bytes | 2 bytes | | CA CA ...
 * -----------+ +-------------+-----------+---------+ +-----------
 *              \----- Used for CRC ------/
 *
 * CRC parameters: polynomial=0x1021, seed=0, LSB first, output-XOR=0, big-endian
 */


#define LMS6_RINGBUFFER_SIZE    512     /* Must be large enough to take twice the maximum of a
                                         * data frame (223 bytes) and a RS codeword payload
                                         * (223 bytes)
                                         */
#define LMS6_FRAME_TYPE_T       "$T"

/** Context */
typedef struct LMS6_Context {
    uint8_t *pRawData;

    float rxFrequencyHz;
    int nCorrectedErrors;

    uint8_t ringBuffer[2 * LMS6_RINGBUFFER_SIZE];   /* Ringbuffer with linear extension */
    int ringBufferWrIndex;
    int ringBufferRdIndex;

    LMS6_InstanceData *instance;
} LMS6_Context;


static LMS6_Context _lms6;


//TODO
LPCLIB_Result LMS6_open (LMS6_Handle *pHandle)
{
    *pHandle = &_lms6;

    return LPCLIB_SUCCESS;
}


/* Send position as a KISS packet */
static void _LMS6_sendKiss (LMS6_InstanceData *instance)
{
    char s[160];
    int length = 0;


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

    length = snprintf((char *)s, sizeof(s),
                "%"PRIu32",23,%.3f,,%.5lf,%.5lf,%.0f,%.1f,%.1f,%.1f,%.1f,%.1f,,,%.1f,,%.1f,,,%"PRIu32",,,,,%.2lf",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    instance->gps.observerLLA.climbRate,    /* Climb rate [m/s] */
                    direction,                              /* Direction [degrees} */
                    velocity,                               /* Velocity [km/h] */
                    instance->metro.temperature,                      /* Temperature main sensor [°C] */
                    instance->metro.pressure,               /* Pressure [hPa] */
                    instance->metro.humidity,               /* Relative humidity [%] */
                    instance->rssi,
                    instance->frameNumber,
                    instance->realTime * 0.01
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",23,0,%"PRIu32,
                instance->id,
                instance->serial
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}


static char _lms6_rawCompressed[320];

static void _LMS6_sendRaw (LMS6_InstanceData *instance, uint8_t *buffer, uint32_t length)
{
    uint32_t i = 0;
    int j;
    uint8_t uu[4+1];
    int N = ((length + 1) / 3) * 3;
    int n;
    int slen = 0;
    char *s = _lms6_rawCompressed;


    slen += snprintf(&s[slen], sizeof(_lms6_rawCompressed) - slen, "%"PRIu32",23,1,%"PRIu32",",
                     instance->id,
                     length
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

        slen += snprintf(&s[slen], sizeof(_lms6_rawCompressed) - slen, "%s", uu);
    }

    SYS_send2Host(HOST_CHANNEL_INFO, s);
}


LPCLIB_Result LMS6_processBlock (
        LMS6_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime)
{
    /* Return if raw data is not the size of an RS(255,223) block */
    if (numBits != 255 * 8) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    handle->pRawData = buffer;

    /* Error correction */
    handle->nCorrectedErrors = 0;
    if (_LMS6_checkReedSolomon (handle->pRawData, &handle->nCorrectedErrors) != LPCLIB_SUCCESS) {
        return LPCLIB_ERROR;
    }

    /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
    handle->rxFrequencyHz = rxFrequencyHz;

    /* Copy RS codeword message part to ring buffer */
    memcpy(&handle->ringBuffer[handle->ringBufferWrIndex], handle->pRawData, 223);
    int extSize = 223 - (LMS6_RINGBUFFER_SIZE - handle->ringBufferWrIndex);
    if (extSize > 0) {
        memcpy(handle->ringBuffer, &handle->ringBuffer[LMS6_RINGBUFFER_SIZE], extSize);
    }
    handle->ringBufferWrIndex = (handle->ringBufferWrIndex + 223) % LMS6_RINGBUFFER_SIZE;

    /* Look for frame signature */
    while (handle->ringBufferRdIndex != handle->ringBufferWrIndex) {
        if (handle->ringBuffer[handle->ringBufferRdIndex] == '$') {
            /* Possible signature start found. Check if there's currently enough data in
             * ringbuffer for a complete frame.
             */
            int bytesAvailable = handle->ringBufferWrIndex >= handle->ringBufferRdIndex ?
                        handle->ringBufferWrIndex - handle->ringBufferRdIndex :
                        LMS6_RINGBUFFER_SIZE - handle->ringBufferRdIndex + handle->ringBufferWrIndex;
            if (bytesAvailable >= (int)sizeof(LMS6_RawFrame)) {
                LMS6_RawFrame *p = (LMS6_RawFrame *)&handle->ringBuffer[handle->ringBufferRdIndex];
                if (!strncmp(p->signature, LMS6_FRAME_TYPE_T, 4 /* TODO sizeof... */)) {
                    _LMS6_sendRaw(handle->instance, (uint8_t *)p, sizeof(*p));

                    /* Check data frame CRC */
                    if (_LMS6_checkCRC(p, sizeof(LMS6_RawFrame) - sizeof(uint16_t), __REV16(p->crc))) {
                        _LMS6_processConfigBlock(p, &handle->instance);
                        if (handle->instance) {
                            handle->instance->rssi = rssi;
                            handle->instance->realTime = realTime;
                            handle->instance->rxFrequencyMHz = handle->rxFrequencyHz / 1e6f;
                            handle->instance->frameNumber = __REV16(p->frameNumber);

                            if (_LMS6_processPayload(p,
                                                      &handle->instance->gps,
                                                      &handle->instance->metro) == LPCLIB_SUCCESS) {
                                _LMS6_sendKiss(handle->instance);
                            }
                        }
                    }

                    handle->ringBufferRdIndex = (handle->ringBufferRdIndex + sizeof(LMS6_RawFrame)) % LMS6_RINGBUFFER_SIZE;
                }
            }
            break;
        }
        handle->ringBufferRdIndex = (handle->ringBufferRdIndex + 1) % LMS6_RINGBUFFER_SIZE;
    }

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result LMS6_resendLastPositions (LMS6_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LMS6_InstanceData *instance = NULL;
    while (_LMS6_iterateInstance(&instance)) {
        _LMS6_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result LMS6_removeFromList (LMS6_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    LMS6_InstanceData *instance = NULL;
    while (_LMS6_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;
            
            /* Remove sonde */
            _LMS6_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

