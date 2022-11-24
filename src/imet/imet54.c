
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "imet54.h"
#include "imet54private.h"
#include "bch.h"

/*
 * iMet-54 frame parameters (from firmware analysis)
 *
 * 2FSK, 4800 bit/s, 2.5 kHz deviation
 * Continuous transmission (no preamble)
 * All bytes are wrapped as 8N1 UART characters, which guarantees two signal edges
 * every 10 bit times.
 * Sync character (after UART deframing): (At least) 10x 0x00 0x55 (20 characters), followed by 4x 0x24.
 * As bits:  ..... 0 00000000 1  0 10101010 1  0 00100100 1  0 00100100 1  0 00100100 1  0 00100100 1
 *
 * Packet structure (all bytes received as 8N1 UART characters: start(0) + 8 bits (LSB first) + stop(1):
 *
 * +--------+----------+---------+
 * | Header | Main     | CRC32   |
 * | 1 byte | 52 bytes | 4 bytes |
 * +--------+----------+---------+
 * |  0x03  | Encoded: 112 bytes |   If header bit 6 is cleared, block 2 is omitted.
 * +--------+--------------------+
 *  \----------- CRC -----------/
 *           \--- Interleaved --/
 *           \--- Hamming 8,4 --/
 *
 * +--------+----------+---------+-----------+---------+
 * | Header | Main     | CRC32   | Block 2   | CRC32   |
 * | 1 byte | 52 bytes | 4 bytes | 148 bytes | 4 bytes |
 * +--------+----------+---------+-----------+---------+
 * |  0xC3  | Encoded: 112 bytes | Encoded: 304 bytes  |
 * +--------+--------------------+---------------------+
 *  \----------- CRC -----------/ \------- CRC -------/
 *           \--- Interleaved --/ \--- Interleaved ---/
 *           \--- Hamming 8,4 --/ \--- Hamming 8,4 ---/
 *
 * +--------+----------+---------+-------------+---------+
 * | Header | Main     | CRC32   | Block 2     | CRC32   |
 * | 1 byte | 52 bytes | 4 bytes | 156 bytes   | 4 bytes |
 * +--------+----------+---------+-------------+---------+
 * |  0x43  | Encoded: 112 bytes | Encoded: 320 bytes    |
 * +--------+--------------------+-----------------------+
 *  \----------- CRC -----------/ \-------- CRC --------/
 *           \--- Interleaved --/ \---- Interleaved ----/
 *           \--- Hamming 8,4 --/ \---- Hamming 8,4 ----/
 *
 * +--------+----------+---------+-------------+---------+
 * | Header | Main     | CRC32   | Block 2     | CRC32   |
 * | 1 byte | 52 bytes | 4 bytes | 316 bytes   | 4 bytes |
 * +--------+----------+---------+-------------+---------+
 * |  0x43  | Encoded: 112 bytes | Encoded: 320 bytes    |
 * +--------+--------------------+-----------------------+
 *  \----------- CRC -----------/ \-------- CRC --------/
 *           \--- Interleaved --/ \---- Interleaved ----/
 *           \--- Hamming 8,4 --/
 *
 * A frame may come in one of four different formats. The header value and the 'flags' field in the
 * main block indicate which format is used. You must first decode and verify the main block to make
 * sure the header byte is valid.
 * Frames are sent every full second on the average. Additional 0x00/0x55 fill bytes are appended
 * to shorter frames.
 * The last frame format (without Hamming encoding of block2) is used if there is more than
 * 156 bytes of XDATA to be transmitted in a frame.
 *
 * Regular CRC32 (valid for both blocks):
 * CRC parameters: polynomial=0x04C11DB7, seed=0xFFFFFFFF, LSB first, output-XOR=0
 * CRC is sent in little-endian format
 *
 * Hamming 8,4 code: 4-bit nibbles are encoded to systematic 8-bit Hamming codewords. However, the
 * plain text bits do not appear as a block, but are distributed as follows over the codeword bits:
 *
 * Nibble bit --> Codeword bit
 *      0     -->      2
 *      1     -->      4
 *      2     -->      5
 *      3     -->      6
 *                                    / 0 1 0 0 1 0 1 1 \
 * The generator matrix is:       G = | 1 0 1 0 1 0 1 0 |
 *                                    | 1 0 0 1 1 0 0 1 |
 *                                    \ 1 0 0 0 0 1 1 1 /
 *
 * Interleaving: Blocks of eight bytes a...h (a7...a0 b7...b0  ...  g7...g0 h7...h0) are rearranged
 * as follows:
 *     a0b0c0d0e0f0g0h0 a1b1c1d1e1f1g1h1 a2b2c2d2e2f2g2h2 a3b3c3d3e3f3g3h3
 *     a4b4c4d4e4f4g4h4 a5b5c5d5e5f5g5h5 a6b6c6d6e6f6g6h6 a7b7c7d7e7f7g7h7
 *
 * Decoding process: Received data is first deinterleaved, then Hamming decoded (if applicable).
 */


/** Context */
typedef struct _IMET54_Context {
    IMET54_Packet packet;
    IMET54_InstanceData *instance;
    float rxFrequencyHz;

    IMET54_SubFrameMain frameMain;
    union {
        IMET54_SubFrame2Long frame2Long;
        IMET54_SubFrame2Short frame2Short;

        IMET54_Block2_Type1_6 block2_type1_6;
    };
} IMET54_Context;

static IMET54_Context _imet54;



//TODO
LPCLIB_Result IMET54_open (IMET54_Handle *pHandle)
{
    IMET54_Handle handle = &_imet54;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _IMET54_sendKiss (IMET54_InstanceData *instance)
{
    static char s[160];
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

    length = snprintf((char *)s, sizeof(s), "%"PRIu32",18,%.3f,%d,%.5lf,%.5lf,%.0f,,,,%.1f,%.1f,,,%.1f,,%.1f,,%d,%d,,%.2f,%.1f,,%.1lf",
                    instance->id,
                    instance->rxFrequencyMHz,               /* Nominal sonde frequency [MHz] */
                    instance->gps.usedSats,
                    latitude,                               /* Latitude [degrees] */
                    longitude,                              /* Longitude [degrees] */
                    instance->gps.observerLLA.alt,          /* Altitude [m] */
                    instance->metro.temperature,            /* Temperature [°C] */
                    instance->metro.pressure,               /* Pressure [hPa] */
                    instance->metro.humidity,               /* Relative humidity [%] */
                    instance->rssi,
                    instance->gps.visibleSats,
                    instance->frameCounter,
                    instance->batteryVoltage,
                    instance->metro.temperatureCpu,         /* CPU temperature [°C] */
                    instance->realTime / 10.0
                    );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_KISS, s);
    }

    length = snprintf(s, sizeof(s), "%"PRIu32",18,0,%s,%.1f,%.1f,%.1f",
                instance->id,
                instance->name,
                instance->metro.temperatureRH,
                instance->metro.temperaturePSensor,
                instance->metro.temperatureInner
                );

    if (length > 0) {
        SYS_send2Host(HOST_CHANNEL_INFO, s);
    }
}

#if 0
static void _IMET54_sendRaw (IMET54_Handle handle, IMET54_Packet *p1)
{
(void)handle;
(void)p1;
}
#endif


LPCLIB_Result IMET54_processBlock (
        IMET54_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime)
{
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;
    uint8_t header;
    int i;

    if (numBits == 8*(1+sizeof(IMET54_Packet))) {
        header = ((uint8_t *)buffer)[0];
        memcpy(&handle->packet, (uint8_t *)buffer+1, sizeof(handle->packet));

        /* Main block is always there */
        for (i = 0; i < 14; i++) {
            _IMET54_deinterleave(&handle->packet.rawData.dat8[0+8*i]);
        }

        /* Main block is always Hamming encocded */
        if (_IMET54_checkParity(handle->packet.frameMainRaw, sizeof(IMET54_SubFrameMainRaw), (uint8_t *)&handle->frameMain)) {
            /* Check CRC of main block */
            if (_IMET54_checkCRC(
                    &header, 1,
                    (uint8_t *)&handle->frameMain, sizeof(handle->frameMain) - 4,
                    handle->frameMain.crc)) {
                _IMET54_prepare(&handle->frameMain, &handle->instance, rxFrequencyHz);
                if (handle->instance) {
                    handle->instance->rssi = rssi;
                    handle->instance->realTime = realTime;

                    if (_IMET54_processPayloadMain(&handle->frameMain, handle->instance) == LPCLIB_SUCCESS) {
                        /* Process block 2 based on header information */
                        if (header & (1u << 6)) {
                            /* Deinterleave block 2 */
                            for (i = 0; i < 40; i++) {
                                _IMET54_deinterleave(&handle->packet.rawData.dat8[112+8*i]);
                            }

                            _Bool block2valid = false;
                            if (handle->frameMain.flags & (1u << 13)) {
                                /* Short block 2 requires Hamming decoding, then check CRC */
                                if (_IMET54_checkParity(handle->packet.frame2Raw, sizeof(IMET54_SubFrame2Raw), (uint8_t *)&handle->frame2Short)) {
                                    block2valid = _IMET54_checkCRC(NULL, 0,
                                            (uint8_t *)&handle->frame2Short, sizeof(handle->frame2Short) - 4,
                                            handle->frame2Short.crc);
                                }
                            } else {
                                /* Skip Hamming decode for long block 2, just check CRC */
                                memcpy(&handle->frame2Long, handle->packet.frame2Raw, sizeof(handle->frame2Long));
                                block2valid = _IMET54_checkCRC(NULL, 0,
                                        (uint8_t *)&handle->frame2Long, sizeof(handle->frame2Long) - 4,
                                        handle->frame2Long.crc);
                            }

                            if (block2valid) {
                                ;
                                ; //TODO
                                ;
                            }
                        }

                        /* Send position report */
                        _IMET54_sendKiss(handle->instance);
                    }
                }
            }
        }
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result IMET54_resendLastPositions (IMET54_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    IMET54_InstanceData *instance = NULL;
    while (_IMET54_iterateInstance(&instance)) {
        _IMET54_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result IMET54_removeFromList (IMET54_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    IMET54_InstanceData *instance = NULL;
    while (_IMET54_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _IMET54_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

