
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


/** Context */
typedef struct LMS6_Context {
    uint8_t *pRawData;

    bool thisFrameNumberValid;

    float rxFrequencyHz;
    int nCorrectedErrors;

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
}


LPCLIB_Result LMS6_processBlock (
        LMS6_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime)
{
    /* Return if raw data is not the size of a RS(255,223) frame plus tail byte (in symbols!) */
    if (numBits != 2*(255+1)*8) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    //TODO convolutional decoder

    handle->pRawData = buffer;

    /* Error correction */
    handle->nCorrectedErrors = 0;
    if (_LMS6_checkReedSolomon (handle->pRawData, &handle->nCorrectedErrors) != LPCLIB_SUCCESS) {
        return LPCLIB_ERROR;
    }

    //TODO  add to data ring buffer; detect data frame, return if no complete data frame yet

    /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
    handle->rxFrequencyHz = rxFrequencyHz;

    /* Check data frame CRC */
#if 0
    if (_LMS6_checkCRC(..., ..., ...)) {

    }
#endif

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

