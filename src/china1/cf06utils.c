
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "cf06.h"
#include "cf06private.h"
#include "reedsolomon.h"



/* Check CRC of a outer block */
_Bool _CF06_checkCRCOuter (uint8_t *buffer, int length, uint16_t receivedCRC)
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
        CRC_seed(crc, 0x0000);
        CRC_write(crc, buffer, length, NULL, NULL);

        result = receivedCRC == CRC_read(crc);

        CRC_close(&crc);
    }

    return result;
}


/* Check CRC of inner block */
_Bool _CF06_checkCRCInner (uint8_t *buffer, int length, uint16_t receivedCRC)
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
        CRC_seed(crc, 0x0000);
        CRC_write(crc, buffer, length, NULL, NULL);

        result = receivedCRC == CRC_read(crc);

        CRC_close(&crc);
    }

    return result;
}


/* Reed-Solomon error correction */
LPCLIB_Result _CF06_checkReedSolomonOuter (uint8_t rawFrame[], int *pNumErrors)
{
    int numErrorsOuter = 0;
    LPCLIB_Result result;
    uint8_t _CF06_null;


    uint8_t * _CF06_getDataAddressOuter (int index)
    {
        if (index < 89) {
            return &rawFrame[93 - index];
        }
        else if (index >= 249) {
            return &rawFrame[348 - index];
        }
        else {
            _CF06_null = 0;
            return &_CF06_null;
        }
    }

    /* Make sure the correct Galois field is used */
    REEDSOLOMON_makeGaloisField(0x11D);

    /* Reed-Solomon error correction for outer frame */
    result = REEDSOLOMON_process(6, 1, 1, 1, _CF06_getDataAddressOuter, &numErrorsOuter);

    if (pNumErrors) {
        *pNumErrors = numErrorsOuter;
    }

    return result;
}


/* Reed-Solomon error correction */
LPCLIB_Result _CF06_checkReedSolomonInner (uint8_t rawFrame[], int *pNumErrors)
{
    int numErrorsInner = 0;
    LPCLIB_Result result;
    uint8_t _CF06_null;


    uint8_t * _CF06_getDataAddressInner (int index)
    {
        if (index < 42) {
            return &rawFrame[46 - index];
        }
        else if (index >= 249) {
            return &rawFrame[301 - index];
        }
        else {
            _CF06_null = 0;
            return &_CF06_null;
        }
    }


    /* Reed-Solomon error correction for inner frame */
    result = REEDSOLOMON_process(6, 1, 1, 1, _CF06_getDataAddressInner, &numErrorsInner);

    if (pNumErrors) {
        *pNumErrors = numErrorsInner;
    }

    return result;
}

