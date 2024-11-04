
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "lms6.h"
#include "lms6private.h"
#include "reedsolomon.h"



/* Check CRC of a data frame */
_Bool _LMS6_checkCRC (void *buffer, int length, uint16_t receivedCRC)
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


/* Reed-Solomon error correction, RS(255,223) */
LPCLIB_Result _LMS6_checkReedSolomon (uint8_t rawFrame[], int *pNumErrors)
{
    int numErrors = 0;
    LPCLIB_Result result;


    uint8_t * _LMS6_getDataAddress (int index)
    {
        return &rawFrame[index];
    }

    /* Make sure the correct Galois field is used */
    REEDSOLOMON_makeGaloisField(0x187);

    /* Reed-Solomon error correction */
    result = REEDSOLOMON_process(32, 112, 11, 116, _LMS6_getDataAddress, &numErrors);

    if (pNumErrors) {
        *pNumErrors = numErrors;
    }

    return result;
}

