
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "gth3.h"
#include "gth3private.h"



/* Check CRC */
_Bool _GTH3_checkCRC (uint8_t *buffer, int length, uint16_t receivedCRC)
{
    CRC_Handle crc = LPCLIB_INVALID_HANDLE;
    CRC_Mode crcMode;
    bool result = false;

    crcMode = CRC_makeMode(
            CRC_POLY_CRCCCITT,
            CRC_DATAORDER_REVERSE,
            CRC_SUMORDER_REVERSE,
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

