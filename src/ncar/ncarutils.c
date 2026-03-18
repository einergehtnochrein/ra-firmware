
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "ncar.h"
#include "ncarprivate.h"



/* Check CRC of a sub-block.
 * RD41 and RD94 append 16-bit CRCs to the end of data block. The CRC is stored in big-endian
 * format, which allows to check for valid CRC in a special way: Once the data block is shifted
 * through the CRC generator, the CRC shift register contains the CRC and can be compared to the
 * 16-bit value following the block. However, an alternative way is to also shift the received
 * big-endian CRC through the CRC generator, which results in the shift register being all zero when
 * the CRC is correct.
 */
_Bool _RD41_checkCRC (void *buffer, int length)
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
        CRC_seed(crc, 0);
        CRC_write(crc, buffer, length, NULL, NULL);

        result = 0 == CRC_read(crc);

        CRC_close(&crc);
    }

    return result;
}



_Bool _RD94_checkChecksum (void *buffer, int length, uint8_t checkSUM1, uint8_t checkSUM2)
{
    uint8_t sum1 = 0;
    uint8_t sum2 = 0;
    for (int i = 0; i < length; i++) {
        sum1 += ((uint8_t *)buffer)[i];
        sum2 += sum1;
    }

    return (sum1 == checkSUM1) && (sum2 == checkSUM2);
}
