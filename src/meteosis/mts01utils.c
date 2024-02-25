
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "mts01.h"
#include "mts01private.h"


/* Check frame CRC */
_Bool _MTS01_checkCRC (uint8_t *buffer, int length, uint16_t receivedCRC)
{
    /* Out of the myriad of CRC variants, MTS-01 has chosen one that is not implemented
     * by the CRC peripheral of the LPC microcontroller.
     * Need to implement it manually :-(
     */
    uint16_t crc = 0xFFFF;
    while (length--) {
        /* Do the XOR of input data (MSB first) with the LFSR output
         * in one step for all eight bits.
         */
        crc ^= *buffer++ << 8;

        // TODO use a table instead of the loop below
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : crc << 1;
        }
    }

    /* Reverse bit order */
    crc = __REV16(crc);

    return receivedCRC == crc;
}
