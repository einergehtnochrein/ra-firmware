
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "china1.h"
#include "cf06.h"
#include "gth3.h"


LPCLIB_Result CHINA1_processBlock (
        void *handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz)
{
    /* Determine sonde type */
    uint32_t payloadStart;
    payloadStart = (((uint8_t *)buffer)[2] << 0)
                 + (((uint8_t *)buffer)[3] << 8)
                 + (((uint8_t *)buffer)[4] << 16)
                 ;

    if (payloadStart == 0x00EEFFFF) {
        // GTH3
        return GTH3_processBlock(handle, buffer, numBits, rxFrequencyHz);
    }

    if (payloadStart == 0x00AAAAFF) {
        // CF-06AH
        return CF06_processBlock(handle, buffer, numBits, rxFrequencyHz);
    }

    return LPCLIB_ERROR;
}

