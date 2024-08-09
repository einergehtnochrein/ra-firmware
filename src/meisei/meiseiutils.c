
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bch.h"
#include "meisei.h"
#include "meiseiprivate.h"



/* BCH error correction */
LPCLIB_Result _MEISEI_checkBCH (MEISEI_RawPacket *raw, int *pNumErrors)
{
    int totalErrors = 0;
    int numErrors = 0;
    LPCLIB_Result result;
    uint64_t *pBCH;
    int i;


    int _MEISEI_getDataBCH (int index)
    {
        if (index < 12+34) {
            return (*pBCH >> (46 - 1 - index)) & 1;
        }
        else {
            return 0;
        }
    }

    void _MEISEI_toggleDataBCH (int index)
    {
        if (index < 12+34) {
            *pBCH ^= 1ull << (46 - 1 - index);
        }
    }


    result = LPCLIB_SUCCESS;
    for (i = 0; i < 6; i++) {
        pBCH = &raw->fields[i];
        result = BCH_63_51_t2_process(_MEISEI_getDataBCH, _MEISEI_toggleDataBCH, &numErrors);
        totalErrors += numErrors;
        if (result != LPCLIB_SUCCESS) {
            break;
        }
    }

    if (pNumErrors) {
        *pNumErrors = totalErrors;
    }

    return result;
}


/* Index: 0...11 */
static uint16_t _MEISEI_getPayloadHalfWord (const uint64_t *fields, int index)
{
    uint32_t x = 0;
    if (index % 2) {
        x = (fields[index / 2] & 0x00000001FFFE0000LL) >> 1;
    }
    else {
        x = (fields[index / 2] & 0x000000000000FFFFLL) << 16;
    };

    return __RBIT(x);
}


void _MEISEI_extractDataFromCodewords (const MEISEI_RawPacket *rawPacket, MEISEI_Packet *packet)
{
    for (int i = 0; i < 12; i++) {
        packet->w[i] = _MEISEI_getPayloadHalfWord(rawPacket->fields, i);
    }
}
