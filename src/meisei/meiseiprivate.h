
#ifndef __MEISEIPRIVATE_H
#define __MEISEIPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "meisei.h"
#include "gps.h"


typedef uint8_t MEISEI_RawData[48];


typedef __PACKED(union {
    MEISEI_RawData rawData;
    uint64_t fields[6];
}) MEISEI_Packet;



typedef struct {
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
} MEISEI_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _MEISEI_InstanceData {
    struct _MEISEI_InstanceData *next;
    uint32_t id;
    float rxFrequencyMHz;
    float rxOffset;

    uint32_t lastUpdated;

    uint16_t frameCounter;
    uint16_t _even1;
    uint16_t _even2;
    uint16_t _even3;
    uint16_t _even4;
    uint16_t _even5;
    uint16_t _even6;
    uint16_t _even7;
    uint16_t _even8;
    uint16_t _even9;
    uint16_t _even10;
    uint16_t _even11;
    uint16_t _odd1;
    uint16_t _odd2;
    uint16_t _odd3;
    uint16_t _odd4;
    uint16_t _odd5;
    uint16_t _odd6;
    uint16_t _odd7;
    uint16_t _odd8;
    uint16_t _odd9;
    uint16_t _odd10;
    uint16_t _odd11;

    MEISEI_CookedGps gps;
    uint16_t _gps10;
    uint16_t _gps11;
} MEISEI_InstanceData;



LPCLIB_Result _MEISEI_processConfigFrame (
        MEISEI_Packet *packet,
        MEISEI_InstanceData **instancePointer,
        float rxFrequencyHz);
LPCLIB_Result _MEISEI_processGpsFrame (
        const MEISEI_Packet *packet,
        MEISEI_InstanceData *instance);
uint16_t _MEISEI_getPayloadHalfWord (const uint64_t *fields, int index);

/* Iterate through instances */
bool _MEISEI_iterateInstance (MEISEI_InstanceData **instance);

/* Remove an instance from the chain */
void _MEISEI_deleteInstance (MEISEI_InstanceData *instance);

#endif
