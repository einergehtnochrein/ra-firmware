
#ifndef __MRZPRIVATE_H
#define __MRZPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "mrz.h"
#include "gps.h"


typedef __PACKED(union {
    uint8_t dat8[47];
    uint32_t dat32[11];
}) MRZ_RawData;


typedef union {
    MRZ_RawData rawData;
    __PACKED(struct {
        uint8_t unk0;
        __PACKED(struct {
            uint8_t hour;
            uint8_t minute;
            uint8_t second;
        });
        uint8_t unk4;
        __PACKED(struct {
            uint32_t ecef_x;
            uint32_t ecef_y;
            uint32_t ecef_z;
        }) gps;
        uint8_t unk11[28];
        uint16_t crc;
    });
} MRZ_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
} MRZ_CookedMetrology;


typedef struct {
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
} MRZ_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _JINYANG_InstanceData {
    struct _JINYANG_InstanceData *next;
    uint32_t id;
    uint16_t frameCounter;
    float rxFrequencyMHz;
    uint32_t lastUpdated;

    uint32_t lastGpsTime;
    MRZ_CookedGps gps;
    MRZ_CookedMetrology metro;
} MRZ_InstanceData;



LPCLIB_Result _MRZ_processConfigFrame (
        MRZ_Packet *packet,
        MRZ_InstanceData **instancePointer,
        float rxFrequencyHz);

/* Iterate through instances */
bool _MRZ_iterateInstance (MRZ_InstanceData **instance);

/* Remove an instance from the chain */
void _MRZ_deleteInstance (MRZ_InstanceData *instance);

/* Process the GPS frame */
LPCLIB_Result _MRZ_processGpsFrame (
        MRZ_Packet *rawGps,
        MRZ_InstanceData *instance);

#endif
