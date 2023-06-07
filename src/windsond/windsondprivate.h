
#ifndef __WINDSONDPRIVATE_H
#define __WINDSONDPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "windsond.h"
#include "gps.h"



typedef uint8_t WINDSOND_RawData[64];


typedef union {
    WINDSOND_RawData rawData;
} WINDSOND_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
    float cpuTemperature;                       /* CPU temperature [°C] */
} WINDSOND_CookedMetrology;


typedef struct {
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
} WINDSOND_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _WINDSOND_InstanceData {
    struct _WINDSOND_InstanceData *next;
    uint32_t id;
    float rxFrequencyMHz;
    float rxOffset;
    uint32_t lastUpdated;
    uint16_t frameCounter;

    uint64_t configValidFlags;                  /* Indicates valid fields in "config" */
    float config[64];

    WINDSOND_CookedGps gps;
    WINDSOND_CookedMetrology metro;
} WINDSOND_InstanceData;



LPCLIB_Result _MEISEI_processFrame (
        WINDSOND_Packet *packet,
        WINDSOND_InstanceData **instancePointer,
        float rxFrequencyHz);


/* Iterate through instances */
bool _WINDSOND_iterateInstance (WINDSOND_InstanceData **instance);

/* Remove an instance from the chain */
void _WINDSOND_deleteInstance (WINDSOND_InstanceData *instance);

#endif
