
#ifndef __BEACONPRIVATE_H
#define __BEACONPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "beacon.h"
#include "gps.h"


typedef uint8_t BEACON_RawData[7];

typedef __PACKED(union {
    BEACON_RawData rawData;
}) BEACON_Packet;


/* Data that needs to be stored for every instance. */
typedef struct _BEACON_InstanceData {
    struct _BEACON_InstanceData *next;
    uint32_t id;
    char name[20];
    float rxFrequencyMHz;
    float rxOffset;

    uint32_t lastUpdated;
} BEACON_InstanceData;



/* Iterate through instances */
bool _BEACON_iterateInstance (BEACON_InstanceData **instance);

/* Remove an instance from the chain */
void _BEACON_deleteInstance (BEACON_InstanceData *instance);

void BEACON_DSP_processAudio (const int32_t *rawAudio, float *cookedAudio, int nSamples);
void BEACON_DSP_initAudio (void);

#endif
