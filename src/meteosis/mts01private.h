
#ifndef __MTS01PRIVATE_H
#define __MTS01PRIVATE_H


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "mts01.h"
#include "gps.h"


typedef __PACKED(struct {
    char dat8[128];
    uint16_t crc;
}) MTS01_RawData;


typedef union {
    MTS01_RawData rawData;
} MTS01_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
    float humidity;                             /* Relative humidity [%] */
    float innerTemperature;                     /* Temperature inside sonde [°C] */
} MTS01_CookedMetrology;


typedef struct {
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
    uint8_t usedSats;
    float pdop;
} MTS01_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _MTS01_InstanceData {
    struct _MTS01_InstanceData *next;
    uint32_t id;
    char name[20];
    uint16_t frameCounter;
    float rxFrequencyMHz;
    uint32_t lastUpdated;
    float rssi;
    uint64_t realTime;

    uint32_t lastGpsTime;
    MTS01_CookedGps gps;
    MTS01_CookedMetrology metro;
} MTS01_InstanceData;



LPCLIB_Result _MTS01_prepare (
        MTS01_Packet *packet,
        MTS01_InstanceData **instancePointer,
        float rxFrequencyHz);

/* Iterate through instances */
bool _MTS01_iterateInstance (MTS01_InstanceData **instance);

/* Remove an instance from the chain */
void _MTS01_deleteInstance (MTS01_InstanceData *instance);

/* Check frame CRC */
_Bool _MTS01_checkCRC (uint8_t *buffer, int length, uint16_t receivedCRC);

/* Process the payload */
LPCLIB_Result _MTS01_processPayload (MTS01_Packet* payload, MTS01_InstanceData* instance);

#endif
