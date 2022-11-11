
#ifndef __GTH3PRIVATE_H
#define __GTH3PRIVATE_H


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "gth3.h"
#include "gps.h"


typedef __PACKED(union {
    uint8_t dat8[100];
    uint32_t dat32[25];
}) GTH3_RawData;


typedef __PACKED(struct {
    uint8_t reserved00[5];
    uint8_t serial[4];
    uint8_t reserved05;
    uint8_t hour;
    uint8_t sec_minute;
    uint8_t tenth_seconds;
    uint8_t reserved0D;
    float longitude;
    float latitude;
    uint32_t altitude;
    int16_t speedN;
    int16_t speedE;
    int16_t speedV;
    uint8_t reserved20[66];
    uint16_t crc;
}) GTH3_Payload;


typedef union {
    GTH3_RawData rawData;
    __PACKED(struct {
        GTH3_Payload payload;
        uint16_t crc_magic;
    });
} GTH3_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
} GTH3_CookedMetrology;


typedef struct {
    LLA_Coordinate observerLLA;
} GTH3_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _GTH3_InstanceData {
    struct _GTH3_InstanceData *next;

    uint32_t id;
    char name[20];                              /* Sonde name */
    uint16_t frameCounter;
    float rxFrequencyMHz;
    uint32_t lastUpdated;
    float rssi;
    uint64_t realTime;

    uint32_t lastGpsTime;
    GTH3_CookedGps gps;
    GTH3_CookedMetrology metro;
} GTH3_InstanceData;



LPCLIB_Result _GTH3_prepare (
        GTH3_Payload *payload,
        GTH3_InstanceData **instancePointer,
        float rxFrequencyHz);

/* Iterate through instances */
bool _GTH3_iterateInstance (GTH3_InstanceData **instance);

/* Remove an instance from the chain */
void _GTH3_deleteInstance (GTH3_InstanceData *instance);

/* Check CRC */
_Bool _GTH3_checkCRC (uint8_t *buffer, int length, uint16_t receivedCRC);

LPCLIB_Result _GTH3_processPayload (
        const GTH3_Payload *payload,
        GTH3_CookedGps *cookedGps,
        GTH3_CookedMetrology *cookedMetro);

#endif
