
#ifndef __HT03PRIVATE_H
#define __HT03PRIVATE_H


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "ht03.h"
#include "gps.h"


typedef __PACKED(union {
    uint8_t dat8[100];
    uint32_t dat32[25];
}) HT03_RawData;


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
}) HT03_Payload;


typedef union {
    HT03_RawData rawData;
    __PACKED(struct {
        HT03_Payload payload;
        uint16_t crc_magic;
    });
} HT03_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
} HT03_CookedMetrology;


typedef struct {
    LLA_Coordinate observerLLA;
} HT03_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _HT03_InstanceData {
    struct _HT03_InstanceData *next;

    uint32_t id;
    char name[20];                              /* Sonde name */
    uint16_t frameCounter;
    float rxFrequencyMHz;
    uint32_t lastUpdated;

    uint32_t lastGpsTime;
    HT03_CookedGps gps;
    HT03_CookedMetrology metro;
} HT03_InstanceData;



LPCLIB_Result _HT03_prepare (
        HT03_Payload *payload,
        HT03_InstanceData **instancePointer,
        float rxFrequencyHz);

/* Iterate through instances */
bool _HT03_iterateInstance (HT03_InstanceData **instance);

/* Remove an instance from the chain */
void _HT03_deleteInstance (HT03_InstanceData *instance);

/* Check CRC */
_Bool _HT03_checkCRC (uint8_t *buffer, int length, uint16_t receivedCRC);

LPCLIB_Result _HT03_processPayload (
        const HT03_Payload *payload,
        HT03_CookedGps *cookedGps,
        HT03_CookedMetrology *cookedMetro);

#endif
