
#ifndef __CF06PRIVATE_H
#define __CF06PRIVATE_H


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "cf06.h"
#include "gps.h"


typedef __PACKED(union {
    uint8_t dat8[100];
    uint32_t dat32[25];
}) CF06_RawData;


typedef __PACKED(struct {
    uint32_t serial;
    uint32_t tow;
    uint32_t longitude;
    uint32_t latitude;
    uint32_t altitude;
    int16_t speed_east;
    int16_t speed_north;
    int16_t reserved18;
    uint8_t reserved1A;
    uint8_t reserved1B;
    uint16_t reserved1C;
    uint16_t reserved1E;
    uint16_t reserved20;
    uint8_t reserved0[6];
    uint16_t crc;
}) CF06_PayloadBlock1;


typedef __PACKED(struct {
    uint16_t reserved00;
    uint16_t reserved02;
    uint16_t reserved04;
    uint16_t reserved06;
    uint16_t reserved08;
    uint16_t reserved0A;
    uint16_t reserved0C;
    uint16_t reserved0E;
    uint16_t reserved10;
    uint16_t reserved12;
    uint16_t reserved14;
    uint16_t reserved16;
    uint16_t reserved18;
    uint16_t reserved1A;
    uint8_t reserved1C;
    uint16_t crc;
}) CF06_PayloadBlock2;


typedef union {
    CF06_RawData rawData;
    __PACKED(struct {
        uint8_t reserved00[3];
        CF06_PayloadBlock1 block1;
        uint8_t fec_inner[6];
        uint8_t reserved33[10];
        CF06_PayloadBlock2 block2;
        uint8_t fec_outer[6];
    });
} CF06_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
} CF06_CookedMetrology;


typedef struct {
    LLA_Coordinate observerLLA;
} CF06_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _CF06_InstanceData {
    struct _CF06_InstanceData *next;

    uint32_t id;
    char name[20];                              /* Sonde name */
    uint16_t frameCounter;
    float rxFrequencyMHz;
    uint32_t lastUpdated;

    uint32_t lastGpsTime;
    CF06_CookedGps gps;
    CF06_CookedMetrology metro;
} CF06_InstanceData;



LPCLIB_Result _CF06_prepare (
        CF06_PayloadBlock1 *block1,
        CF06_InstanceData **instancePointer,
        float rxFrequencyHz);

/* Iterate through instances */
bool _CF06_iterateInstance (CF06_InstanceData **instance);

/* Remove an instance from the chain */
void _CF06_deleteInstance (CF06_InstanceData *instance);

/* Check CRC of outer/inner block */
_Bool _CF06_checkCRCOuter (uint8_t *buffer, int length, uint16_t receivedCRC);
_Bool _CF06_checkCRCInner (uint8_t *buffer, int length, uint16_t receivedCRC);
/* Reed-Solomon error correction */
LPCLIB_Result _CF06_checkReedSolomonOuter (uint8_t rawFrame[], int *pNumErrors);
LPCLIB_Result _CF06_checkReedSolomonInner (uint8_t rawFrame[], int *pNumErrors);

LPCLIB_Result _CF06_processPayloadBlock1 (
        const CF06_PayloadBlock1 *payload,
        CF06_CookedGps *cookedGps,
        CF06_CookedMetrology *cookedMetro);
LPCLIB_Result _CF06_processPayloadBlock2 (
        const CF06_PayloadBlock2 *payload,
        CF06_CookedGps *cookedGps,
        CF06_CookedMetrology *cookedMetro);

#endif
