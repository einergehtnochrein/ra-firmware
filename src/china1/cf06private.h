
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
    uint8_t dat8[102];
    uint32_t dat32[102/4];
}) CF06_RawData;


typedef __PACKED(struct {
    uint32_t serial;
    uint32_t tow;
    uint32_t longitude;
    uint32_t latitude;
    uint32_t altitude;
    int16_t speed_east;
    int16_t speed_north;
    int16_t speed_down;
    uint8_t usedSats;
    uint8_t pdop;
    int16_t temperature;
    int16_t humidity;
    int16_t reserved20;
    int8_t temperature_CPU;
    int16_t temperature_Usensor;
    uint8_t vbat;
    uint8_t upCounter;
    uint8_t flags;
    uint16_t crc;
}) CF06_PayloadBlock1;


typedef __PACKED(struct {
    uint8_t reserved00[10];
    uint16_t reserved0A;
    uint16_t reserved0C;
    uint16_t reserved0E;
    uint16_t reserved10;
    uint16_t reserved12;
    uint16_t reserved14;
    uint16_t reserved16;
    uint16_t reserved18;
    uint16_t reserved1A;
    uint16_t reserved1C;
    uint16_t reserved1E;
    uint16_t reserved20;
    uint16_t reserved22;
    uint16_t reserved24;
    uint8_t flags;
    uint16_t crc;
}) CF06_PayloadBlock2;


typedef union {
    CF06_RawData rawData;
    __PACKED(struct {
        uint8_t reserved00[5];
        CF06_PayloadBlock1 block1;
        uint8_t fec_inner[6];
        CF06_PayloadBlock2 block2;
        uint8_t fec_outer[6];
    });
} CF06_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
    float humidity;                             /* Relative humidity [%] */
    float temperature_Usensor;                  /* Temperature of humidity sensor [°C] */
    float temperature_CPU;                      /* CPU temperature (main board) [°C] */
    float batteryVoltage;                       /* Battery voltage [V] */
} CF06_CookedMetrology;


typedef struct {
    LLA_Coordinate observerLLA;
    float pdop;
    uint8_t usedSats;
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
