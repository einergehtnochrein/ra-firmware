
#ifndef __PSB3PRIVATE_H
#define __PSB3PRIVATE_H


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "psb3.h"
#include "gps.h"


typedef __PACKED(union {
    uint8_t dat8[44];
    uint32_t dat32[11];
}) PSB3_RawData;


typedef union {
    PSB3_RawData rawData;
    __PACKED(struct {
        uint16_t frameNumber;                   /* Frame number (BE) */
        uint16_t rawAnalogU4;                   /* Raw ADC input from extension module U4 (BE) */
        uint16_t rawTemperature;                /* Raw ADC value of ambient temperature sensor (BE) */
        uint16_t reserved06;
        uint16_t rawHumidity;                   /* Raw ADC value of humidity sensor HIH-4021 (BE) */
        uint8_t reserved0A[3];
        uint8_t reserved0D[3];
        uint8_t day;
        uint8_t reserved11;
        uint8_t year;
        uint8_t hour;                           /* UTC Hour (BCD) */
        uint8_t minute;                         /* UTC Minute (BCD) */
        uint8_t second;                         /* UTC Second (BCD) */
        uint8_t latitude[4];                    /* Latitude (BCD) */
        uint8_t longitude[5];                   /* Longitude (BCD) */
        uint8_t reserved1F;
        uint8_t altitude[3];                    /* Altitude (BCD) */
        uint8_t fec[9];
    });
} PSB3_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
    float humidity;                             /* Relative humidity [%] */
} PSB3_CookedMetrology;


typedef struct {
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
} PSB3_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _PSB3_InstanceData {
    struct _PSB3_InstanceData *next;
    uint32_t id;
    char name[20];
    uint16_t frameCounter;
    float rxFrequencyMHz;
    uint32_t lastUpdated;

    uint32_t lastGpsTime;
    PSB3_CookedGps gps;
    PSB3_CookedMetrology metro;
} PSB3_InstanceData;



LPCLIB_Result _PSB3_prepare (
        PSB3_Packet *packet,
        PSB3_InstanceData **instancePointer,
        float rxFrequencyHz);

/* Iterate through instances */
bool _PSB3_iterateInstance (PSB3_InstanceData **instance);

/* Remove an instance from the chain */
void _PSB3_deleteInstance (PSB3_InstanceData *instance);

/* Check parity bits */
_Bool _PSB3_checkParity (uint8_t *buffer, int length);

/* Process the payload */
LPCLIB_Result _PSB3_processPayload (
        const PSB3_Packet *payload,
        PSB3_InstanceData *instance);

#endif
