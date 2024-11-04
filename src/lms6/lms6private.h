
#ifndef __LMS6PRIVATE_H
#define __LMS6PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "lms6.h"
#include "gps.h"


typedef __PACKED(struct {
    uint8_t d[3];
}) int24_str;

typedef __PACKED(struct {
    char signature[4];

    uint32_t serial;
    uint16_t frameNumber;
    uint32_t time_ms;
    int32_t reserved0A;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    int24_str reserved1A;
    int24_str reserved1D;
    int24_str reserved20;

    uint8_t _dummy_[182];

    uint16_t crc;
}) LMS6_RawFrame;


typedef struct {
    float temperature;                  /* Temperature [°C] main (air) sensor */
    float pressure;                     /* Atmospheric pressure [hPa] */
    float humidity;
} LMS6_CookedMetrology;


typedef struct {
    double tow;
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
} LMS6_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _LMS6_InstanceData {
    struct _LMS6_InstanceData *next;
    uint32_t id;
    uint32_t serial;
    uint32_t lastUpdated;
    float rssi;
    uint64_t realTime;
    float rxFrequencyMHz;
    uint32_t frameNumber;

    LMS6_CookedGps gps;
    LMS6_CookedMetrology metro;
} LMS6_InstanceData;


/* Iterate through instances */
bool _LMS6_iterateInstance (LMS6_InstanceData **instance);

/* Remove an instance from the chain */
void _LMS6_deleteInstance (LMS6_InstanceData *instance);

/* Check CRC of a data frame */
_Bool _LMS6_checkCRC (void *buffer, int length, uint16_t receivedCRC);
/* Reed-Solomon error correction */
LPCLIB_Result _LMS6_checkReedSolomon (uint8_t rawFrame[], int *pNumErrors);

LPCLIB_Result _LMS6_processConfigBlock (
        const LMS6_RawFrame *rawConfig,
        LMS6_InstanceData **instancePointer);

LPCLIB_Result _LMS6_processPayload (
        const LMS6_RawFrame *payload,
        LMS6_CookedGps *cookedGps,
        LMS6_CookedMetrology *cookedMetro);

#ifdef __cplusplus
}
#endif
#endif
